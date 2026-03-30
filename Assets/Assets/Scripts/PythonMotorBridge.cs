using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using SProcess = System.Diagnostics.Process;
using SProcessStartInfo = System.Diagnostics.ProcessStartInfo;
using Debug = UnityEngine.Debug;

public class PythonMotorBridge : MonoBehaviour
{
    [Header("Python")]
    public string pythonExecutable = "python";
    public string scriptPath = "Assets/Python/main.py";
    public string pythonMode = "unity_demo";
    public bool useScriptDirectoryAsWorkingDir = true;

    [Header("Startup Config")]
    public bool configureOnStart = true;
    public bool useSizeVelocityMode = true;      // true -> VSIZE, false -> VAPPL
    public bool autoReverseOnAnimDone = false;   // true -> AUTOREV1, false -> AUTOREV0

    private SProcess proc;
    private StreamWriter pyIn;
    private StreamReader pyOut;
    private StreamReader pyErr;

    private bool bootReady = false;
    private bool isBooting = false;
    private Task bootTask = null;

    private readonly SemaphoreSlim _cmdLock = new SemaphoreSlim(1, 1);

    async void Start()
    {
        Debug.Log("[PYTHON] starting...");
        await EnsureStartedAsync();

        if (configureOnStart)
        {
            await EnsureConfiguredAsync();
        }
    }

    void OnDestroy()
    {
        StopPythonProcess();
        _cmdLock?.Dispose();
    }

    private string ResolveScriptPath()
    {
        string raw = (scriptPath ?? string.Empty).Trim();
        string normalized = raw.Replace('\\', '/');

        if (Path.IsPathRooted(normalized))
            return Path.GetFullPath(normalized);

        string projectRoot = Directory.GetParent(Application.dataPath)?.FullName ?? Environment.CurrentDirectory;
        string trimmedAssetsPrefix = normalized;
        if (trimmedAssetsPrefix.StartsWith("Assets/", StringComparison.OrdinalIgnoreCase))
            trimmedAssetsPrefix = trimmedAssetsPrefix.Substring("Assets/".Length);

        var candidates = new List<string>
        {
            Path.GetFullPath(Path.Combine(projectRoot, normalized)),
            Path.GetFullPath(Path.Combine(Application.dataPath, normalized)),
            Path.GetFullPath(Path.Combine(Application.dataPath, trimmedAssetsPrefix)),
            Path.GetFullPath(Path.Combine(Application.dataPath, "Assets", trimmedAssetsPrefix))
        };

        foreach (string candidate in candidates)
        {
            if (File.Exists(candidate))
                return candidate;
        }

        // Return a sensible default for logging/startup error handling.
        return candidates[0];
    }

    public async Task EnsureConfiguredAsync()
    {
        // string v = await SetVelocityModeAsync(useSizeVelocityMode);
        // if (v != "K")
        //     Debug.LogError("[PythonBridge] SetVelocityModeAsync failed: " + v);

        string a = await SetAutoReverseModeAsync(autoReverseOnAnimDone);
        if (a != "K")
            Debug.LogError("[PythonBridge] SetAutoReverseModeAsync failed: " + a);
    }

    private bool IsBootReadyAndProcessRunning()
    {
        try
        {
            return bootReady && proc != null && !proc.HasExited;
        }
        catch (InvalidOperationException)
        {
            return false;
        }
    }

    private bool IsProcessRunning()
    {
        try
        {
            return proc != null && !proc.HasExited;
        }
        catch (InvalidOperationException)
        {
            return false;
        }
    }

    private async Task EnsureStartedAsync()
    {
        if (IsBootReadyAndProcessRunning())
            return;

        if (isBooting && bootTask != null)
        {
            await bootTask;
            return;
        }

        bootTask = StartPythonProcessAsync();
        await bootTask;
    }

    private async Task StartPythonProcessAsync()
    {
        if (IsBootReadyAndProcessRunning())
            return;

        isBooting = true;
        bootReady = false;

        try
        {
            string script = ResolveScriptPath();
            if (!File.Exists(script))
            {
                Debug.LogError("[PythonBridge] Python script not found: " + script + ". Check scriptPath in inspector.");
                return;
            }

            string workingDirectory = useScriptDirectoryAsWorkingDir
                ? (Path.GetDirectoryName(script) ?? Environment.CurrentDirectory)
                : Environment.CurrentDirectory;

            if (useScriptDirectoryAsWorkingDir && !Directory.Exists(workingDirectory))
            {
                Debug.LogWarning("[PythonBridge] Invalid working directory, fallback to current: " + workingDirectory);
                workingDirectory = Environment.CurrentDirectory;
            }

            var psi = new SProcessStartInfo
            {
                FileName = pythonExecutable,
                Arguments = $"\"{script}\" {pythonMode}",
                UseShellExecute = false,
                RedirectStandardInput = true,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
                WorkingDirectory = workingDirectory
            };

            Debug.Log($"[PythonBridge] Run: {psi.FileName} {psi.Arguments}");
            Debug.Log($"[PythonBridge] WorkingDirectory = {psi.WorkingDirectory}");

            var tempProc = new SProcess { StartInfo = psi, EnableRaisingEvents = true };
            if (!tempProc.Start())
            {
                Debug.LogError("[PythonBridge] 無法啟動 Python");
                tempProc.Dispose();
                return;
            }
            
            proc = tempProc;

            pyIn = proc.StandardInput;
            pyOut = proc.StandardOutput;
            pyErr = proc.StandardError;

            _ = Task.Run(async () =>
            {
                try
                {
                    while (IsProcessRunning())
                    {
                        string errLine = await pyErr.ReadLineAsync();
                        if (errLine == null) break;
                        Debug.LogWarning("[Python][stderr] " + errLine);
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogWarning("[PythonBridge] stderr reader stopped: " + ex.Message);
                }
            });

            string bootLine = await pyOut.ReadLineAsync();
            Debug.Log("[PythonBridge] boot <- " + bootLine);

            if (bootLine == "K")
            {
                bootReady = true;
                Debug.Log("[PythonBridge] Python boot ready.");
            }
            else
            {
                Debug.LogError("[PythonBridge] Python boot failed. First line = " + (bootLine ?? "null"));
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("[PythonBridge] StartPythonProcessAsync exception: " + ex.Message);
            if (proc != null)
            {
                proc.Dispose();
                proc = null;
            }
        }
        finally
        {
            isBooting = false;
        }
    }

    private void StopPythonProcess()
    {
        try
        {
            if (pyIn != null)
            {
                pyIn.WriteLine("Q");
                pyIn.Flush();
                pyIn.Close();
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning("[PythonBridge] Stop write Q failed: " + ex.Message);
        }

        try
        {
            if (IsProcessRunning())
                proc.Kill();
        }
        catch (Exception ex)
        {
            Debug.LogWarning("[PythonBridge] Kill process failed: " + ex.Message);
        }

        if (proc != null)
        {
            try { proc.Dispose(); } catch {}
        }

        pyIn = null;
        pyOut = null;
        pyErr = null;
        proc = null;
        bootReady = false;
        isBooting = false;
        bootTask = null;
    }

    private async Task<string> SendCommandAsync(string cmd)
    {
        await _cmdLock.WaitAsync();
        try
        {
            await EnsureStartedAsync();

            if (!IsBootReadyAndProcessRunning() || pyIn == null || pyOut == null)
            {
                Debug.LogError("[PythonBridge] Python process not ready.");
                return null;
            }

            Debug.Log("[PythonBridge] send -> " + cmd);
            await pyIn.WriteLineAsync(cmd);
            await pyIn.FlushAsync();

            string line = await pyOut.ReadLineAsync();
            Debug.Log("[PythonBridge] recv <- " + line);

            if (!string.IsNullOrEmpty(line) && line.StartsWith("E"))
                Debug.LogError("[PythonBridge] Python error: " + line);

            return line;
        }
        catch (Exception ex)
        {
            Debug.LogError("[PythonBridge] SendCommandAsync exception: " + ex.Message);
            return null;
        }
        finally
        {
            _cmdLock.Release();
        }
    }

    public async Task<string> RecordBaselineAsync()
    {
        return await SendCommandAsync("B");
    }

    public async Task<string> GoInitialAsync()
    {
        return await SendCommandAsync("N");
    }

    public async Task<string> SetVelocityModeAsync(bool useSizeMode)
    {
        useSizeVelocityMode = useSizeMode;
        return await SendCommandAsync(useSizeMode ? "VSIZE" : "VAPPL");
    }

    public async Task<string> SetAutoReverseModeAsync(bool enabled)
    {
        autoReverseOnAnimDone = enabled;
        return await SendCommandAsync(enabled ? "AUTOREV1" : "AUTOREV0");
    }

    // public async Task<string> SendTrialRelativeAsync(int bDeg, int eDeg)
    // {
    //     int bMag = Mathf.Abs(bDeg);
    //     int eMag = Mathf.Abs(eDeg);

    //     string cmd = "";
    //     if (bMag > 0) cmd += $"b{bMag}";
    //     if (eMag > 0) cmd += $"e{eMag}";

    //     if (string.IsNullOrEmpty(cmd))
    //         return "K";

    //     return await SendCommandAsync(cmd);
    // }
    public async Task<string> SendTrialRelativeAsync(int bDeg, int eDeg)
    {
        string cmd = "";

        if (bDeg != 0) cmd += $"b{bDeg}";
        if (eDeg != 0) cmd += $"e{eDeg}";

        if (string.IsNullOrEmpty(cmd))
            return "K";

        return await SendCommandAsync(cmd);
    }

    public async Task<string> SendAnimDoneAsync()
    {
        return await SendCommandAsync("K");
    }

    // Backward-compatible wrappers for older controllers (e.g. BallFlyController)

    public async Task<string> SendBDeltaAsync(int deg)
    {
        return await SendTrialRelativeAsync(deg, 0);
    }

    public async Task<string> SendEDeltaAsync(int deg)
    {
        return await SendTrialRelativeAsync(0, deg);
    }

    public async Task<string> SendEventAsync(string evt)
    {
        if (string.IsNullOrWhiteSpace(evt))
            return "K";

        string e = evt.Trim().ToUpperInvariant();

        if (e == "ANIM_DONE" || e == "K")
            return await SendAnimDoneAsync();

        Debug.LogWarning("[PythonBridge] Unknown legacy event, ACK only: " + evt);
        return "K";
    }

    public async Task<string> QuitAsync()
    {
        return await SendCommandAsync("Q");
    }

    public async Task<string> GoFinishAsync()
    {
        return await SendCommandAsync("F");
    }

    public async Task<string> RebootAllMotorsAsync()
    {
        return await SendCommandAsync("REBOOT");
    }

    public async Task<string> SendRawCommandAsync(string cmd)
    {
        if (string.IsNullOrWhiteSpace(cmd))
            return "K";

        string normalized = cmd.Trim();

        if (string.Equals(normalized, "C", StringComparison.OrdinalIgnoreCase))
        {
            Debug.Log("[PythonBridge] Sending raw command C");
        }
        else if (string.Equals(normalized, "A", StringComparison.OrdinalIgnoreCase))
        {
            Debug.Log("[PythonBridge] Sending raw command A");
        }

        return await SendCommandAsync(normalized);
    }
}
