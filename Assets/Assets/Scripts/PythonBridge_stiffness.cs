using System;
using System.IO;
using System.Globalization;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using UnityEngine;
using Debug = UnityEngine.Debug;
using SProcess = System.Diagnostics.Process;
using SProcessStartInfo = System.Diagnostics.ProcessStartInfo;

public class PythonBridge_stiffness : MonoBehaviour
{
    [Header("Python")]
    public string pythonExecutable = "python";
    public string scriptPath = "Assets/Python/stiffness.py";
    public bool useScriptDirectoryAsWorkingDir = true;

    [Header("Runtime State (Read Only)")]
    public bool pythonStarted = false;
    public bool hardwareReady = false;   // 收到 K 後變 true
    public float latestForce = 0f;
    public float latestDisplacement = 0f;

    [Header("Debug")]
    public bool logStdout = false;
    public bool logStderr = true;

    private SProcess proc;
    private StreamWriter pyIn;
    private StreamReader pyOut;
    private StreamReader pyErr;

    private readonly ConcurrentQueue<string> stdoutQueue = new ConcurrentQueue<string>();
    private readonly ConcurrentQueue<string> stderrQueue = new ConcurrentQueue<string>();

    private bool isStarting = false;

    public event Action OnHardwareReady;
    public event Action<float> OnForceUpdated;
    public event Action<float> OnDisplacementUpdated;
    public event Action<string> OnPythonStderr;

    async void Start()
    {
        await EnsureStartedAsync();
    }

    void Update()
    {
        DrainStdoutQueue();
        DrainStderrQueue();
    }

    void OnDestroy()
    {
        StopPythonProcess();
    }

    // --------------------------------------------------
    // Process start / stop
    // --------------------------------------------------
    private string ResolveScriptPath()
    {
        string script = scriptPath;

        if (script.StartsWith("Assets/") || script.StartsWith("Assets\\"))
            script = script.Substring("Assets/".Length);

        if (!Path.IsPathRooted(script))
            script = Path.Combine(Application.dataPath, script);

        return Path.GetFullPath(script);
    }

    public async Task EnsureStartedAsync()
    {
        if (pythonStarted && proc != null && !proc.HasExited)
            return;

        if (isStarting)
            return;

        isStarting = true;

        try
        {
            string script = ResolveScriptPath();

            var psi = new SProcessStartInfo
            {
                FileName = pythonExecutable,
                Arguments = $"\"{script}\"",
                UseShellExecute = false,
                RedirectStandardInput = true,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
                WorkingDirectory = useScriptDirectoryAsWorkingDir
                    ? (Path.GetDirectoryName(script) ?? Environment.CurrentDirectory)
                    : Environment.CurrentDirectory
            };

            Debug.Log($"[PythonBridge] Run: {psi.FileName} {psi.Arguments}");
            Debug.Log($"[PythonBridge] WorkingDirectory: {psi.WorkingDirectory}");

            proc = new SProcess
            {
                StartInfo = psi,
                EnableRaisingEvents = true
            };

            if (!proc.Start())
            {
                Debug.LogError("[PythonBridge] Failed to start Python process.");
                return;
            }

            pyIn = proc.StandardInput;
            pyOut = proc.StandardOutput;
            pyErr = proc.StandardError;

            pythonStarted = true;
            hardwareReady = false;

            _ = Task.Run(async () =>
            {
                try
                {
                    while (proc != null && !proc.HasExited)
                    {
                        string line = await pyOut.ReadLineAsync();
                        if (line == null) break;
                        stdoutQueue.Enqueue(line);
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogWarning("[PythonBridge] stdout reader stopped: " + ex.Message);
                }
            });

            _ = Task.Run(async () =>
            {
                try
                {
                    while (proc != null && !proc.HasExited)
                    {
                        string line = await pyErr.ReadLineAsync();
                        if (line == null) break;
                        stderrQueue.Enqueue(line);
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogWarning("[PythonBridge] stderr reader stopped: " + ex.Message);
                }
            });

            await Task.Delay(100);
        }
        catch (Exception ex)
        {
            Debug.LogError("[PythonBridge] Start exception: " + ex.Message);
        }
        finally
        {
            isStarting = false;
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
            Debug.LogWarning("[PythonBridge] Failed to send Q: " + ex.Message);
        }

        try
        {
            if (proc != null && !proc.HasExited)
                proc.Kill();
        }
        catch (Exception ex)
        {
            Debug.LogWarning("[PythonBridge] Kill failed: " + ex.Message);
        }

        pyIn = null;
        pyOut = null;
        pyErr = null;
        proc = null;

        pythonStarted = false;
        hardwareReady = false;
    }

    // --------------------------------------------------
    // Read Python output
    // --------------------------------------------------
    private void DrainStdoutQueue()
    {
        while (stdoutQueue.TryDequeue(out string line))
        {
            if (logStdout)
                Debug.Log("[Python stdout] " + line);

            HandleStdoutLine(line);
        }
    }

    private void DrainStderrQueue()
    {
        while (stderrQueue.TryDequeue(out string line))
        {
            if (logStderr)
                Debug.LogWarning("[Python stderr] " + line);

            OnPythonStderr?.Invoke(line);
        }
    }

    private void HandleStdoutLine(string line)
    {
        if (string.IsNullOrWhiteSpace(line))
            return;

        line = line.Trim();

        if (line == "K")
        {
            hardwareReady = true;
            Debug.Log("[PythonBridge] Hardware ready (K)");
            OnHardwareReady?.Invoke();
            return;
        }

        if (line.StartsWith("F,"))
        {
            string s = line.Substring(2).Trim();
            if (float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float forceValue))
            {
                latestForce = forceValue;
                OnForceUpdated?.Invoke(forceValue);
                return;
            }
        }

        if (line.StartsWith("X,"))
        {
            string s = line.Substring(2).Trim();
            if (float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float dispValue))
            {
                latestDisplacement = dispValue;
                OnDisplacementUpdated?.Invoke(dispValue);
                return;
            }
        }

        Debug.LogWarning("[PythonBridge] Unrecognized stdout line: " + line);
    }

    // --------------------------------------------------
    // Send command
    // --------------------------------------------------
    private async Task SendCommandAsync(string cmd)
    {
        await EnsureStartedAsync();

        if (proc == null || proc.HasExited || pyIn == null)
        {
            Debug.LogError("[PythonBridge] Python process not ready.");
            return;
        }

        try
        {
            await pyIn.WriteLineAsync(cmd);
            await pyIn.FlushAsync();
            Debug.Log("[PythonBridge] -> " + cmd);
        }
        catch (Exception ex)
        {
            Debug.LogError("[PythonBridge] SendCommandAsync exception: " + ex.Message);
        }
    }

    // --------------------------------------------------
    // Public API
    // --------------------------------------------------
    public Task SendBaselineAsync() => SendCommandAsync("B");
    public Task SendInitialAsync() => SendCommandAsync("N");
    public Task SendNextAsync() => SendCommandAsync("M");
    public Task SendQuitAsync() => SendCommandAsync("Q");

    public Task SendTrialAsync(string trialCode)
    {
        return SendCommandAsync(trialCode);
    }
}