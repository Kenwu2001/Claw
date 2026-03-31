using UnityEngine;

public class TriggerCat : MonoBehaviour
{
    [Header("Hand Detection")]
    public Transform handRoot;
    public string handTag = "Hand";

    [Header("Python Bridge")]
    public PythonMotorBridge pythonBridge;

    [Header("Raw Command")]
    [Tooltip("碰到手時送出的原始指令")]
    public string commandOnTrigger = "a";

    [Tooltip("true: 每次手進入都可觸發；false: 只觸發一次")]
    public bool allowRepeatTrigger = true;

    [Header("Pre-Send Setup")]
    [Tooltip("送角度前先強制切到 VAPPL（仿照 BallFly PlayOnce）")]
    public bool forceApplVelocityBeforeSend = true;

    [Tooltip("送角度前先回初始位置 N")]
    public bool goInitialBeforeSend = false;

    [Header("Baseline Guard")]
    [Tooltip("進入關卡後需先按 B 記錄 baseline，才能觸發")]
    public bool requireBaselineBeforeTrigger = true;

    public bool enableBaselineHotkey = true;

    private bool _alreadyTriggered;
    private bool _isSending;

    private bool IsHandHit(Transform other)
    {
        if (other == null) return false;

        if (handRoot != null)
            return other == handRoot || other.IsChildOf(handRoot);

        return other.CompareTag(handTag);
    }

    private void Update()
    {
        if (!enableBaselineHotkey || !Input.GetKeyDown(KeyCode.B))
            return;

        RecordBaselineFromUnity();
    }

    private void OnTriggerEnter(Collider other)
    {
        if (!IsHandHit(other.transform)) return;
        if (requireBaselineBeforeTrigger && !BaselineRequirementState.IsReady)
        {
            Debug.LogWarning("[TriggerCat] Please press B to record baseline before triggering.");
            return;
        }

        Debug.Log("[TriggerCat] Hit hand (trigger): " + other.name);

        if (_isSending) return;
        if (!allowRepeatTrigger && _alreadyTriggered) return;

        SendCommand();
    }

    private async void RecordBaselineFromUnity()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[TriggerCat] pythonBridge is null.");
            return;
        }

        if (!BaselineRequirementState.TryBeginRecording())
            return;

        string result = await pythonBridge.RecordBaselineAsync();
        if (result == "K")
        {
            BaselineRequirementState.CompleteRecording(true);
            Debug.Log("[TriggerCat] Baseline recorded successfully.");
        }
        else
        {
            BaselineRequirementState.CompleteRecording(false);
            Debug.LogError("[TriggerCat] Record baseline failed: " + result);
        }
    }

    private async void SendCommand()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[TriggerCat] pythonBridge is null.");
            return;
        }

        _isSending = true;
        _alreadyTriggered = true;

        if (forceApplVelocityBeforeSend)
        {
            string modeResult = await pythonBridge.SetVelocityModeAsync(false); // false -> VAPPL
            if (modeResult != "K")
            {
                Debug.LogError("[TriggerCat] Set VAPPL failed: " + modeResult);
                _isSending = false;
                return;
            }
        }

        if (goInitialBeforeSend)
        {
            string initResult = await pythonBridge.GoInitialAsync();
            if (initResult != "K")
            {
                Debug.LogError("[TriggerCat] GoInitial failed: " + initResult);
                _isSending = false;
                return;
            }
        }

        string cmd = string.IsNullOrWhiteSpace(commandOnTrigger) ? "a" : commandOnTrigger.Trim().ToLowerInvariant();
        if (string.IsNullOrEmpty(cmd))
        {
            Debug.LogWarning("[TriggerCat] command is empty, skip sending.");
            _isSending = false;
            return;
        }

        string result = await pythonBridge.SendRawCommandAsync(cmd);
        if (string.IsNullOrEmpty(result) || !result.StartsWith("K"))
        {
            Debug.LogError($"[TriggerCat] Send failed: cmd={cmd}, result={result}");
        }
        else
        {
            Debug.Log($"[TriggerCat] Send success: cmd={cmd}, result={result}");
        }

        _isSending = false;
    }

    private void OnTriggerExit(Collider other)
    {
        if (!allowRepeatTrigger) return;
        if (!IsHandHit(other.transform)) return;

        _alreadyTriggered = false;
    }
}
