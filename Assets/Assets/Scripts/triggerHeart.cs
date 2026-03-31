using UnityEngine;

public class triggerHeart : MonoBehaviour
{
    [Header("Hand Detection")]
    public Transform handRoot;
    public string handTag = "Hand";

    [Header("Python Bridge")]
    public PythonMotorBridge pythonBridge;

    [Header("Send Command")]
    public string commandOnTrigger = "A";

    [Tooltip("true: 每次手進入都可觸發；false: 只觸發一次")]
    public bool allowRepeatTrigger = true;

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
            Debug.LogWarning("[triggerHeart] Please press B to record baseline before triggering.");
            return;
        }

        Debug.Log("[triggerHeart] Hit hand (trigger): " + other.name);

        if (_isSending) return;
        if (!allowRepeatTrigger && _alreadyTriggered) return;

        SendCommand();
    }

    private async void RecordBaselineFromUnity()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[triggerHeart] pythonBridge is null.");
            return;
        }

        if (!BaselineRequirementState.TryBeginRecording())
            return;

        string result = await pythonBridge.RecordBaselineAsync();
        if (result == "K")
        {
            BaselineRequirementState.CompleteRecording(true);
            Debug.Log("[triggerHeart] Baseline recorded successfully.");
        }
        else
        {
            BaselineRequirementState.CompleteRecording(false);
            Debug.LogError("[triggerHeart] Record baseline failed: " + result);
        }
    }

    private async void SendCommand()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[triggerHeart] pythonBridge is null.");
            return;
        }

        _isSending = true;
        _alreadyTriggered = true;

        string cmd = string.IsNullOrWhiteSpace(commandOnTrigger) ? "A" : commandOnTrigger.Trim();
        string result = await pythonBridge.SendRawCommandAsync(cmd);

        if (string.IsNullOrEmpty(result) || !result.StartsWith("K"))
        {
            Debug.LogError($"[triggerHeart] Send failed: cmd={cmd}, result={result}");
        }
        else
        {
            Debug.Log($"[triggerHeart] Send success: cmd={cmd}, result={result}");
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
