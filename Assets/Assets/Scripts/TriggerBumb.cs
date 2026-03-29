using UnityEngine;

public class TriggerBumb : MonoBehaviour
{
    [Header("Hand Detection")]
    public Transform handRoot;
    public string handTag = "Hand";

    [Header("Python Bridge")]
    public PythonMotorBridge pythonBridge;

    [Header("Send Command (BallFly style)")]
    [Tooltip("送到 B 軸的相對角度，例如 50 會送 b50")]
    public int bDelta = 100;

    [Tooltip("送到 E 軸的相對角度，例如 50 會送 e50")]
    public int eDelta = 0;

    [Tooltip("true: 每次手進入都可觸發；false: 只觸發一次")]
    public bool allowRepeatTrigger = true;

    [Header("Pre-Send Setup")]
    [Tooltip("送角度前先強制切到 VAPPL（仿照 BallFly PlayOnce）")]
    public bool forceApplVelocityBeforeSend = true;

    [Tooltip("送角度前先回初始位置 N")]
    public bool goInitialBeforeSend = false;

    private bool _alreadyTriggered;
    private bool _isSending;

    private bool IsHandHit(Transform other)
    {
        if (other == null) return false;

        if (handRoot != null)
            return other == handRoot || other.IsChildOf(handRoot);

        return other.CompareTag(handTag);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (!IsHandHit(other.transform)) return;

        Debug.Log("[TriggerBumb] Hit hand (trigger): " + other.name);

        if (_isSending) return;
        if (!allowRepeatTrigger && _alreadyTriggered) return;

        SendMotorCommand();
    }

    private async void SendMotorCommand()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[TriggerBumb] pythonBridge is null.");
            return;
        }

        _isSending = true;
        _alreadyTriggered = true;

        int sendB = bDelta;
        int sendE = eDelta;

        if (forceApplVelocityBeforeSend)
        {
            string modeResult = await pythonBridge.SetVelocityModeAsync(false); // false -> VAPPL
            if (modeResult != "K")
            {
                Debug.LogError("[TriggerBumb] Set VAPPL failed: " + modeResult);
                _isSending = false;
                return;
            }
        }

        if (goInitialBeforeSend)
        {
            string initResult = await pythonBridge.GoInitialAsync();
            if (initResult != "K")
            {
                Debug.LogError("[TriggerBumb] GoInitial failed: " + initResult);
                _isSending = false;
                return;
            }
        }

        if (sendB == 0 && sendE == 0)
        {
            Debug.LogWarning("[TriggerBumb] bDelta and eDelta are both 0, skip sending.");
            _isSending = false;
            return;
        }

        string result = await pythonBridge.SendTrialRelativeAsync(sendB, sendE);
        if (string.IsNullOrEmpty(result) || !result.StartsWith("K"))
        {
            Debug.LogError($"[TriggerBumb] Send failed: {result} (b={sendB}, e={sendE})");
        }
        else
        {
            Debug.Log($"[TriggerBumb] Send success: b={sendB}, e={sendE}, result={result}");
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
