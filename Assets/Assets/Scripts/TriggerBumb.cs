using UnityEngine;

public class TriggerBumb : MonoBehaviour
{
    [Header("Hand Detection")]
    public Transform handRoot;
    public string handTag = "Hand";

    [Header("Python Bridge")]
    public PythonMotorBridge pythonBridge;

    [Header("Raw Commands")]
    [Tooltip("放大期間持續傳送的指令")]
    public string commandDuringGrow = "f";

    [Tooltip("爆炸瞬間傳送的指令")]
    public string commandOnExplode = "g";

    [Tooltip("放大期間送指令的間隔秒數")]
    public float growCommandIntervalSeconds = 0.1f;

    [Tooltip("true: 每次手進入都可觸發；false: 只觸發一次")]
    public bool allowRepeatTrigger = true;

    [Header("Pre-Send Setup")]
    [Tooltip("送角度前先強制切到 VAPPL（仿照 BallFly PlayOnce）")]
    public bool forceApplVelocityBeforeSend = true;

    [Tooltip("送角度前先回初始位置 N")]
    public bool goInitialBeforeSend = false;

    private bool _alreadyTriggered;
    private bool _isSending;

    [Header("Boom Sequence")]
    public GameObject boom;
    public float growDurationSeconds = 5f;
    public float growMultiplier = 2f;

    [Header("Activation Guard")]
    [Tooltip("物件啟用後，延遲幾秒才允許觸發，避免切關瞬間就被手重疊觸發")]
    public float armDelaySeconds = 0.25f;

    [Tooltip("每次啟用時自動把 boom 關閉")]
    public bool hideBoomOnEnable = true;

    private bool _sequenceStarted;
    private Vector3 _initialScale;
    private float _armedAt;

    private void Awake()
    {
        _initialScale = transform.localScale;
    }

    private void OnEnable()
    {
        _sequenceStarted = false;
        _alreadyTriggered = false;
        _isSending = false;
        _armedAt = Time.time + Mathf.Max(0f, armDelaySeconds);

        transform.localScale = _initialScale;

        if (hideBoomOnEnable && boom != null)
            boom.SetActive(false);
    }

    private bool IsHandHit(Transform other)
    {
        if (other == null) return false;

        if (handRoot != null)
            return other == handRoot || other.IsChildOf(handRoot);

        return other.CompareTag(handTag);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (Time.time < _armedAt) return;
        if (!IsHandHit(other.transform)) return;

        Debug.Log("[TriggerBumb] Hit hand (trigger): " + other.name);

        if (_isSending) return;
        if (_sequenceStarted) return;
        if (!allowRepeatTrigger && _alreadyTriggered) return;

        StartCoroutine(GrowThenBoomCo());
    }

    private System.Collections.IEnumerator GrowThenBoomCo()
    {
        _sequenceStarted = true;
        _isSending = true;
        _alreadyTriggered = true;

        Vector3 startScale = transform.localScale;
        Vector3 targetScale = startScale * Mathf.Max(0f, growMultiplier);
        float duration = Mathf.Max(0.01f, growDurationSeconds);
        float commandInterval = Mathf.Max(0.01f, growCommandIntervalSeconds);
        float elapsed = 0f;
        float nextCommandTime = 0f;

        while (elapsed < duration)
        {
            if (elapsed >= nextCommandTime)
            {
                SendRawCommand(commandDuringGrow);
                nextCommandTime += commandInterval;
            }

            elapsed += Time.deltaTime;
            float t = Mathf.Clamp01(elapsed / duration);
            transform.localScale = Vector3.Lerp(startScale, targetScale, t);
            yield return null;
        }

        transform.localScale = targetScale;

        SendRawCommand(commandOnExplode);

        if (boom != null)
        {
            boom.SetActive(true);
        }
        else
        {
            Debug.LogWarning("[TriggerBumb] boom is not assigned.");
        }

        _isSending = false;
        gameObject.SetActive(false);
    }

    private async void SendRawCommand(string cmd)
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[TriggerBumb] pythonBridge is null.");
            return;
        }

        if (forceApplVelocityBeforeSend)
        {
            string modeResult = await pythonBridge.SetVelocityModeAsync(false); // false -> VAPPL
            if (modeResult != "K")
            {
                Debug.LogError("[TriggerBumb] Set VAPPL failed: " + modeResult);
                return;
            }
        }

        if (goInitialBeforeSend)
        {
            string initResult = await pythonBridge.GoInitialAsync();
            if (initResult != "K")
            {
                Debug.LogError("[TriggerBumb] GoInitial failed: " + initResult);
                return;
            }
        }

        string normalized = string.IsNullOrWhiteSpace(cmd) ? string.Empty : cmd.Trim();
        if (string.IsNullOrEmpty(normalized))
        {
            Debug.LogWarning("[TriggerBumb] command is empty, skip sending.");
            return;
        }

        string result = await pythonBridge.SendRawCommandAsync(normalized);
        if (string.IsNullOrEmpty(result) || !result.StartsWith("K"))
        {
            Debug.LogError($"[TriggerBumb] Send failed: cmd={normalized}, result={result}");
        }
        else
        {
            Debug.Log($"[TriggerBumb] Send success: cmd={normalized}, result={result}");
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (!allowRepeatTrigger) return;
        if (!IsHandHit(other.transform)) return;

        _alreadyTriggered = false;
    }
}
