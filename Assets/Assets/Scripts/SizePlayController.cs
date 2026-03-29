using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using TMPro;
using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine.EventSystems;
using System.Threading.Tasks;
using System.Linq;

public class SizePlayController : MonoBehaviour
{
    // ───────────── Targets ─────────────
    [Header("Targets")]
    [Tooltip("主球（播放階段 / 調整階段循環動畫都用這一顆）")]
    public Transform ball;

    [Tooltip("靜態球（顯示目標大小；尺寸 = 動畫終點，無任何縮放係數）")]
    public Transform targetPreviewBall;

    // ───────────── Animation ─────────────
    [Header("Animation")]
    [Tooltip("initial 尺寸（ball.localScale = Vector3.one * initialSize）")]
    public float initialSize = 1.0f;

    [Tooltip("避免縮到 0 或負數")]
    public float minTargetSize = 0.05f;

    [Tooltip("固定模式下主動畫時間（秒）")]
    public float duration = 0.8f;

    [Tooltip("播放前在 initial 停留（秒）")]
    public float holdAtStartSeconds = 0.0f;

    [Tooltip("補間曲線")]
    public AnimationCurve curve = AnimationCurve.EaseInOut(0, 0, 1, 1);

    [Tooltip("調整階段循環動畫：到達終點後停留秒數")]
    public float adjustLoopEndHold = 0.3f;

    [Tooltip("每個 trial 第一次播放時使用的固定預設變化量（百分比）")]
    public float defaultBaseChangePercent = 1f;

    [Header("Auto Reverse")]
    [Tooltip("AUTOREV1 時，動畫到終點後要停留多久才送 K 回 initial")]
    public float autoReverseHoldSeconds = 1.0f;

    // ───────────── Motor-Visual Sync ─────────────
    [Header("Motor-Visual Sync")]
    [Tooltip("true: 視覺動畫時間跟馬達 mode 校正速度同步；false: 使用固定 duration")]
    public bool useMotorSyncedDuration = true;

    [Tooltip("true: 啟動時選用 SIZE_V；false: 啟動時選用 APPL_V")]
    public bool useSizeVelocityMode = true;

    [Tooltip("true: 動畫結束送 K，Python 自動 reverse 回 initial；false: 馬達停在終點直到下一次 Play / Next 前送 N")]
    public bool autoReverseOnAnimDone = false;

    [Tooltip("B group 在 SIZE mode 下的校正角速度（deg/s）")]
    public float sizeModeDegPerSecB = 40f;

    [Tooltip("E group 在 SIZE mode 下的校正角速度（deg/s）")]
    public float sizeModeDegPerSecE = 40f;

    [Tooltip("B group 在 APPL mode 下的校正角速度（deg/s）")]
    public float applModeDegPerSecB = 120f;

    [Tooltip("E group 在 APPL mode 下的校正角速度（deg/s）")]
    public float applModeDegPerSecE = 120f;

    // ───────────── UI ─────────────
    [Header("UI")]
    public Button playButton;
    public Button lowerButton;
    public Button bestButton;
    public Button upperButton;
    public Button applyBestValueButton;
    public Button resetValueButton;
    public Button nextTrialButton;

    [Tooltip("防護訊息（未填滿三鍵時顯示）")]
    public TextMeshProUGUI nextGuardLabel;
    public string nextGuardMessage = "請先設定 Best，再完成 Lower 與 Upper 後再繼續。";

    [Tooltip("（可選）試次資訊")]
    public TextMeshProUGUI trialInfoLabel;

    [Header("Hotkeys")]
    public bool enableHotkeys = true;
    public TextMeshProUGUI hotkeyHelpLabel;

    [Tooltip("W/S(↑/↓)=±1%，E/D=±5%，限制範圍")]
    public float stepSmall = 1f;
    public float stepLarge = 5f;
    public float minPercent = -100f, maxPercent = 100f;

    [Header("Snap Button Visual")]
    public Color snapIdleColor = new Color32(84, 110, 122, 255);
    public Color snapDoneColor = new Color32(46, 125, 50, 255);
    public Color snapTextColor = Color.white;
    [Range(0f, 1f)] public float disabledButtonAlpha = 0.45f;

    // ───────────── Python ─────────────
    [Header("Python Bridge")]
    public PythonMotorBridge pythonBridge;
    public bool sendAngleOnPlay = true;

    // // ───────────── Finger 2 motor angle ─────────────
    [Header("Angle Conversion")]
    [Tooltip("送給 Python 前，ab_ad_angle 乘上的倍率")]
    public float abAdScale = 1.5689f;

    [Tooltip("送給 Python 前，ex_angle 乘上的倍率")]
    public float exScale = 9.1618f;

    // ───────────── CSV / Participant / Blocks ─────────────
    [Header("Spreadsheet (CSV)")]
    [Tooltip("每位 user 各自的 trial 表，例如 Assets/Data/trial_size/Trial_Size_{UserID}.csv")]
    public string spreadsheetPath = "Assets/Data/trial_size/Trial_Size_{UserID}.csv";

    [Header("Participant")]
    public int userId = 1;

    [Header("Start Trial")]
    [Tooltip("從第幾個 global trial 開始（1-based）。例如填 1 = 從第一筆開始，填 13 = 從第二個 block 的第一筆開始。")]
    public int startFromTrialNumber = 1;

    [Header("CSV Columns")]
    public string userIdColumn = "UserID";
    public string trialNoColumn = "Trial_no";
    public string blockColumn = "Block";
    public string methodColumn = "Method";
    public string objectTypeColumn = "ObjectType";

    [Tooltip("欄位：每列的馬達角度")]
    public string abAdColumn = "ab_ad_angle";
    public string exColumn = "ex_angle";

    public string lowerColumn = "lowerbound";
    public string bestColumn = "Size Change(%)";
    public string upperColumn = "upperbound";
    public string durationColumn = "duration";

    [Header("Block Navigation")]
    [Tooltip("true = 依 CSV 的 Block 欄位自動分 block；建議開啟")]
    public bool useBlockColumnFromCsv = true;

    public int currentBlock = 0;   // 這裡是 block 的索引，不是 CSV 裡的 Block 數字
    public int trialInBlock = 0;

    [Header("Rest Panel")]
    public GameObject restPanel;
    public TextMeshProUGUI restMessageLabel;
    public Button restNextButton;

    // ───────────── Internal States ─────────────
    private bool isAnimating = false;
    private bool adjustmentsUnlocked = false;
    private bool _firstPlayThisTrial = true;
    private float _deltaPercent = 0f;
    private float DeltaPercent => _deltaPercent;
    private readonly Dictionary<Button, Coroutine> _buttonFlashCos = new Dictionary<Button, Coroutine>();

    private float _currentAnimDuration = 0.8f;

    // CSV cache（只處理目前 user 的單一 CSV）
    private List<string> _headers = new List<string>();
    private List<string[]> _rows = new List<string[]>();
    private Dictionary<string, int> _colIndex = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);

    // block mapping（針對當前 user）
    private List<int> _uniqueBlocks = new List<int>();                    // 例如 [1,2,3,4,5,6]
    private Dictionary<int, List<int>> _blockToUserRowIndices = new Dictionary<int, List<int>>();
    // key = CSV 裡實際 block 編號，value = 該 block 對應的 _rows 索引清單

    // 三個暫存截值（儲存的是 deltaPercent）
    private float? _snapLower = null;
    private float? _snapBest = null;
    private float? _snapUpper = null;

    private Coroutine _adjustCo;
    private float trialStartTime;

    // ───────────── Unity lifecycle ─────────────
    async void Start()
    {
        if (!ball)
        {
            Debug.LogError("[SizePlayController] Ball 未指定");
            enabled = false;
            return;
        }

        ApplyScale(ball, initialSize);

        if (playButton) playButton.onClick.AddListener(PlayOnce);
        if (lowerButton) lowerButton.onClick.AddListener(SnapLower);
        if (bestButton) bestButton.onClick.AddListener(SnapBest);
        if (upperButton) upperButton.onClick.AddListener(SnapUpper);
        if (applyBestValueButton) applyBestValueButton.onClick.AddListener(ApplyRecordedBestValue);
        if (resetValueButton) resetValueButton.onClick.AddListener(ResetToInitialValue);
        if (nextTrialButton) nextTrialButton.onClick.AddListener(TryNext);
        if (restNextButton) restNextButton.onClick.AddListener(GoNextBlock);

        InitSnapButton(lowerButton, "Lowerbound", snapIdleColor, snapTextColor);
        InitSnapButton(bestButton, "Best", snapIdleColor, snapTextColor);
        InitSnapButton(upperButton, "Upperbound", snapIdleColor, snapTextColor);

        LoadCsvOrCreate();
        BuildBlockMapFromFilteredRows();

        // currentBlock = Mathf.Clamp(currentBlock, 0, Mathf.Max(0, BlockCount - 1));
        // trialInBlock = 0;
        ApplyStartTrialNumber();

        _firstPlayThisTrial = true;
        UpdateTrialInfoLabel();
        BuildHotkeyHelpText();

        adjustmentsUnlocked = false;
        if (targetPreviewBall) targetPreviewBall.gameObject.SetActive(false);
        if (restPanel) restPanel.SetActive(false);

        ResetSnapFlowForCurrentTrial();
        LoadSnapshotsForCurrentTrial();
        SetAllControls(true);

        if (pythonBridge != null)
        {
            string v = await pythonBridge.SetVelocityModeAsync(useSizeVelocityMode);
            if (v != "K") Debug.LogError("[Trial] SetVelocityModeAsync failed: " + v);

            string a = await pythonBridge.SetAutoReverseModeAsync(autoReverseOnAnimDone);
            if (a != "K") Debug.LogError("[Trial] SetAutoReverseModeAsync failed: " + a);
        }
    }

    void Update()
    {
        if (!enableHotkeys) return;

        if (EventSystem.current != null)
        {
            var go = EventSystem.current.currentSelectedGameObject;
            if (go && (go.GetComponent<TMP_InputField>() || go.GetComponent<InputField>()))
                return;
        }

        if (Input.GetKeyDown(KeyCode.B))
            RecordBaselineFromUnity();

        if (Input.GetKeyDown(KeyCode.I))
            GoInitialFromUnity();

        if (Input.GetKeyDown(KeyCode.O))
            RebootAllMotorsFromUnity();

        if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter) || Input.GetKeyDown(KeyCode.V))
            TryNext();

        if (Input.GetKeyDown(KeyCode.Space) && playButton && playButton.interactable)
            PlayOnce();

        if (adjustmentsUnlocked && !(restPanel && restPanel.activeSelf))
        {
            if (Input.GetKeyDown(KeyCode.W) || Input.GetKeyDown(KeyCode.UpArrow)) Nudge(+stepSmall);
            if (Input.GetKeyDown(KeyCode.S) || Input.GetKeyDown(KeyCode.DownArrow)) Nudge(-stepSmall);
            if (Input.GetKeyDown(KeyCode.E)) Nudge(+stepLarge);
            if (Input.GetKeyDown(KeyCode.D)) Nudge(-stepLarge);
            if (Input.GetKeyDown(KeyCode.R)) ApplyRecordedBestValue();
            if (Input.GetKeyDown(KeyCode.F)) ResetToInitialValue();

            if (Input.GetKeyDown(KeyCode.Z) && lowerButton && lowerButton.interactable) SnapLower();
            if (Input.GetKeyDown(KeyCode.X) && bestButton && bestButton.interactable) SnapBest();
            if (Input.GetKeyDown(KeyCode.C) && upperButton && upperButton.interactable) SnapUpper();
        }
    }

    // ───────────── Python config ─────────────
    public async void ApplyPythonModeSettings()
    {
        if (pythonBridge == null) return;

        string a = await pythonBridge.SetAutoReverseModeAsync(autoReverseOnAnimDone);
        if (a != "K") Debug.LogError("[Trial] SetAutoReverseModeAsync failed: " + a);
    }

    // ───────────── Baseline / Initial ─────────────
    public async void RecordBaselineFromUnity()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[Trial] pythonBridge is null");
            return;
        }

        string result = await pythonBridge.RecordBaselineAsync();
        if (result == "K")
            Debug.Log("[Trial] Baseline recorded successfully.");
        else
            Debug.LogError("[Trial] Record baseline failed: " + result);
    }

    public async void GoInitialFromUnity()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[Trial] pythonBridge is null");
            return;
        }

        string result = await pythonBridge.GoInitialAsync();
        if (result == "K")
        {
            ResetBallToInitialVisual();
            Debug.Log("[Trial] GoInitial success.");
        }
        else
            Debug.LogError("[Trial] GoInitial failed: " + result);
    }

    // ───────────── Play flow ─────────────
    public async void RebootAllMotorsFromUnity()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[Trial] pythonBridge is null");
            return;
        }

        string result = await pythonBridge.RebootAllMotorsAsync();
        if (result == "K")
            Debug.Log("[Trial] Reboot all motors success. Please record baseline again.");
        else
            Debug.LogError("[Trial] Reboot all motors failed: " + result);
    }

    public async void PlayOnce()
    {
        if (isAnimating) return;
        if (_rows.Count == 0)
        {
            Debug.LogError("[Trial] 沒有可用的 trial rows。請檢查 CSV / UserID。");
            return;
        }

        StopAllCoroutines();
        StopAdjustLoop();

        int gi = GlobalTrialIndex;
        var (ab_ad_angle, ex_angle) = GetConvertedAnglesForPython(gi);

        float deltaToUse = _firstPlayThisTrial ? defaultBaseChangePercent : DeltaPercent;
        bool usedBaseChange = _firstPlayThisTrial;

        adjustmentsUnlocked = false;
        SetAllControls(false);
        if (targetPreviewBall) targetPreviewBall.gameObject.SetActive(false);

        if (_firstPlayThisTrial)
            trialStartTime = Time.time;

        ResetBallToInitialVisual();

        Task<string> goInitTask = null;
        if (pythonBridge != null)
            goInitTask = pythonBridge.GoInitialAsync();

        _currentAnimDuration = GetMotorSyncedDuration(Mathf.Abs(ab_ad_angle), Mathf.Abs(ex_angle));

        StartCoroutine(PlaySequenceAfterGoInitial(
            goInitTask,
            deltaToUse,
            usedBaseChange,
            ab_ad_angle,
            ex_angle,
            _currentAnimDuration
        ));
    }

    private void yieldBreakLikeRecovery()
    {
        isAnimating = false;
        adjustmentsUnlocked = false;
        SetAllControls(true);
    }

    private IEnumerator PlayAnim(float animDeltaPercent, bool usedBaseChange, int ab_ad_angle, int ex_angle, float animDuration)
    {
        isAnimating = true;

        ApplyScale(ball, initialSize);
        yield return new WaitForSeconds(Mathf.Max(0f, holdAtStartSeconds));

        Task<string> moveTask = null;

        if (sendAngleOnPlay && pythonBridge != null)
        {
            moveTask = pythonBridge.SendTrialRelativeAsync(ab_ad_angle, ex_angle);
            Debug.Log($"[Trial] Python combined command dispatched: b{Mathf.Abs(ab_ad_angle)} e{Mathf.Abs(ex_angle)}");
        }

        float targetSize = CalcTargetSize(initialSize, animDeltaPercent);
        Vector3 start = Vector3.one * initialSize;
        Vector3 end = Vector3.one * targetSize;

        float t = 0f;
        while (t < 1f)
        {
            t += Time.deltaTime / Mathf.Max(0.0001f, animDuration);
            float k = curve.Evaluate(Mathf.Clamp01(t));
            ball.localScale = Vector3.LerpUnclamped(start, end, k);
            yield return null;
        }
        ball.localScale = end;

        if (moveTask != null)
        {
            while (!moveTask.IsCompleted) yield return null;

            string result = moveTask.Result;
            if (result != "K")
            {
                Debug.LogError("[Trial] Python trial move failed: " + result);
                isAnimating = false;
                SetAllControls(true);
                yield break;
            }
        }

        if (pythonBridge != null && autoReverseOnAnimDone)
        {
            yield return new WaitForSeconds(Mathf.Max(0f, autoReverseHoldSeconds));

            var animDoneTask = pythonBridge.SendAnimDoneAsync();
            while (!animDoneTask.IsCompleted) yield return null;

            string r = animDoneTask.Result;
            if (r != "K")
            {
                Debug.LogError("[Trial] SendAnimDoneAsync failed: " + r);
                isAnimating = false;
                SetAllControls(true);
                yield break;
            }

            ResetBallToInitialVisual();
            Debug.Log("[Trial] Python K -> auto reversed to initial");
        }

        isAnimating = false;
        adjustmentsUnlocked = true;
        _firstPlayThisTrial = false;

        if (usedBaseChange)
            _deltaPercent = Mathf.Clamp(animDeltaPercent, minPercent, maxPercent);

        RefreshStaticPreview();
        StartAdjustLoop();
        UpdateSnapFlowState();
        SetAllControls(true);
    }

    private IEnumerator PlaySequenceAfterGoInitial(
        Task<string> goInitTask,
        float deltaToUse,
        bool usedBaseChange,
        int ab_ad_angle,
        int ex_angle,
        float animDuration)
    {
        if (goInitTask != null)
        {
            while (!goInitTask.IsCompleted)
                yield return null;

            string goInitResult = goInitTask.Result;
            if (goInitResult != "K")
            {
                Debug.LogError("[Trial] GoInitial before Play failed: " + goInitResult);
                SetAllControls(true);
                yieldBreakLikeRecovery();
                yield break;
            }
        }

        yield return StartCoroutine(
            PlayAnim(deltaToUse, usedBaseChange, ab_ad_angle, ex_angle, animDuration)
        );
    }

    private void ResetBallToInitialVisual()
    {
        StopAdjustLoop();
        ApplyScale(ball, initialSize);

        if (targetPreviewBall)
            targetPreviewBall.gameObject.SetActive(false);
    }

    // ───────────── Adjust ─────────────
    public void Nudge(float stepPercent)
    {
        if (!adjustmentsUnlocked) return;
        _deltaPercent = Mathf.Clamp(_deltaPercent + stepPercent, minPercent, maxPercent);
        RefreshStaticPreview();
    }

    private void StartAdjustLoop()
    {
        StopAdjustLoop();
        _adjustCo = StartCoroutine(AdjustLoopCo());
    }

    private void StopAdjustLoop()
    {
        if (_adjustCo != null)
        {
            StopCoroutine(_adjustCo);
            _adjustCo = null;
        }
    }

    private IEnumerator AdjustLoopCo()
    {
        if (!ball) yield break;

        while (adjustmentsUnlocked)
        {
            Vector3 start = Vector3.one * Mathf.Max(minTargetSize, initialSize);

            float loopDuration = useMotorSyncedDuration ? _currentAnimDuration : duration;
            float t = 0f;
            while (t < 1f && adjustmentsUnlocked)
            {
                float targetSize = CalcTargetSize(initialSize, DeltaPercent);
                Vector3 end = Vector3.one * targetSize;

                t += Time.deltaTime / Mathf.Max(0.0001f, loopDuration);
                float k = curve.Evaluate(Mathf.Clamp01(t));
                ball.localScale = Vector3.LerpUnclamped(start, end, k);
                yield return null;
            }

            if (!adjustmentsUnlocked) break;

            yield return new WaitForSeconds(Mathf.Max(0f, adjustLoopEndHold));
            ApplyScale(ball, initialSize);
        }
    }

    private void RefreshStaticPreview()
    {
        if (!targetPreviewBall) return;

        float targetSize = CalcTargetSize(initialSize, DeltaPercent);
        targetSize = Mathf.Max(minTargetSize, targetSize);

        targetPreviewBall.localScale = Vector3.one * targetSize;
        targetPreviewBall.gameObject.SetActive(adjustmentsUnlocked);
    }

    // ───────────── Snap flow ─────────────
    private void SetButtonAlpha(Button btn, float a)
    {
        if (!btn) return;
        var img = btn.GetComponent<Image>();
        if (!img) return;
        var c = img.color;
        c.a = a;
        img.color = c;
    }

    private void ResetSnapFlowForCurrentTrial()
    {
        _snapLower = null;
        _snapBest = null;
        _snapUpper = null;

        InitSnapButton(lowerButton, "Lowerbound", snapIdleColor, snapTextColor);
        InitSnapButton(bestButton, "Best", snapIdleColor, snapTextColor);
        InitSnapButton(upperButton, "Upperbound", snapIdleColor, snapTextColor);

        UpdateSnapFlowState();
        UpdateNextButtonState();
    }

    private void UpdateSnapFlowState()
    {
        if (!adjustmentsUnlocked)
        {
            if (bestButton) bestButton.interactable = false;
            if (lowerButton) lowerButton.interactable = false;
            if (upperButton) upperButton.interactable = false;

            SetButtonAlpha(bestButton, disabledButtonAlpha);
            SetButtonAlpha(lowerButton, disabledButtonAlpha);
            SetButtonAlpha(upperButton, disabledButtonAlpha);
            return;
        }

        if (!_snapBest.HasValue)
        {
            if (bestButton) bestButton.interactable = true;
            if (lowerButton) lowerButton.interactable = false;
            if (upperButton) upperButton.interactable = false;

            SetButtonAlpha(bestButton, 1f);
            SetButtonAlpha(lowerButton, disabledButtonAlpha);
            SetButtonAlpha(upperButton, disabledButtonAlpha);
            return;
        }

        if (bestButton) bestButton.interactable = true;
        SetButtonAlpha(bestButton, 1f);

        if (lowerButton)
        {
            lowerButton.interactable = true;
            SetButtonAlpha(lowerButton, 1f);
        }

        if (upperButton)
        {
            upperButton.interactable = true;
            SetButtonAlpha(upperButton, 1f);
        }
    }

    public void SnapBest()
    {
        if (!adjustmentsUnlocked) return;

        _snapBest = DeltaPercent;
        SetButtonCaptured(bestButton, true);

        PersistSnapshotsForCurrentTrial();
        UpdateSnapFlowState();
        UpdateNextButtonState();
    }

    public void SnapLower()
    {
        if (!adjustmentsUnlocked || !_snapBest.HasValue) return;

        _snapLower = DeltaPercent;
        SetButtonCaptured(lowerButton, true);

        _deltaPercent = _snapBest.Value;
        RefreshStaticPreview();

        PersistSnapshotsForCurrentTrial();
        UpdateSnapFlowState();
        UpdateNextButtonState();
    }

    public void SnapUpper()
    {
        if (!adjustmentsUnlocked || !_snapBest.HasValue) return;

        _snapUpper = DeltaPercent;
        SetButtonCaptured(upperButton, true);

        _deltaPercent = _snapBest.Value;
        RefreshStaticPreview();

        PersistSnapshotsForCurrentTrial();
        UpdateSnapFlowState();
        UpdateNextButtonState();
    }

    private bool TryGetRecordedBestValue(out float bestValue)
    {
        if (_snapBest.HasValue)
        {
            bestValue = Mathf.Clamp(_snapBest.Value, minPercent, maxPercent);
            return true;
        }

        int gi = GlobalTrialIndex;
        float? recordedBest = GetOptionalValueFor(gi, bestColumn);
        if (!recordedBest.HasValue)
        {
            bestValue = 0f;
            return false;
        }

        bestValue = Mathf.Clamp(recordedBest.Value, minPercent, maxPercent);
        return true;
    }

    public void ApplyRecordedBestValue()
    {
        if (!adjustmentsUnlocked || isAnimating) return;
        if (!TryGetRecordedBestValue(out float bestValue)) return;

        _deltaPercent = bestValue;
        RefreshStaticPreview();
        UpdateTrialInfoLabel();
    }

    public void ResetToInitialValue()
    {
        if (!adjustmentsUnlocked || isAnimating) return;

        _deltaPercent = 0f;
        RefreshStaticPreview();
        UpdateTrialInfoLabel();
    }

    private void InitSnapButton(Button btn, string label, Color bg, Color fg)
    {
        if (!btn) return;

        var img = btn.GetComponent<Image>();
        if (img) img.color = bg;

        var txt = btn.GetComponentInChildren<TextMeshProUGUI>(true);
        if (txt)
        {
            txt.text = label;
            txt.color = fg;
            txt.enabled = true;
        }

        var cb = btn.colors;
        cb.normalColor = bg;
        cb.highlightedColor = bg;
        cb.pressedColor = bg * 0.9f;
        cb.selectedColor = bg;
        cb.disabledColor = bg * 0.6f;
        cb.colorMultiplier = 1f;
        btn.colors = cb;
    }

    private void SetButtonCaptured(Button btn, bool captured)
    {
        if (!btn) return;

        var img = btn.GetComponent<Image>();
        if (img) img.color = captured ? snapDoneColor : snapIdleColor;

        var cb = btn.colors;
        cb.normalColor = img ? img.color : (captured ? snapDoneColor : snapIdleColor);
        cb.highlightedColor = cb.normalColor;
        cb.pressedColor = cb.normalColor * 0.9f;
        cb.selectedColor = cb.normalColor;
        cb.disabledColor = cb.normalColor * 0.6f;
        cb.colorMultiplier = 1f;
        btn.colors = cb;

        var txt = btn.GetComponentInChildren<TextMeshProUGUI>(true);
        if (txt) txt.enabled = true;
    }

    private bool HasAllSnapshots() => _snapLower.HasValue && _snapBest.HasValue && _snapUpper.HasValue;

    private void PersistSnapshotsForCurrentTrial()
    {
        WriteBackSnapshots(GlobalTrialIndex);
        SaveCsv();
    }

    private void UpdateNextButtonState()
    {
        if (nextTrialButton) nextTrialButton.interactable = HasAllSnapshots();
        if (nextGuardLabel) nextGuardLabel.text = HasAllSnapshots() ? "" : nextGuardMessage;
    }

    private void WarnNextGuard()
    {
        if (nextGuardLabel) nextGuardLabel.text = nextGuardMessage;
        Debug.LogWarning("[Trial] 未完成 Best / Lower / Upper，無法進入下一個 trial。");
    }

    public async void TryNext()
    {
        if (restPanel && restPanel.activeSelf)
        {
            GoNextBlock();
            return;
        }

        if (!HasAllSnapshots())
        {
            WarnNextGuard();
            return;
        }

        if (pythonBridge != null)
        {
            string goInit = await pythonBridge.GoInitialAsync();
            if (goInit != "K")
            {
                Debug.LogError("[Trial] GoInitial before Next failed: " + goInit);
                return;
            }

            ResetBallToInitialVisual();
        }

        NextTrial();
    }

    public void NextTrial()
    {
        if (!HasAllSnapshots())
        {
            WarnNextGuard();
            return;
        }

        int gi = GlobalTrialIndex;
        WriteBackSnapshots(gi);

        EnsureColumn(durationColumn);
        {
            var row = _rows[gi];
            _colIndex = BuildIndex(_headers);
            row[_colIndex[durationColumn]] = (Time.time - trialStartTime).ToString("0.###");
            _rows[gi] = row;
        }

        SaveCsv();

        int size = CurrentBlockTrialCount;
        if (trialInBlock + 1 < size)
        {
            trialInBlock++;
            AfterMoveToNewTrial();
        }
        else
        {
            if (currentBlock + 1 < BlockCount)
            {
                StopAdjustLoop();
                SetAllControls(false);
                if (nextTrialButton) nextTrialButton.interactable = false;

                if (restPanel) restPanel.SetActive(true);
                if (restMessageLabel) restMessageLabel.text = "Rest for a while, then click Next to continue.";
            }
            else
            {
                StopAdjustLoop();
                SetAllControls(false);
                if (targetPreviewBall) targetPreviewBall.gameObject.SetActive(false);
                if (nextGuardLabel) nextGuardLabel.text = "Experiment completed. Thank you for participating";
                Debug.Log("[Block] 全部完成！");
            }
        }
    }

    private void GoNextBlock()
    {
        if (!(currentBlock + 1 < BlockCount)) return;

        currentBlock++;
        trialInBlock = 0;

        if (restPanel) restPanel.SetActive(false);
        AfterMoveToNewTrial();
    }

    private void AfterMoveToNewTrial()
    {
        _deltaPercent = 0f;
        ApplyScale(ball, initialSize);

        adjustmentsUnlocked = false;
        StopAdjustLoop();
        if (targetPreviewBall) targetPreviewBall.gameObject.SetActive(false);

        _firstPlayThisTrial = true;
        ResetSnapFlowForCurrentTrial();
        LoadSnapshotsForCurrentTrial();
        UpdateTrialInfoLabel();
        SetAllControls(true);
    }

    // ───────────── 計算 / 共同工具 ─────────────
    private float GetMotorSyncedDuration(int bDegAbs, int eDegAbs)
    {
        if (!useMotorSyncedDuration)
            return duration;

        float vb = useSizeVelocityMode ? sizeModeDegPerSecB : applModeDegPerSecB;
        float ve = useSizeVelocityMode ? sizeModeDegPerSecE : applModeDegPerSecE;

        float tb = (bDegAbs <= 0) ? 0f : (bDegAbs / Mathf.Max(0.001f, vb));
        float te = (eDegAbs <= 0) ? 0f : (eDegAbs / Mathf.Max(0.001f, ve));

        float t = Mathf.Max(tb, te);
        if (t <= 0f) t = duration;

        return Mathf.Max(0.05f, t);
    }

    private float CalcTargetSize(float baseSize, float deltaPercent)
    {
        float factor = 1f + deltaPercent / 100f;
        float t = baseSize * factor;
        return Mathf.Max(minTargetSize, t);
    }

    private void ApplyScale(Transform tr, float size)
    {
        if (!tr) return;
        tr.localScale = Vector3.one * Mathf.Max(minTargetSize, size);
    }

    private void SetAllControls(bool on)
    {
        if (playButton) playButton.interactable = on && !isAnimating;
        UpdateSnapFlowState();
        if (applyBestValueButton) applyBestValueButton.interactable = on && adjustmentsUnlocked;
        if (resetValueButton) resetValueButton.interactable = on && adjustmentsUnlocked;
        if (nextTrialButton) nextTrialButton.interactable = on && HasAllSnapshots();
    }

    private void BuildHotkeyHelpText()
    {
        if (!hotkeyHelpLabel) return;
        hotkeyHelpLabel.text =
            "Hotkeys:\n" +
            "B: Record Baseline\n" +
            "I: Go Initial\n" +
            "O: Reboot Motors\n" +
            "W / ↑ : +1%\n" +
            "S / ↓ : -1%\n" +
            "E : +5%   D : -5%\n" +
            "R: apply best   F: reset initial\n" +
            "Z: lower   X: best   C: upper\n" +
            "Space: Play   Enter: Next";
    }

    private void UpdateTrialInfoLabel()
    {
        if (!trialInfoLabel) return;

        if (_rows.Count == 0)
        {
            trialInfoLabel.text = $"User {userId} | No trials found";
            return;
        }

        int gi = GlobalTrialIndex;
        int realBlockNumber = CurrentBlockNumber;
        int blockCount = BlockCount;
        int blockTrialCount = CurrentBlockTrialCount;

        float rawAbAd = GetValueFor(gi, abAdColumn);
        float rawEx = GetValueFor(gi, exColumn);
        var (cmdAbAd, cmdEx) = GetConvertedAnglesForPython(gi);

        string method = GetStringFor(gi, methodColumn);
        string objectType = GetStringFor(gi, objectTypeColumn);
        string trialNo = GetStringFor(gi, trialNoColumn);

        trialInfoLabel.text =
            $"User {userId} | Block {realBlockNumber} ({currentBlock + 1}/{blockCount}) | Trial {trialInBlock + 1}/{blockTrialCount} | CSV Trial_no {trialNo}\n" +
            $"Method = {method} | Object = {objectType}\n" +
            $"raw ab_ad = {rawAbAd:0.###}, raw ex = {rawEx:0.###}\n" +
            $"cmd ab_ad = {cmdAbAd}, cmd ex = {cmdEx}";
    }

    // ───────────── Block mapping ─────────────
    private void BuildBlockMapFromFilteredRows()
    {
        _uniqueBlocks.Clear();
        _blockToUserRowIndices.Clear();

        if (_rows.Count == 0)
        {
            Debug.LogWarning($"[CSV] UserID={userId} 沒有任何 trial。");
            return;
        }

        if (!useBlockColumnFromCsv || !_colIndex.ContainsKey(blockColumn))
        {
            _uniqueBlocks.Add(1);
            var allIndices = new List<int>();
            for (int i = 0; i < _rows.Count; i++) allIndices.Add(i);
            _blockToUserRowIndices[1] = allIndices;
            return;
        }

        var temp = new SortedDictionary<int, List<int>>();
        for (int i = 0; i < _rows.Count; i++)
        {
            int blockValue = Mathf.RoundToInt(GetValueFor(i, blockColumn));
            if (!temp.ContainsKey(blockValue))
                temp[blockValue] = new List<int>();
            temp[blockValue].Add(i);
        }

        foreach (var kv in temp)
        {
            _uniqueBlocks.Add(kv.Key);
            _blockToUserRowIndices[kv.Key] = kv.Value;
        }

        Debug.Log($"[CSV] User {userId} block map = {string.Join(", ", _uniqueBlocks.Select(b => $"{b}({ _blockToUserRowIndices[b].Count })"))}");
    }

    private void ApplyStartTrialNumber()
    {
        if (_rows.Count == 0)
        {
            currentBlock = 0;
            trialInBlock = 0;
            return;
        }

        // 1-based -> 0-based
        int targetGlobalIndex = Mathf.Clamp(startFromTrialNumber - 1, 0, _rows.Count - 1);

        // 先給預設值，避免 block map 異常時壞掉
        currentBlock = 0;
        trialInBlock = 0;

        // 依照 block map 找出這個 global row index 落在哪個 block / 哪個 trial
        for (int b = 0; b < _uniqueBlocks.Count; b++)
        {
            int realBlock = _uniqueBlocks[b];
            if (_blockToUserRowIndices.TryGetValue(realBlock, out var rowList))
            {
                int localIndex = rowList.IndexOf(targetGlobalIndex);
                if (localIndex >= 0)
                {
                    currentBlock = b;
                    trialInBlock = localIndex;

                    Debug.Log($"[StartTrial] startFromTrialNumber={startFromTrialNumber} -> globalRow={targetGlobalIndex}, block={realBlock}, trialInBlock={trialInBlock + 1}");
                    return;
                }
            }
        }

        // 如果沒找到，退回第一筆
        Debug.LogWarning($"[StartTrial] 找不到 startFromTrialNumber={startFromTrialNumber} 對應的 row，退回第一筆。");
        currentBlock = 0;
        trialInBlock = 0;
    }

    private int BlockCount => Mathf.Max(1, _uniqueBlocks.Count);

    private int CurrentBlockNumber
    {
        get
        {
            if (_uniqueBlocks.Count == 0) return 1;
            int idx = Mathf.Clamp(currentBlock, 0, _uniqueBlocks.Count - 1);
            return _uniqueBlocks[idx];
        }
    }

    private int CurrentBlockTrialCount
    {
        get
        {
            int blockNum = CurrentBlockNumber;
            if (_blockToUserRowIndices.TryGetValue(blockNum, out var list))
                return Mathf.Max(1, list.Count);
            return Mathf.Max(1, _rows.Count);
        }
    }

    private int GlobalTrialIndex
    {
        get
        {
            if (_rows.Count == 0) return 0;

            int blockNum = CurrentBlockNumber;
            if (_blockToUserRowIndices.TryGetValue(blockNum, out var list) && list.Count > 0)
            {
                int t = Mathf.Clamp(trialInBlock, 0, list.Count - 1);
                return list[t];
            }

            return Mathf.Clamp(trialInBlock, 0, _rows.Count - 1);
        }
    }

    // ───────────── CSV ─────────────
    private void LoadCsvOrCreate()
    {
        string full = ResolvePathWithUserId(spreadsheetPath);

        try
        {
            var dir = Path.GetDirectoryName(full);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);
        }
        catch { }

        if (!File.Exists(full))
        {
            _headers = new List<string>
            {
                trialNoColumn, blockColumn, methodColumn, objectTypeColumn,
                abAdColumn, exColumn, lowerColumn, bestColumn, upperColumn, durationColumn
            };
            _rows = new List<string[]>();
            _colIndex = BuildIndex(_headers);
            File.WriteAllLines(full, new[] { string.Join(",", _headers) });
            Debug.Log($"[CSV] 未找到，已建立新檔：{full}");
            return;
        }

        var lines = File.ReadAllLines(full);
        if (lines.Length == 0)
        {
            _headers = new List<string>
            {
                trialNoColumn, blockColumn, methodColumn, objectTypeColumn,
                abAdColumn, exColumn, lowerColumn, bestColumn, upperColumn, durationColumn
            };
            _rows = new List<string[]>();
            _colIndex = BuildIndex(_headers);
            File.WriteAllLines(full, new[] { string.Join(",", _headers) });
            Debug.Log($"[CSV] 空檔，已寫入表頭：{full}");
            return;
        }

        _headers = new List<string>(ParseCsvLine(lines[0]));

        EnsureColumn(trialNoColumn);
        EnsureColumn(blockColumn);
        EnsureColumn(methodColumn);
        EnsureColumn(objectTypeColumn);
        EnsureColumn(abAdColumn);
        EnsureColumn(exColumn);
        EnsureColumn(lowerColumn);
        EnsureColumn(bestColumn);
        EnsureColumn(upperColumn);
        EnsureColumn(durationColumn);

        _colIndex = BuildIndex(_headers);

        _rows = new List<string[]>();
        for (int i = 1; i < lines.Length; i++)
        {
            var cells = ParseCsvLine(lines[i]).ToArray();
            ArrayResizeOrPad(ref cells, _headers.Count);
            _rows.Add(cells);
        }

        Debug.Log($"[CSV] 載入 user 檔案：{full} | 欄位={string.Join("|", _headers)} | 列數={_rows.Count}");
    }

    private void SaveCsv()
    {
        string full = ResolvePathWithUserId(spreadsheetPath);
        var outLines = new List<string>();
        outLines.Add(JoinCsvLine(_headers.ToArray()));

        foreach (var r in _rows)
        {
            var row = r;
            if (row.Length != _headers.Count) ArrayResizeOrPad(ref row, _headers.Count);
            outLines.Add(JoinCsvLine(row));
        }

        File.WriteAllLines(full, outLines.ToArray());
        Debug.Log($"[CSV] 已儲存 user 檔案：{full}（{_rows.Count} 列）");
    }

    private void WriteBackSnapshots(int rowIdx)
    {
        if (_rows.Count == 0 || rowIdx < 0 || rowIdx >= _rows.Count) return;

        var row = _rows[rowIdx];
        EnsureColumn(lowerColumn);
        EnsureColumn(bestColumn);
        EnsureColumn(upperColumn);
        _colIndex = BuildIndex(_headers);

        if (_snapLower.HasValue)
            row[_colIndex[lowerColumn]] = _snapLower.Value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        if (_snapBest.HasValue)
            row[_colIndex[bestColumn]] = _snapBest.Value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        if (_snapUpper.HasValue)
            row[_colIndex[upperColumn]] = _snapUpper.Value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);

        _rows[rowIdx] = row;
    }

    private float? GetOptionalValueFor(int rowIdx, string colName)
    {
        if (_rows.Count == 0 || rowIdx < 0 || rowIdx >= _rows.Count) return null;
        if (!_colIndex.TryGetValue(colName, out int c)) return null;

        var s = _rows[rowIdx][c]?.Trim();
        if (string.IsNullOrEmpty(s)) return null;
        if (string.Equals(s, "NaN", StringComparison.OrdinalIgnoreCase)) return null;

        if (float.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float v))
            return float.IsNaN(v) ? (float?)null : v;
        if (float.TryParse(s, out v))
            return float.IsNaN(v) ? (float?)null : v;

        return null;
    }

    private void LoadSnapshotsForCurrentTrial()
    {
        if (_rows.Count == 0) return;

        int gi = GlobalTrialIndex;
        _snapLower = GetOptionalValueFor(gi, lowerColumn);
        _snapBest = GetOptionalValueFor(gi, bestColumn);
        _snapUpper = GetOptionalValueFor(gi, upperColumn);

        SetButtonCaptured(lowerButton, _snapLower.HasValue);
        SetButtonCaptured(bestButton, _snapBest.HasValue);
        SetButtonCaptured(upperButton, _snapUpper.HasValue);
        UpdateSnapFlowState();
        UpdateNextButtonState();
    }

    private (float ab_ad, float ex) GetAnglesFor(int rowIdx)
    {
        float ab_ad = GetValueFor(rowIdx, abAdColumn);
        float ex = GetValueFor(rowIdx, exColumn);
        return (ab_ad, ex);
    }

    private (int ab_ad_cmd, int ex_cmd) GetConvertedAnglesForPython(int rowIdx)
    {
        float rawAbAd = GetValueFor(rowIdx, abAdColumn);
        float rawEx = GetValueFor(rowIdx, exColumn);

        int abAdCmd = Mathf.RoundToInt(rawAbAd * abAdScale);
        int exCmd = Mathf.RoundToInt(rawEx * exScale);

        // int abAdCmd = Mathf.RoundToInt(rawAbAd);
        // int exCmd = Mathf.RoundToInt(rawEx);

        return (abAdCmd, exCmd);
    }

    private float GetValueFor(int rowIdx, string colName)
    {
        if (_rows.Count == 0 || rowIdx < 0 || rowIdx >= _rows.Count) return 0f;
        if (!_colIndex.TryGetValue(colName, out int c)) return 0f;

        var s = _rows[rowIdx][c]?.Trim();
        if (string.IsNullOrEmpty(s)) return 0f;

        if (string.Equals(s, "NaN", StringComparison.OrdinalIgnoreCase))
            return 0f;

        if (float.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float v))
            return v;
        if (float.TryParse(s, out v))
            return v;

        return 0f;
    }

    private string GetStringFor(int rowIdx, string colName)
    {
        if (_rows.Count == 0 || rowIdx < 0 || rowIdx >= _rows.Count) return "";
        if (!_colIndex.TryGetValue(colName, out int c)) return "";
        return (_rows[rowIdx][c] ?? "").Trim();
    }

    private void EnsureColumn(string name)
    {
        int idx = _headers.FindIndex(h => string.Equals(h, name, StringComparison.OrdinalIgnoreCase));
        if (idx >= 0) return;

        _headers.Add(name);

        for (int i = 0; i < _rows.Count; i++)
        {
            var row = _rows[i];
            ArrayResizeOrPad(ref row, _headers.Count);
            _rows[i] = row;
        }
    }

    // ───────────── Low-level CSV utils ─────────────
    private static Dictionary<string, int> BuildIndex(List<string> headers)
    {
        var map = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        for (int i = 0; i < headers.Count; i++) map[headers[i]] = i;
        return map;
    }

    private static void ArrayResizeOrPad(ref string[] arr, int size)
    {
        if (arr.Length == size) return;
        Array.Resize(ref arr, size);
        for (int i = 0; i < arr.Length; i++)
            if (arr[i] == null) arr[i] = "";
    }

    private static List<string> ParseCsvLine(string line)
    {
        var res = new List<string>();
        if (line == null) return res;

        bool inQuotes = false;
        System.Text.StringBuilder sb = new System.Text.StringBuilder();

        for (int i = 0; i < line.Length; i++)
        {
            char ch = line[i];
            if (inQuotes)
            {
                if (ch == '\"')
                {
                    if (i + 1 < line.Length && line[i + 1] == '\"')
                    {
                        sb.Append('\"');
                        i++;
                    }
                    else
                    {
                        inQuotes = false;
                    }
                }
                else
                {
                    sb.Append(ch);
                }
            }
            else
            {
                if (ch == ',')
                {
                    res.Add(sb.ToString());
                    sb.Length = 0;
                }
                else if (ch == '\"')
                {
                    inQuotes = true;
                }
                else
                {
                    sb.Append(ch);
                }
            }
        }

        res.Add(sb.ToString());
        return res;
    }

    private static string JoinCsvLine(string[] cells)
    {
        var list = new List<string>(cells.Length);
        foreach (var s in cells)
        {
            var t = s ?? "";
            bool needQuote = t.Contains(",") || t.Contains("\"") || t.Contains("\n") || t.Contains("\r");
            if (needQuote) t = "\"" + t.Replace("\"", "\"\"") + "\"";
            list.Add(t);
        }
        return string.Join(",", list);
    }

    private static string ResolvePath(string p)
    {
        if (Path.IsPathRooted(p)) return p;
        string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        return Path.GetFullPath(Path.Combine(projectRoot, p));
    }

    private string ResolvePathWithUserId(string p)
    {
        string withId = (p ?? "").Replace("{UserID}", userId.ToString());
        return ResolvePath(withId);
    }
}
