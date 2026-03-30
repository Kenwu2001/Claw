using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine.EventSystems;

public class BallFlyController : MonoBehaviour
{
    [Header("Scene References")]
    public Transform ball;
    public Transform startPoint;
    public Transform endPoint;

    private Rigidbody ballRb;
    private bool isStuckToHand = false;

    [Header("Hand Collision")]
    public Transform handRoot;

    [Header("Dynamic Target")]
    [Tooltip("球在飛行時持續追蹤目標，確保最後到達目標位置")]
    public bool trackMovingTarget = true;

    [Tooltip("要追蹤的目標。若空白，會嘗試用 handRoot 或名稱尋找")]
    public Transform movingTarget;

    [Tooltip("當 movingTarget 沒設時，會用名稱自動尋找")]
    public string movingTargetName = "EndPoint";

    [Tooltip("球距離目標小於此值時，直接吸附到目標")]
    public float targetReachDistance = 0.015f;

    [Tooltip("追蹤移動目標時的速度倍率")]
    public float movingTargetSpeedMultiplier = 1.6f;

    [Tooltip("追蹤超過此秒數仍未命中，會強制吸附到目標")]
    public float maxMovingTargetChaseSeconds = 5.0f;

    [Header("Python Bridge")]
    public PythonMotorBridge pythonBridge;

    [Tooltip("開始時主動送 VAPPL")]
    public bool forceApplVelocityModeOnStart = true;

    [Tooltip("撞擊時是否送馬達命令")]
    public bool sendAngleOnImpact = true;

    [Header("Impact Timing")]
    public float handPoseDelay = 0.05f;  // 50 ms，自己調

    [Tooltip("撞擊後幾秒再送馬達命令；0 = 立刻")]
    public float motorTriggerDelay = 0f;

    [Header("Pretrigger")]
    public bool usePreTrigger = true;
    public float preTriggerDistance = 0.04f; // 4 cm，可再調

    [Tooltip("若要縮小/放大 trial angle，可用這個倍率。注意：1.5689/9.1618 校正已放在 Python，不在這裡。")]
    [Range(0f, 2f)]
    public float impactAngleScale = 1f;

    [Tooltip("按 T 測試馬達")]
    public bool enableManualMotorTestKey = true;

    [Tooltip("不看 CSV，直接用固定角度測試")]
    public bool useHardcodedImpactAngles = false;
    public int hardcodedBDelta = 20;
    public int hardcodedEDelta = 20;

    [Tooltip("Play 前先 GoInitial（關掉，僅第一次手動 I）")]
    public bool goInitialBeforePlay = false;

    [Tooltip("撞擊後停在 50% 開啟量")]
    [Range(0f, 1f)]
    public float returnRatio = 0.5f;
    [Tooltip("回到 0.5 開啟量後，延遲幾秒再回初始位置")]
    public float goInitialDelayAfterHalfReturn = 1f;

    [Header("Speed (units/second)")]
    public float fallbackInitialSpeed = 3f;
    public float minTargetSpeed = 0.2f;
    public float maxTargetSpeed = 20f;
    public float stepSmall = 0.5f;
    public float stepLarge = 2.5f;
    

    [Header("Pauses (seconds)")]
    public float holdAtStartSeconds = 1f;
    public float holdAtEndSeconds = 1f;
    public float holdAtCollision = 1f;

    [Header("Post Play")]
    [Tooltip("命中目標後保留球在目標上，不自動回起點")]
    public bool keepBallOnTargetAfterImpact = true;

    [Tooltip("每次 Play 完成後是否自動進入預覽循環")]
    public bool autoRestartPreviewAfterPlay = false;

    [Header("Hand Pose Blend")]
    public HandPoseBlender handPoseBlender;
    public bool useHandPoseBlend = true;

    [Header("UI")]
    public Button playButton;
    public Button lowerButton;
    public Button bestButton;
    public Button upperButton;
    public Button applyBestSpeedButton;
    public Button resetSpeedButton;
    public Button nextTrialButton;

    public TextMeshProUGUI nextGuardLabel;
    public string nextGuardMessage = "請先設定 lower、best、upper 後再繼續。";
    public TextMeshProUGUI trialInfoLabel;

    [Header("Hotkeys")]
    public bool enableHotkeys = true;
    public TextMeshProUGUI hotkeyHelpLabel;

    [Header("Snap Button Visual")]
    public Color snapIdleColor = new Color32(84, 110, 122, 255);
    public Color snapDoneColor = new Color32(46, 125, 50, 255);
    public Color snapTextColor = Color.white;

    [Header("Button Flash")]
    [Tooltip("按下按鈕時閃一下的顏色")]
    public Color buttonFlashColor = new Color32(255, 213, 79, 255);

    [Tooltip("按鈕閃色持續時間")]
    public float buttonFlashDuration = 0.12f;

    [Header("Spreadsheet (CSV)")]
    public string spreadsheetPath = "Assets/Data/force_trials/user_{UserID}_force_trials.csv";

    [Header("Participant")]
    public int userId = 1;

    [Header("Start Trial")]
    [Tooltip("從第幾個 global trial 開始（1-based）。例如填 1 = 從第一筆開始，填 13 = 從第二個 block 的第一筆開始。")]
    public int startFromTrialNumber = 1;

    [Tooltip("CSV 欄位：trial 原始角度（還沒乘 Python 校正值）")]
    public string abAdColumn = "ab_ad_angle";
    public string exColumn = "ex_angle";

    public string lowerColumn = "lowerbound";
    public string bestColumn = "best";
    public string upperColumn = "upperbound";
    public string durationColumn = "duration";

    [Header("Extra CSV Info Columns (for display only)")]
    public string trialNoColumn = "Trial_no";
    public string methodColumn = "Method";
    public string objectTypeColumn = "ObjectType";
    public string blockColumn = "Block";

    [Header("Blocks")]
    public int[] blockSizes = new int[] { 12, 12, 12 };
    public int currentBlock = 0;
    public int trialInBlock = 0;
    private int[] _blockOffsets;

    [Tooltip("休息面板")]
    public GameObject restPanel;
    public TextMeshProUGUI restMessageLabel;
    public Button restNextButton;

    private bool isAnimating = false;
    private bool adjustmentsUnlocked = false;
    private bool _firstPlayThisTrial = true;
    private bool _impactAlreadyTriggered = false;
    private bool _isPreparingPlay = false;

    private float _baseSpeedThisTrial = 0f;
    private float _deltaSpeed = 0f;
    private float DeltaSpeed => _deltaSpeed;

    private List<string> _headers = new List<string>();
    private List<string[]> _rows = new List<string[]>();
    private Dictionary<string, int> _colIndex = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);

    private float? _snapLower = null;
    private float? _snapBest = null;
    private float? _snapUpper = null;

    private Coroutine _previewLoopCo;
    private float trialStartTime;

    private int _preparedImpactB = 0;
    private int _preparedImpactE = 0;

    private readonly Dictionary<Button, Coroutine> _buttonFlashCos = new Dictionary<Button, Coroutine>();

    private bool EnsureCoreReferences()
    {
        ResolveMovingTargetIfNeeded();

        if (handPoseBlender == null)
            handPoseBlender = GetComponent<HandPoseBlender>();

        if (!ball || !startPoint || !endPoint)
        {
            Debug.LogError("[BallFly] 請指定 ball/startPoint/endPoint");
            enabled = false;
            return false;
        }

        if (ballRb == null)
            ballRb = ball.GetComponent<Rigidbody>();

        if (!ballRb)
        {
            Debug.LogError("[BallFly] ball 物件需要 Rigidbody");
            enabled = false;
            return false;
        }

        return true;
    }

    private void Awake()
    {
        EnsureCoreReferences();
    }

    private void OnEnable()
    {
        RegisterUiListeners();
    }

    private void OnDisable()
    {
        UnregisterUiListeners();
    }

    private void OnDestroy()
    {
        UnregisterUiListeners();
    }

    private void RegisterUiListeners()
    {
        if (playButton)
        {
            playButton.onClick.RemoveListener(PlayOnce);
            playButton.onClick.AddListener(PlayOnce);
        }

        if (lowerButton)
        {
            lowerButton.onClick.RemoveListener(SnapLower);
            lowerButton.onClick.AddListener(SnapLower);
        }

        if (bestButton)
        {
            bestButton.onClick.RemoveListener(SnapBest);
            bestButton.onClick.AddListener(SnapBest);
        }

        if (upperButton)
        {
            upperButton.onClick.RemoveListener(SnapUpper);
            upperButton.onClick.AddListener(SnapUpper);
        }

        if (applyBestSpeedButton)
        {
            applyBestSpeedButton.onClick.RemoveListener(ApplyRecordedBestSpeed);
            applyBestSpeedButton.onClick.AddListener(ApplyRecordedBestSpeed);
        }

        if (resetSpeedButton)
        {
            resetSpeedButton.onClick.RemoveListener(ResetToInitialSpeed);
            resetSpeedButton.onClick.AddListener(ResetToInitialSpeed);
        }

        if (nextTrialButton)
        {
            nextTrialButton.onClick.RemoveListener(TryNext);
            nextTrialButton.onClick.AddListener(TryNext);
        }

        if (restNextButton)
        {
            restNextButton.onClick.RemoveListener(GoNextBlock);
            restNextButton.onClick.AddListener(GoNextBlock);
        }
    }

    private void UnregisterUiListeners()
    {
        if (playButton) playButton.onClick.RemoveListener(PlayOnce);
        if (lowerButton) lowerButton.onClick.RemoveListener(SnapLower);
        if (bestButton) bestButton.onClick.RemoveListener(SnapBest);
        if (upperButton) upperButton.onClick.RemoveListener(SnapUpper);
        if (applyBestSpeedButton) applyBestSpeedButton.onClick.RemoveListener(ApplyRecordedBestSpeed);
        if (resetSpeedButton) resetSpeedButton.onClick.RemoveListener(ResetToInitialSpeed);
        if (nextTrialButton) nextTrialButton.onClick.RemoveListener(TryNext);
        if (restNextButton) restNextButton.onClick.RemoveListener(GoNextBlock);
    }

    void Start()
    {
        if (!EnsureCoreReferences())
            return;

        ResetBallToStart();

        InitSnapButton(lowerButton, "Lowerbound", snapIdleColor, snapTextColor);
        InitSnapButton(bestButton, "Best", snapIdleColor, snapTextColor);
        InitSnapButton(upperButton, "Upperbound", snapIdleColor, snapTextColor);

        LoadCsvOrCreate();
        BuildBlockOffsets();
        // currentBlock = Mathf.Clamp(currentBlock, 0, Mathf.Max(0, blockSizes.Length - 1));
        // trialInBlock = 0;
        ApplyStartTrialNumber();

        LoadBaseSpeedForCurrentTrial();
        LoadSnapshotsForCurrentTrial();

        _firstPlayThisTrial = true;
        UpdateTrialInfoLabel();
        UpdateNextButtonState();
        BuildHotkeyHelpText();

        adjustmentsUnlocked = false;
        if (restPanel) restPanel.SetActive(false);
        SetAllControls(true);

        _ = ConfigurePythonOnStartAsync();
    }

    private void ResolveMovingTargetIfNeeded()
    {
        if (movingTarget != null) return;

        if (!string.IsNullOrWhiteSpace(movingTargetName))
        {
            var go = GameObject.Find(movingTargetName);
            if (go != null)
            {
                movingTarget = go.transform;
                if (handRoot == null)
                    handRoot = go.transform;
                return;
            }
        }

        if (endPoint != null)
        {
            movingTarget = endPoint;
            return;
        }

        if (handRoot != null)
            movingTarget = handRoot;
    }

    private bool TryGetActiveTarget(out Transform target)
    {
        ResolveMovingTargetIfNeeded();
        target = movingTarget != null ? movingTarget : (handRoot != null ? handRoot : endPoint);
        return target != null;
    }

    private float GetChaseTimeoutSeconds(float speed, Transform target)
    {
        float configuredTimeout = Mathf.Max(0.1f, maxMovingTargetChaseSeconds);
        if (target == null || ball == null)
            return configuredTimeout;

        float effectiveSpeed = Mathf.Max(0.01f, speed * Mathf.Max(1f, movingTargetSpeedMultiplier));
        float distance = Vector3.Distance(ball.position, target.position);
        float distanceBasedTimeout = (distance / effectiveSpeed) + 0.5f;

        return Mathf.Max(configuredTimeout, distanceBasedTimeout);
    }

    private void AttachBallToTarget(Transform target)
    {
        if (target == null) return;

        isStuckToHand = true;
        ballRb.linearVelocity = Vector3.zero;
        ballRb.angularVelocity = Vector3.zero;
        ballRb.isKinematic = true;
        ballRb.detectCollisions = false;
        ballRb.MovePosition(target.position);
        ball.SetParent(target);
    }

    private async Task ConfigurePythonOnStartAsync()
    {
        if (pythonBridge == null) return;

        try
        {
            if (forceApplVelocityModeOnStart)
            {
                string r = await pythonBridge.SetVelocityModeAsync(false); // false -> VAPPL
                Debug.Log("[BallFly] Force VAPPL on start result = " + r);
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("[BallFly] ConfigurePythonOnStartAsync exception: " + ex);
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

        if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter)|| Input.GetKeyDown(KeyCode.V))
            TryNext();

        if (Input.GetKeyDown(KeyCode.Space) && playButton && playButton.interactable)
            PlayOnce();

        if (enableManualMotorTestKey && Input.GetKeyDown(KeyCode.T))
        {
            Debug.Log("[BallFly] Manual motor test by key T");
            if (pythonBridge == null)
                Debug.LogError("[BallFly] pythonBridge is null");
            else
                StartCoroutine(ImpactMotorCo());
        }

        if (adjustmentsUnlocked && !(restPanel && restPanel.activeSelf))
        {
            if (Input.GetKeyDown(KeyCode.W) || Input.GetKeyDown(KeyCode.UpArrow)) Nudge(+stepSmall);
            if (Input.GetKeyDown(KeyCode.S) || Input.GetKeyDown(KeyCode.DownArrow)) Nudge(-stepSmall);
            if (Input.GetKeyDown(KeyCode.E)) Nudge(+stepLarge);
            if (Input.GetKeyDown(KeyCode.D)) Nudge(-stepLarge);
            if (Input.GetKeyDown(KeyCode.R)) ApplyRecordedBestSpeed();
            if (Input.GetKeyDown(KeyCode.F)) ResetToInitialSpeed();

            if (Input.GetKeyDown(KeyCode.Z) && lowerButton && lowerButton.interactable) SnapLower();
            if (Input.GetKeyDown(KeyCode.X) && bestButton && bestButton.interactable) SnapBest();
            if (Input.GetKeyDown(KeyCode.C) && upperButton && upperButton.interactable) SnapUpper();
        }

        if (Input.GetKeyDown(KeyCode.Y))
        {
            Debug.Log("[BallFly] Manual hand pose test by key Y");
            if (handPoseBlender != null)
                handPoseBlender.PlayImpactPose();
        }
    }

    public async void RecordBaselineFromUnity()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[BallFly] pythonBridge is null");
            return;
        }

        string result = await pythonBridge.RecordBaselineAsync();
        if (result == "K") Debug.Log("[BallFly] Baseline recorded successfully.");
        else Debug.LogError("[BallFly] Record baseline failed: " + result);
    }

    public async void GoInitialFromUnity()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[BallFly] pythonBridge is null");
            return;
        }

        string result = await pythonBridge.GoInitialAsync();
        if (result == "K") Debug.Log("[BallFly] GoInitial success.");
        else Debug.LogError("[BallFly] GoInitial failed: " + result);
    }

    public async void RebootAllMotorsFromUnity()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[BallFly] pythonBridge is null");
            return;
        }

        string result = await pythonBridge.RebootAllMotorsAsync();
        if (result == "K") Debug.Log("[BallFly] Reboot all motors success. Please record baseline again.");
        else Debug.LogError("[BallFly] Reboot all motors failed: " + result);
    }

    private void PrepareImpactAnglesForCurrentTrial()
    {
        if (useHardcodedImpactAngles)
        {
            _preparedImpactB = Mathf.RoundToInt(hardcodedBDelta * impactAngleScale);
            _preparedImpactE = Mathf.RoundToInt(hardcodedEDelta * impactAngleScale);

            Debug.Log($"[BallFly] Prepared HARD impact angles | B={_preparedImpactB}, E={_preparedImpactE}");
            return;
        }

        int gi = GlobalTrialIndex;
        float rawB = GetValueFor(gi, abAdColumn);
        float rawE = GetValueFor(gi, exColumn);

        _preparedImpactB = Mathf.RoundToInt(rawB * impactAngleScale);
        _preparedImpactE = Mathf.RoundToInt(rawE * impactAngleScale);

        Debug.Log($"[BallFly] Prepared CSV impact angles | row={gi} | rawB={rawB}, rawE={rawE} | sendB={_preparedImpactB}, sendE={_preparedImpactE}");
    }

    private void PlayHandPose()
    {
        if(useHandPoseBlend && handPoseBlender != null)
        {
            handPoseBlender.PlayImpactPose();
            Debug.Log("[BallFly] Hand pose triggered");
        }
    }

    private IEnumerator DelayedHandPoseCo()
    {
        if (handPoseDelay > 0f)
            yield return new WaitForSeconds(handPoseDelay);

        PlayHandPose();
    }

    private void TriggerImpactResponse(Transform handTransform)
    {
        if (_impactAlreadyTriggered) return;
        _impactAlreadyTriggered = true;

        Debug.Log("[BallFly] TriggerImpactResponse() called");
        if (useHandPoseBlend && handPoseBlender != null)
            StartCoroutine(DelayedHandPoseCo());

        // 先送馬達，再做手部動畫，降低主觀延遲感
        if (sendAngleOnImpact && isAnimating)
        {
            if (pythonBridge == null)
            {
                Debug.LogError("[BallFly] pythonBridge is null");
            }
            else
            {
                StartCoroutine(ImpactMotorCo());
            }
        }

        // Invoke(nameof(PlayHandPose), 50);

        // if (useHandPoseBlend && handPoseBlender != null)
        // {
        //     handPoseBlender.PlayImpactPose();
        //     Debug.Log("[BallFly] Hand pose triggered");
        // }
    }

    private IEnumerator ImpactMotorCo()
    {
        if (pythonBridge == null)
        {
            Debug.LogError("[BallFly] pythonBridge is null");
            yield break;
        }

        if (motorTriggerDelay > 0f)
            yield return new WaitForSeconds(motorTriggerDelay);

        int fullB = _preparedImpactB;
        int fullE = _preparedImpactE;

        Debug.Log($"[BallFly] Use prepared impact angles | B={fullB}, E={fullE}");

        if (fullB == 0 && fullE == 0)
        {
            Debug.LogWarning("[BallFly] Both full angles are 0, skip motor.");
            yield break;
        }

        // Step 1: 張開到 CSV 指定角度（100%）
        Task<string> fullTask = null;
        try
        {
            fullTask = pythonBridge.SendTrialRelativeAsync(fullB, fullE);
            Debug.Log($"[BallFly] Sent FULL move | B={fullB}, E={fullE}");
        }
        catch (Exception ex)
        {
            Debug.LogError("[BallFly] Send FULL move exception: " + ex.Message);
            yield break;
        }

        while (!fullTask.IsCompleted)
            yield return null;

        if (fullTask.IsFaulted)
        {
            Debug.LogError("[BallFly] FULL task faulted: " + fullTask.Exception);
            yield break;
        }

        string fullResult = fullTask.Result;
        Debug.Log("[BallFly] FULL result = " + fullResult);

        if (string.IsNullOrEmpty(fullResult) || !fullResult.StartsWith("K"))
        {
            Debug.LogError("[BallFly] FULL move failed: " + fullResult);
            yield break;
        }

        // Step 2: 回到張開角度的 0.5
        // if (forceApplVelocityModeOnStart)
        //     {
        //         string r = await pythonBridge.SetVelocityModeAsync(true); // false -> VAPPL
        //         Debug.Log("[BallFly] Force VAPPL on start result = " + r);
        //     }

        // int targetB = Mathf.RoundToInt(fullB * returnRatio);
        // int targetE = Mathf.RoundToInt(fullE * returnRatio);

        // int backB = targetB - fullB;
        // int backE = targetE - fullE;

        // Debug.Log($"[BallFly] BACK calc | fullB={fullB}, fullE={fullE}, targetB={targetB}, targetE={targetE}, backB={backB}, backE={backE}, ratio={returnRatio}");

        
        // if (backB != 0 || backE != 0)
        // {
        //     Task<string> backTask = null;
        //     try
        //     {
        //         backTask = pythonBridge.SendTrialRelativeAsync(backB, backE);
        //         Debug.Log($"[BallFly] Sent BACK move | B={backB}, E={backE} | final ratio={returnRatio}");
        //     }
        //     catch (Exception ex)
        //     {
        //         Debug.LogError("[BallFly] Send BACK move exception: " + ex.Message);
        //         yield break;
        //     }

        //     while (!backTask.IsCompleted)
        //         yield return null;

        //     if (backTask.IsFaulted)
        //     {
        //         Debug.LogError("[BallFly] BACK task faulted: " + backTask.Exception);
        //         yield break;
        //     }

        //     string backResult = backTask.Result;
        //     Debug.Log("[BallFly] BACK result = " + backResult);

        //     if (string.IsNullOrEmpty(backResult) || !backResult.StartsWith("K"))
        //     {
        //         Debug.LogError("[BallFly] BACK move failed: " + backResult);
        //         yield break;
        //     }
        // }
        // else
        // {
        //     Debug.Log("[BallFly] back move = 0, keep full pose.");
        // }

        Task<string> finTask = null;
        try
        {
            finTask = pythonBridge.GoFinishAsync();
            Debug.Log("[BallFly] Sent GoFinish");
        }
        catch (Exception ex)
        {
            Debug.LogError("[BallFly] GoFinish exception: " + ex.Message);
            yield break;
        }

        while (!finTask.IsCompleted)
            yield return null;

        if (finTask.IsFaulted)
        {
            Debug.LogError("[BallFly] GoFinish task faulted: " + finTask.Exception);
            yield break;
        }

        string finResult = finTask.Result;
        Debug.Log("[BallFly] GoFinish result = " + finResult);

        if (string.IsNullOrEmpty(finResult) || !finResult.StartsWith("K"))
        {
            Debug.LogError("[BallFly] GoFinish failed: " + finResult);
            yield break;
        }

        // Step 3: 停 1 秒（或 Inspector 指定秒數）
        Debug.Log($"[BallFly] Hold at returnRatio pose for {goInitialDelayAfterHalfReturn:0.###} sec");
        if (goInitialDelayAfterHalfReturn > 0f)
            yield return new WaitForSeconds(goInitialDelayAfterHalfReturn);

        // Step 4: 回初始位置
        Task<string> initTask = null;
        try
        {
            initTask = pythonBridge.GoInitialAsync();
            Debug.Log("[BallFly] Sent GoInitial after delay");
        }
        catch (Exception ex)
        {
            Debug.LogError("[BallFly] GoInitial exception: " + ex.Message);
            yield break;
        }

        while (!initTask.IsCompleted)
            yield return null;

        if (initTask.IsFaulted)
        {
            Debug.LogError("[BallFly] GoInitial task faulted: " + initTask.Exception);
            yield break;
        }

        string initResult = initTask.Result;
        Debug.Log("[BallFly] GoInitial result = " + initResult);

        if (string.IsNullOrEmpty(initResult) || !initResult.StartsWith("K"))
        {
            Debug.LogError("[BallFly] GoInitial failed: " + initResult);
            yield break;
        }

        Debug.Log("[BallFly] Impact motor sequence finished.");
    }

    public async void PlayOnce()
    {
        if (isAnimating || _isPreparingPlay) return;
        
        FlashButton(playButton);
        StopPreviewLoop();
        adjustmentsUnlocked = false;
        SetAllControls(false);
        _isPreparingPlay = true;

        try
        {
            if (pythonBridge != null && forceApplVelocityModeOnStart)
            {
                string vapplResult = await pythonBridge.SetVelocityModeAsync(false);
                Debug.Log("[BallFly] Ensure VAPPL before play = " + vapplResult);
                if (vapplResult != "K")
                {
                    Debug.LogError("[BallFly] VAPPL failed before play: " + vapplResult);
                    SetAllControls(true);
                    _isPreparingPlay = false;
                    return;
                }
            }

            float speedToUse = _firstPlayThisTrial ? BaseSpeed() : CurrentTargetSpeed();

            if (_firstPlayThisTrial)
                trialStartTime = Time.time;

            PrepareImpactAnglesForCurrentTrial();
            StartCoroutine(PlayOnceRoutine(speedToUse));
        }
        catch (Exception ex)
        {
            Debug.LogError("[BallFly] PlayOnce exception: " + ex);
            SetAllControls(true);
        }
        finally
        {
            _isPreparingPlay = false;
        }
    }

    private IEnumerator PlayOnceRoutine(float oneShotSpeed)
    {
        isAnimating = true;
        _impactAlreadyTriggered = false;

        ResetBallToStart();

        if (holdAtStartSeconds > 0f)
            yield return new WaitForSeconds(holdAtStartSeconds);

        Vector3 A = startPoint.position;
        Vector3 B = endPoint.position;
        float dist = Mathf.Max(0.0001f, Vector3.Distance(A, B));
        float t = 0f;
        float reachSqr = Mathf.Max(0.0001f, targetReachDistance) * Mathf.Max(0.0001f, targetReachDistance);
        float chaseElapsed = 0f;
        float chaseTimeoutSeconds = Mathf.Max(0.1f, maxMovingTargetChaseSeconds);

        if (trackMovingTarget && TryGetActiveTarget(out Transform initialTarget))
            chaseTimeoutSeconds = GetChaseTimeoutSeconds(oneShotSpeed, initialTarget);

        while (t < 1f && !isStuckToHand)
        {
            float speed = Mathf.Clamp(oneShotSpeed, minTargetSpeed, maxTargetSpeed);

            if (trackMovingTarget && TryGetActiveTarget(out Transform target))
            {
                chaseElapsed += Time.deltaTime;
                Vector3 toTarget = target.position - ball.position;
                float step = speed * Mathf.Max(1f, movingTargetSpeedMultiplier) * Time.deltaTime;

                if (toTarget.sqrMagnitude <= reachSqr || toTarget.magnitude <= step)
                {
                    AttachBallToTarget(target);
                    TriggerImpactResponse(target);
                    break;
                }

                if (chaseElapsed >= chaseTimeoutSeconds)
                {
                    Debug.LogWarning($"[BallFly] Moving target chase timeout after {chaseElapsed:0.###} sec, force attach to target.");
                    AttachBallToTarget(target);
                    TriggerImpactResponse(target);
                    break;
                }

                ballRb.MovePosition(ball.position + toTarget.normalized * step);
            }
            else
            {
                t += speed / dist * Time.deltaTime;
                ballRb.MovePosition(Vector3.LerpUnclamped(A, B, t));
            }

            if (usePreTrigger && !_impactAlreadyTriggered && handRoot != null)
            {
                float d = Vector3.Distance(ball.position, handRoot.position);
                if (d <= preTriggerDistance)
                {
                    Debug.Log($"[BallFly] PreTrigger at distance={d:0.000}");
                    TriggerImpactResponse(handRoot);
                }
            }

            yield return null;
        }

        if (!_impactAlreadyTriggered && handPoseBlender != null)
            handPoseBlender.PlayImpactPose();

        if (!isStuckToHand)
            ballRb.MovePosition(B);

        if (!isStuckToHand || !keepBallOnTargetAfterImpact)
        {
            if (holdAtEndSeconds > 0f)
                yield return new WaitForSeconds(holdAtEndSeconds);

            ResetBallToStart();
        }

        isAnimating = false;
        adjustmentsUnlocked = true;
        _firstPlayThisTrial = false;
        SetAllControls(true);

        if (autoRestartPreviewAfterPlay)
            StartPreviewLoop();
    }

    private IEnumerator PreviewLoopCo()
    {
        while (true)
        {
            ResetBallToStart();
            if (holdAtStartSeconds > 0f) yield return new WaitForSeconds(holdAtStartSeconds);

            Vector3 A = startPoint.position;
            Vector3 B = endPoint.position;
            float dist = Mathf.Max(0.0001f, Vector3.Distance(A, B));
            float t = 0f;

            while (t < 1f && !isStuckToHand)
            {
                float speedNow = CurrentTargetSpeed();
                t += Mathf.Clamp(speedNow, minTargetSpeed, maxTargetSpeed) / dist * Time.deltaTime;
                ballRb.MovePosition(Vector3.LerpUnclamped(A, B, t));
                yield return null;
            }

            if (!_impactAlreadyTriggered && handPoseBlender != null)
                handPoseBlender.PlayImpactPose();

            if (!isStuckToHand)
                ballRb.MovePosition(B);

            if (holdAtEndSeconds > 0f)
                yield return new WaitForSeconds(holdAtEndSeconds);

            ResetBallToStart();
            yield return null;
        }
    }

    private void StartPreviewLoop()
    {
        StopPreviewLoop();
        _previewLoopCo = StartCoroutine(PreviewLoopCo());
    }

    private void StopPreviewLoop()
    {
        if (_previewLoopCo != null)
        {
            StopCoroutine(_previewLoopCo);
            _previewLoopCo = null;
        }
    }

    private void ApplyStartTrialNumber()
    {
        if (blockSizes == null || blockSizes.Length == 0)
        {
            currentBlock = 0;
            trialInBlock = 0;
            return;
        }

        int totalTrials = 0;
        for (int i = 0; i < blockSizes.Length; i++)
            totalTrials += Mathf.Max(0, blockSizes[i]);

        int startIndex0 = Mathf.Clamp(startFromTrialNumber - 1, 0, Mathf.Max(0, totalTrials - 1));

        int remain = startIndex0;
        int foundBlock = 0;
        int foundTrialInBlock = 0;

        for (int b = 0; b < blockSizes.Length; b++)
        {
            int size = Mathf.Max(0, blockSizes[b]);
            if (remain < size)
            {
                foundBlock = b;
                foundTrialInBlock = remain;
                break;
            }
            remain -= size;
        }

        currentBlock = foundBlock;
        trialInBlock = foundTrialInBlock;

        Debug.Log($"[BallFly] Start from global trial #{startFromTrialNumber} -> block={currentBlock + 1}, trialInBlock={trialInBlock + 1}");
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (!isAnimating&& _previewLoopCo == null) return;
        if (isStuckToHand) return;

        bool hitHand = false;

        if (handRoot != null)
            hitHand = (collision.transform == handRoot) || collision.transform.IsChildOf(handRoot);
        else
            hitHand = collision.gameObject.CompareTag("Hand");

        if (!hitHand) return;

        AttachBallToTarget(handRoot != null ? handRoot : collision.transform);

        TriggerImpactResponse(collision.transform);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (!isAnimating && _previewLoopCo == null) return;
        if (isStuckToHand) return;

        bool hitHand = false;

        if (handRoot != null)
            hitHand = (other.transform == handRoot) || other.transform.IsChildOf(handRoot);
        else
            hitHand = other.gameObject.CompareTag("Hand");

        if (!hitHand) return;

        AttachBallToTarget(handRoot != null ? handRoot : other.transform);

        TriggerImpactResponse(other.transform);
    }

    private void ResetBallToStart()
    {
        if (!EnsureCoreReferences()) return;

        isStuckToHand = false;
        _impactAlreadyTriggered = false;

        ball.SetParent(null);
        ballRb.isKinematic = false;
        ballRb.detectCollisions = true;
        ballRb.position = startPoint.position;
        ballRb.linearVelocity = Vector3.zero;
        ballRb.angularVelocity = Vector3.zero;
    }

    public void RestartFromLevelStart()
    {
        if (!EnsureCoreReferences()) return;

        StopPreviewLoop();

        isAnimating = false;
        _isPreparingPlay = false;
        adjustmentsUnlocked = false;

        ResetBallToStart();
        SetAllControls(true);
    }

    public void Nudge(float step)
    {
        if (!adjustmentsUnlocked || isAnimating) return;
        float target = Mathf.Clamp(BaseSpeed() + _deltaSpeed + step, minTargetSpeed, maxTargetSpeed);
        _deltaSpeed = target - BaseSpeed();
        UpdateTrialInfoLabel();
    }

    private float BaseSpeed()
    {
        return Mathf.Clamp(_baseSpeedThisTrial > 0f ? _baseSpeedThisTrial : fallbackInitialSpeed,
                           minTargetSpeed, maxTargetSpeed);
    }

    private float CurrentTargetSpeed()
    {
        return Mathf.Clamp(BaseSpeed() + DeltaSpeed, minTargetSpeed, maxTargetSpeed);
    }

    private void SetCurrentTargetSpeed(float targetSpeed)
    {
        float clamped = Mathf.Clamp(targetSpeed, minTargetSpeed, maxTargetSpeed);
        _deltaSpeed = clamped - BaseSpeed();
        UpdateTrialInfoLabel();
    }

    private bool TryGetRecordedBestSpeed(out float bestSpeed)
    {
        if (_snapBest.HasValue)
        {
            bestSpeed = _snapBest.Value;
            return true;
        }

        int gi = GlobalTrialIndex;
        string raw = GetStringFor(gi, bestColumn).Trim();
        if (string.IsNullOrEmpty(raw))
        {
            bestSpeed = 0f;
            return false;
        }

        bestSpeed = Mathf.Clamp(GetValueFor(gi, bestColumn), minTargetSpeed, maxTargetSpeed);
        return true;
    }

    public void ApplyRecordedBestSpeed()
    {
        if (!adjustmentsUnlocked || isAnimating) return;
        if (!TryGetRecordedBestSpeed(out float bestSpeed)) return;

        SetCurrentTargetSpeed(bestSpeed);
        FlashButton(applyBestSpeedButton);
        Debug.Log($"[BallFly] ApplyRecordedBestSpeed = {bestSpeed:0.###}");
    }

    public void ResetToInitialSpeed()
    {
        if (!adjustmentsUnlocked || isAnimating) return;

        SetCurrentTargetSpeed(BaseSpeed());
        FlashButton(resetSpeedButton);
        Debug.Log($"[BallFly] ResetToInitialSpeed = {BaseSpeed():0.###}");
    }

    public void SnapLower()
    {
        if (!adjustmentsUnlocked || isAnimating || !_snapBest.HasValue) return;
        _snapLower = CurrentTargetSpeed();
        SetButtonCaptured(lowerButton, true);
        FlashButton(lowerButton);
        Debug.Log($"[BallFly] SnapLower = {_snapLower.Value:0.###}");
        PersistSnapshotsForCurrentTrial();
        UpdateNextButtonState();
    }

    public void SnapBest()
    {
        if (!adjustmentsUnlocked || isAnimating) return;
        _snapBest = CurrentTargetSpeed();
        SetButtonCaptured(bestButton, true);
        FlashButton(bestButton);

        Debug.Log($"[BallFly] SnapBest = {_snapBest.Value:0.###}");
        PersistSnapshotsForCurrentTrial();
        SetAllControls(true);
        UpdateNextButtonState();
    }

    public void SnapUpper()
    {
        if (!adjustmentsUnlocked || isAnimating || !_snapBest.HasValue) return;
        _snapUpper = CurrentTargetSpeed();
        SetButtonCaptured(upperButton, true);
        FlashButton(upperButton);

        Debug.Log($"[BallFly] SnapUpper = {_snapUpper.Value:0.###}");
        PersistSnapshotsForCurrentTrial();
        UpdateNextButtonState();
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
        Debug.LogWarning("[Trial] 未輸入完整的 lower/best/upper，無法進入下一個 trial。");
    }

    private int GlobalTrialIndex
    {
        get
        {
            if (_blockOffsets == null || _blockOffsets.Length == 0) BuildBlockOffsets();
            int b = Mathf.Clamp(currentBlock, 0, blockSizes.Length - 1);
            int size = Mathf.Max(1, blockSizes[b]);
            int t = Mathf.Clamp(trialInBlock, 0, size - 1);
            int gi = _blockOffsets[b] + t;
            return Mathf.Clamp(gi, 0, Mathf.Max(0, _rows.Count - 1));
        }
    }

    private void LoadBaseSpeedForCurrentTrial()
    {
        _baseSpeedThisTrial = fallbackInitialSpeed;
    }

    private float? GetOptionalValueFor(int rowIdx, string colName)
    {
        if (_rows.Count == 0 || rowIdx < 0 || rowIdx >= _rows.Count) return null;
        if (!_colIndex.TryGetValue(colName, out int c)) return null;

        var s = _rows[rowIdx][c]?.Trim();
        if (string.IsNullOrEmpty(s)) return null;

        if (float.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float v))
            return v;

        if (float.TryParse(s, out v)) return v;
        return null;
    }

    private void LoadSnapshotsForCurrentTrial()
    {
        int gi = GlobalTrialIndex;
        _snapLower = GetOptionalValueFor(gi, lowerColumn);
        _snapBest = GetOptionalValueFor(gi, bestColumn);
        _snapUpper = GetOptionalValueFor(gi, upperColumn);

        SetButtonCaptured(lowerButton, _snapLower.HasValue);
        SetButtonCaptured(bestButton, _snapBest.HasValue);
        SetButtonCaptured(upperButton, _snapUpper.HasValue);
    }

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
                "UserID", "Trial_no", abAdColumn, exColumn, methodColumn, objectTypeColumn,
                blockColumn, bestColumn, lowerColumn, upperColumn, durationColumn
            };
            _rows = new List<string[]>();
            _colIndex = BuildIndex(_headers);
            File.WriteAllLines(full, new[] { JoinCsvLine(_headers.ToArray()) });
            Debug.Log($"[CSV] 未找到，已建立新檔：{full}");
            return;
        }

        var lines = File.ReadAllLines(full);
        if (lines.Length == 0)
        {
            _headers = new List<string>
            {
                "UserID", "Trial_no", abAdColumn, exColumn, methodColumn, objectTypeColumn,
                blockColumn, bestColumn, lowerColumn, upperColumn, durationColumn
            };
            _rows = new List<string[]>();
            _colIndex = BuildIndex(_headers);
            File.WriteAllLines(full, new[] { JoinCsvLine(_headers.ToArray()) });
            Debug.Log($"[CSV] 空檔，已寫入表頭：{full}");
            return;
        }

        _headers = new List<string>(ParseCsvLine(lines[0]));
        EnsureColumn("UserID");
        EnsureColumn("Trial_no");
        EnsureColumn(abAdColumn);
        EnsureColumn(exColumn);
        EnsureColumn(methodColumn);
        EnsureColumn(objectTypeColumn);
        EnsureColumn(blockColumn);
        EnsureColumn(lowerColumn);
        EnsureColumn(bestColumn);
        EnsureColumn(upperColumn);
        EnsureColumn(durationColumn);
        _colIndex = BuildIndex(_headers);

        _rows = new List<string[]>();
        for (int i = 1; i < lines.Length; i++)
        {
            var cellsList = ParseCsvLine(lines[i]);
            var cells = cellsList.ToArray();
            ArrayResizeOrPad(ref cells, _headers.Count);
            _rows.Add(cells);
        }

        Debug.Log($"[CSV] 載入：{full} | 列數={_rows.Count}");
        if (_rows.Count == 0)
            Debug.LogWarning("[CSV] trial 資料列為 0，ab_ad_angle / ex_angle 會讀成 0，馬達不會動。請確認 spreadsheetPath 與 userId。");
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
        Debug.Log($"[CSV] 已儲存：{full}（{_rows.Count} 列）");
    }

    private void WriteBackSnapshots(int rowIdx)
    {
        if (_rows.Count == 0 || rowIdx < 0 || rowIdx >= _rows.Count) return;
        var row = _rows[rowIdx];
        EnsureColumn(lowerColumn);
        EnsureColumn(bestColumn);
        EnsureColumn(upperColumn);
        EnsureColumn(durationColumn);
        _colIndex = BuildIndex(_headers);

        if (_snapLower.HasValue) row[_colIndex[lowerColumn]] = _snapLower.Value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        if (_snapBest.HasValue) row[_colIndex[bestColumn]] = _snapBest.Value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        if (_snapUpper.HasValue) row[_colIndex[upperColumn]] = _snapUpper.Value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);

        float duration = Mathf.Max(0f, Time.time - trialStartTime);
        row[_colIndex[durationColumn]] = duration.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
    }

    private float GetValueFor(int rowIdx, string colName)
    {
        if (_rows.Count == 0 || rowIdx < 0 || rowIdx >= _rows.Count) return 0f;
        if (!_colIndex.TryGetValue(colName, out int c)) return 0f;

        var s = _rows[rowIdx][c]?.Trim();
        if (string.IsNullOrEmpty(s)) return 0f;

        if (float.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float v))
            return v;

        if (float.TryParse(s, out v)) return v;
        return 0f;
    }

    private string GetStringFor(int rowIdx, string colName)
    {
        if (_rows.Count == 0 || rowIdx < 0 || rowIdx >= _rows.Count) return "";
        if (!_colIndex.TryGetValue(colName, out int c)) return "";
        return _rows[rowIdx][c] ?? "";
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
                else sb.Append(ch);
            }
            else
            {
                if (ch == ',')
                {
                    res.Add(sb.ToString());
                    sb.Length = 0;
                }
                else if (ch == '\"') inQuotes = true;
                else sb.Append(ch);
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

        string relative = (p ?? "").Trim().Replace('\\', '/');
        string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        string trimmedAssetsPrefix = relative.StartsWith("Assets/", StringComparison.OrdinalIgnoreCase)
            ? relative.Substring("Assets/".Length)
            : relative;

        string[] candidates = new[]
        {
            Path.GetFullPath(Path.Combine(Application.dataPath, relative)),
            Path.GetFullPath(Path.Combine(Application.dataPath, "Assets", trimmedAssetsPrefix)),
            Path.GetFullPath(Path.Combine(projectRoot, relative))
        };

        foreach (string candidate in candidates)
        {
            if (File.Exists(candidate))
                return candidate;
        }

        return candidates[0];
    }

    private string ResolvePathWithUserId(string p)
    {
        string withId = (p ?? "").Replace("{UserID}", userId.ToString());
        return ResolvePath(withId);
    }

    private void BuildBlockOffsets()
    {
        if (blockSizes == null || blockSizes.Length == 0) blockSizes = new int[] { 1 };
        _blockOffsets = new int[blockSizes.Length];
        int acc = 0;
        for (int i = 0; i < blockSizes.Length; i++)
        {
            _blockOffsets[i] = acc;
            acc += Mathf.Max(0, blockSizes[i]);
        }
    }

    private void SetAllControls(bool on)
    {
        bool canUseMain = on && !_isPreparingPlay;
        bool canUseLowerUpper = canUseMain && adjustmentsUnlocked && _snapBest.HasValue;

        if (playButton) playButton.interactable = canUseMain && !isAnimating;
        if (lowerButton) lowerButton.interactable = canUseLowerUpper;
        if (bestButton) bestButton.interactable = canUseMain && adjustmentsUnlocked;
        if (upperButton) upperButton.interactable = canUseLowerUpper;
        if (applyBestSpeedButton) applyBestSpeedButton.interactable = canUseMain && adjustmentsUnlocked;
        if (resetSpeedButton) resetSpeedButton.interactable = canUseMain && adjustmentsUnlocked;
        if (nextTrialButton) nextTrialButton.interactable = canUseMain && HasAllSnapshots();
    }

    private void BuildHotkeyHelpText()
    {
        if (!hotkeyHelpLabel) return;
        hotkeyHelpLabel.text =
            "Hotkeys:\n" +
            "B: Record Baseline\n" +
            "I: Go Initial\n" +
            "O: Reboot Motors\n" +
            "T: Manual motor test\n" +
            "W / ↑ : +" + stepSmall + " u/s\n" +
            "S / ↓ : -" + stepSmall + " u/s\n" +
            "E : +" + stepLarge + " u/s   D : -" + stepLarge + " u/s\n" +
            "R: apply best   F: reset initial\n" +
            "Z: lower   X: best   C: upper\n" +
            "Space: Play   Enter: Next";
    }

    private void UpdateTrialInfoLabel()
    {
        if (!trialInfoLabel) return;

        int blocks = Mathf.Max(1, blockSizes.Length);
        int b = Mathf.Clamp(currentBlock, 0, blocks - 1);
        int inCount = Mathf.Clamp(blockSizes[b], 1, int.MaxValue);
        int t = Mathf.Clamp(trialInBlock, 0, inCount - 1);

        int gi = GlobalTrialIndex;
        float ab = GetValueFor(gi, abAdColumn);
        float ex = GetValueFor(gi, exColumn);
        string trialNo = GetStringFor(gi, trialNoColumn);
        string method = GetStringFor(gi, methodColumn);
        string obj = GetStringFor(gi, objectTypeColumn);
        string csvBlock = GetStringFor(gi, blockColumn);

        trialInfoLabel.text =
            $"MethodBlock {b + 1}/{blocks} | Trial {t + 1}/{inCount}\n" +
            $"Trial_no={trialNo} | CSV Block={csvBlock}\n" +
            $"Method={method} | Obj={obj}\n" +
            $"speed={CurrentTargetSpeed():0.###} u/s\n" +
            $"ab_ad_angle={ab:0.###}, ex_angle={ex:0.###}";
    }

    public void TryNext()
    {
        FlashButton(nextTrialButton);

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

        NextTrial();
    }

    public void NextTrial()
    {
        if (!HasAllSnapshots())
        {
            WarnNextGuard();
            return;
        }

        int size = blockSizes[Mathf.Clamp(currentBlock, 0, blockSizes.Length - 1)];
        if (trialInBlock + 1 < size)
        {
            trialInBlock++;
            AfterMoveToNewTrial();
        }
        else
        {
            if (currentBlock + 1 < blockSizes.Length)
            {
                StopPreviewLoop();
                SetAllControls(false);
                if (nextTrialButton) nextTrialButton.interactable = false;

                if (restPanel) restPanel.SetActive(true);
                if (restMessageLabel) restMessageLabel.text = "Rest for a while, then click Next to continue.";
            }
            else
            {
                StopPreviewLoop();
                SetAllControls(false);
                if (nextGuardLabel) nextGuardLabel.text = "Experiment completed. Thank you!";
                Debug.Log("[Block] 全部完成！");
            }
        }
    }

    private void GoNextBlock()
    {
        if (!(currentBlock + 1 < blockSizes.Length)) return;
        currentBlock++;
        trialInBlock = 0;

        if (restPanel) restPanel.SetActive(false);
        AfterMoveToNewTrial();
    }

    private void AfterMoveToNewTrial()
    {
        _snapLower = _snapBest = _snapUpper = null;
        InitSnapButton(lowerButton, "Lowerbound", snapIdleColor, snapTextColor);
        InitSnapButton(bestButton, "Best", snapIdleColor, snapTextColor);
        InitSnapButton(upperButton, "Upperbound", snapIdleColor, snapTextColor);

        _deltaSpeed = 0f;
        _firstPlayThisTrial = true;
        adjustmentsUnlocked = false;

        StopPreviewLoop();
        ResetBallToStart();

        LoadBaseSpeedForCurrentTrial();
        LoadSnapshotsForCurrentTrial();
        UpdateTrialInfoLabel();
        UpdateNextButtonState();

        SetAllControls(true);
        if (nextTrialButton) nextTrialButton.interactable = false;
    }

    private void FlashButton(Button btn)
    {
        if (btn == null) return;

        if (_buttonFlashCos.TryGetValue(btn, out Coroutine oldCo) && oldCo != null)
            StopCoroutine(oldCo);

        Coroutine co = StartCoroutine(FlashButtonCo(btn));
        _buttonFlashCos[btn] = co;
    }

    private IEnumerator FlashButtonCo(Button btn)
    {
        if (btn == null) yield break;

        Image img = btn.GetComponent<Image>();
        if (img == null) yield break;

        Color original = img.color;
        img.color = buttonFlashColor;

        float dur = Mathf.Max(0.01f, buttonFlashDuration);
        float t = 0f;

        while (t < 1f)
        {
            t += Time.deltaTime / dur;
            img.color = Color.Lerp(buttonFlashColor, original, Mathf.Clamp01(t));
            yield return null;
        }

        img.color = original;

        if (_buttonFlashCos.ContainsKey(btn))
            _buttonFlashCos[btn] = null;
    }
}
