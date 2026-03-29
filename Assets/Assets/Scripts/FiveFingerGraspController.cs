using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections;
using System;
using System.IO;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine.EventSystems;

public class FiveFingerGraspController : MonoBehaviour
{
    public enum DeformMode
    {
        Parametric,
        HookeLike,
        SizeOnlyHooke
    }

    public enum ControlMode
    {
        LocalPreview,
        PythonDriven
    }

    public enum DeformInputSource
    {
        Force,
        MotorDisplacement
    }

    [Header("Control Mode")]
    public ControlMode controlMode = ControlMode.PythonDriven;

    [Header("Python Bridge")]
    public PythonBridge_stiffness pythonBridge;
    [Tooltip("收到 K 時是否在 Console 顯示")]
    public bool logHardwareReady = true;

    [Header("Auto Init")]
    public bool autoInitializeOnStart = false;
    [Tooltip("送 B 後，等待多久再送 N")]
    public float baselineToInitialDelay = 0.8f;
    [Tooltip("場景開始後多久才自動初始化")]
    public float startupInitDelay = 0.5f;
    [Tooltip("目前是否正在跑初始化流程")]
    public bool isInitializingHardware = false;

    [Header("Hotkeys (PythonDriven backend door)")]
    public bool enableHardwareHotkeys = true;

    [Header("Mode")]
    [Tooltip("HookeLike：五點局部凹陷；SizeOnlyHooke：整顆球只做大小變化")]
    public DeformMode mode = DeformMode.HookeLike;

    [Header("Targets")]
    [Tooltip("指向球的 MeshFilter；留空會自動抓場景中的第一個 MeshFilter")]
    public MeshFilter ballMeshFilter;
    [Tooltip("五個接觸點（建議掛在 Ball 底下，位置在球表面附近）")]
    public Transform[] contactPoints = new Transform[5];

    [Header("Stiffness (Visual k only)")]
    [Tooltip("初始彈性係數（Slider 中心=0 代表此值）")]
    public float initialStiffness = 50f;
    public float minStiffness = 5f;
    public float maxStiffness = 200f;
    [Tooltip("+/- 按鈕的步進（絕對單位）")]
    public float stiffnessStep = 5f;

    [Header("Hooke-like (x = F/k)")]
    [Tooltip("本地預覽模式下，抓握到最緊(grip=1)時的『總力』F")]
    public float fullGripForce = 10f;
    [Tooltip("把力換成位移的比例：位移 = (F/k) * 這個係數")]
    public float forceToUnitScale = 0.02f;
    [Tooltip("PythonDriven 模式下，用來把 force 正規化成 grip 的最大期望值")]
    public float maxExpectedForce = 5f;

    [Header("Deformation Input")]
    public DeformInputSource deformInputSource = DeformInputSource.Force;

    [Header("Displacement Visual Scaling")]
    [Tooltip("位移模式下的視覺參考 k。CurrentStiffness 越小，變形越大；越大，變形越小")]
    public float displacementReferenceStiffness = 50f;

    [Tooltip("若用馬達位移模式，將 Python 傳來的位移乘這個比例變成視覺位移")]
    public float displacementToUnitScale = 25f;

    [Header("Size-Only Hooke")]
    [Tooltip("球體大小變化的基礎比例。半徑變化約為 x = F/k，再乘這個係數")]
    public float sizeModeScaleFactor = 0.5f;
    [Tooltip("避免球體縮太小")]
    public float minScaleMultiplier = 0.75f;
    [Tooltip("避免球體放太大")]
    public float maxScaleMultiplier = 1.35f;
    [Tooltip("正 force 時是放大還是縮小。true=放大，false=縮小")]
    public bool positiveForceMakesBallBigger = false;

    [Header("Parametric（舊法，可當參考）")]
    [Range(0.0f, 0.6f)] public float baseMaxIndentByRadius = 0.12f;

    [Header("Shared Deform Settings")]
    [Range(0.05f, 1.0f)] public float contactRadiusByRadius = 0.35f;
    [Range(0.5f, 3.0f)] public float perVertexIndentCapFactor = 1.2f;

    [Header("Pinch Enhancement (讓接觸點更尖、更明顯)")]
    [Tooltip("越小越尖，越像手指尖捏進去")]
    [Range(0.02f, 0.5f)] public float localPinchRadiusByRadius = 0.12f;
    [Tooltip("接觸點局部凹陷額外加權")]
    [Range(0f, 6f)] public float localPinchStrength = 2.5f;
    [Tooltip("整顆球的柔和整體變形權重")]
    [Range(0f, 2f)] public float globalSoftnessWeight = 0.6f;

    [Header("Timings (LocalPreview only)")]
    public float holdAtStartSeconds = 1f;
    public float squeezeDuration = 0.6f;
    public float releaseDuration = 0.6f;
    public float holdAtEndSeconds = 1f;

    [Header("Curves (LocalPreview only)")]
    public AnimationCurve squeezeCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);
    public AnimationCurve releaseCurve = AnimationCurve.EaseInOut(0, 1, 1, 0);

    [Header("UI - Existing k controls")]
    public Button playButton;
    public Slider deltaSlider;
    public Button plus5Button;
    public Button minus5Button;
    public TextMeshProUGUI stiffnessLabel;

    [Header("UI - Force Bar")]
    [Tooltip("Fill 用 Simple Image，不要用 Filled")]
    public Image forceFillImage;
    public RectTransform forceFillRect;
    public RectTransform forceTrackRect;
    public RectTransform forceMarkerLine;
    public TextMeshProUGUI forceValueLabel;
    public TextMeshProUGUI pythonStatusLabel;
    public float barMinForce = 0f;
    public float barMaxForce = 5f;
    [Tooltip("紅色標準線位置，例如 2 代表超過 2 就算過線")]
    public float barTargetForce = 2f;
    [Tooltip("顏色漸變速度。越大，前段越慢變紅")]
    public float barColorCurvePower = 2.5f;
    public Color zeroColor = Color.white;
    public Color highColor = Color.red;

    [Header("Runtime (Read Only)")]
    public float latestForce = 0f;
    public float latestMotorDisplacement = 0f;
    public bool hardwareReady = false;
    public string currentTrialCode = "";
    public bool autoUnlockAdjustmentsInPythonMode = true;

    [Header("Trial UI")]
    public Button bestButton;
    public Button upperButton;
    public Button lowerButton;
    public TextMeshProUGUI hotkeyHelpLabel;
    public TextMeshProUGUI nextGuardLabel;
    public string nextGuardMessage = "Please select Best first, then Upper/Lower, then Next.";

    [Header("Trial Info")]
    public TextMeshProUGUI trialInfoLabel;

    [Header("Button Visual (ref: SizePlayController style)")]
    public Color snapIdleColor = new Color32(84, 110, 122, 255);
    public Color snapDoneColor = new Color32(46, 125, 50, 255);
    public Color disabledOverlayColor = new Color32(84, 110, 122, 110);
    public Color snapTextColor = Color.white;

    [Header("Visual K Adjust")]
    public float stepSmall = 1f;
    public float stepLarge = 5f;

    [Header("CSV")]
    [Tooltip("目前結構：Assets/Data/trial_size/Trial_Size_{UserID}.csv")]
    public string spreadsheetPath = "Assets/Data/trial_stiffness/user_{UserID}_stiffness_trials.csv";
    [Tooltip("請填 1~12；不要填 001，因為你的檔名是 Trial_Size_1.csv 這種格式")]
    public string userId = "1";
    [Header("Start From")]
    [Tooltip("Override startup user/trial selection.")]
    public bool overrideStartUserAndTrial = false;
    [Tooltip("Optional override for userId on Start.")]
    public string startUserId = "";
    [Tooltip("Trial number to start from.")]
    public int startTrialNumber = 1;
    [Tooltip("True = find by CSV Trial_no. False = use 1-based row number.")]
    public bool startFromCsvTrialNo = true;

    [Header("CSV Column Names")]
    public string colUserId = "UserID";
    public string colTrialNo = "Trial_no";
    public string colK = "k";
    public string colMethod = "Method";
    public string colObjectType = "ObjectType";
    public string colRep = "Repitition";
    public string colBlock = "Block";
    public string colBest = "best";
    public string colLower = "lowerbound";
    public string colUpper = "upperbound";
    public string colDuration = "duration";

    [Header("Trial Runtime")]
    public int currentTrialIndex = 0;
    public bool autoLoadFirstTrialOnStart = true;

    private List<string> _headers = new List<string>();
    private List<string[]> _rows = new List<string[]>();
    private Dictionary<string, int> _colIndex = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);

    private float? _snapBest = null;
    private float? _snapUpper = null;
    private float? _snapLower = null;

    private bool _hasStartedAnyTrial = false;
    private bool _trialLoaded = false;
    private bool _trialRunning = false;
    private float _trialStartTime = 0f;

    private string _currentPythonTrialCode = "";

    private bool isAnimating = false;
    private bool adjustmentsUnlocked = false;
    private Coroutine loopCo = null;

    private Mesh _mesh;
    private Vector3[] _baseVerts, _workVerts, _baseNormals;
    private float _sphereRadius;
    private Vector3 _baseLocalScale;

    private float DeltaStiffness => deltaSlider ? deltaSlider.value : 0f;

    private float CurrentStiffness()
    {
        float k = initialStiffness + DeltaStiffness;
        return Mathf.Clamp(k, minStiffness, maxStiffness);
    }

    private bool HasBest => _snapBest.HasValue;
    private bool HasUpper => _snapUpper.HasValue;
    private bool HasLower => _snapLower.HasValue;

    private bool CanPickBest => adjustmentsUnlocked && !isAnimating;
    private bool CanPickUpper => adjustmentsUnlocked && !isAnimating && HasBest && !HasUpper;
    private bool CanPickLower => adjustmentsUnlocked && !isAnimating && HasBest && !HasLower;
    private bool CanGoNext => HasBest && HasUpper && HasLower;

    private string ResolvedCsvPath
    {
        get
        {
            string paddedUserId = userId;

            if (int.TryParse(userId, out int uid))
                paddedUserId = uid.ToString("00");   // 1 -> 01, 2 -> 02

            string p = (spreadsheetPath ?? "").Replace("{UserID}", paddedUserId);

            if (Path.IsPathRooted(p)) return p;

            string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
            return Path.GetFullPath(Path.Combine(projectRoot, p));
        }
    }

    void Awake()
    {
        if (!ballMeshFilter) ballMeshFilter = FindObjectOfType<MeshFilter>();
        if (!ballMeshFilter || !ballMeshFilter.sharedMesh)
        {
            Debug.LogError("[FiveFingerGrasp] 找不到球的 MeshFilter/mesh");
            enabled = false;
            return;
        }

        _baseLocalScale = ballMeshFilter.transform.localScale;

        _mesh = Instantiate(ballMeshFilter.sharedMesh);
        _mesh.MarkDynamic();
        ballMeshFilter.sharedMesh = _mesh;

        _baseVerts = _mesh.vertices;
        _workVerts = new Vector3[_baseVerts.Length];
        _baseNormals = _mesh.normals;

        _sphereRadius = 0.0001f;
        for (int i = 0; i < _baseVerts.Length; i++)
            _sphereRadius = Mathf.Max(_sphereRadius, _baseVerts[i].magnitude);

        ApplyBaseShape();
    }

    void Start()
    {
        if (deltaSlider)
        {
            deltaSlider.minValue = minStiffness - initialStiffness;
            deltaSlider.maxValue = maxStiffness - initialStiffness;
            deltaSlider.value = 0f;
            deltaSlider.wholeNumbers = false;
            deltaSlider.onValueChanged.AddListener(OnSliderChanged);
        }

        if (plus5Button) plus5Button.onClick.AddListener(() => Nudge(+stiffnessStep));
        if (minus5Button) minus5Button.onClick.AddListener(() => Nudge(-stiffnessStep));

        if (playButton)
            playButton.onClick.AddListener(OnPlayOrNextTrialButtonClicked);

        if (bestButton) bestButton.onClick.AddListener(SnapBest);
        if (upperButton) upperButton.onClick.AddListener(SnapUpper);
        if (lowerButton) lowerButton.onClick.AddListener(SnapLower);

        if (controlMode == ControlMode.LocalPreview)
        {
            SetAdjustControls(false);
            if (playButton) playButton.interactable = true;
        }
        else
        {
            adjustmentsUnlocked = autoUnlockAdjustmentsInPythonMode;
            SetAdjustControls(adjustmentsUnlocked);
            if (playButton) playButton.interactable = true;
        }

        if (pythonBridge != null)
        {
            pythonBridge.OnHardwareReady += HandlePythonHardwareReady;
            pythonBridge.OnForceUpdated += HandlePythonForceUpdated;
            pythonBridge.OnDisplacementUpdated += HandlePythonDisplacementUpdated;
        }

        ApplyStartSelection();
        LoadCsvOrCreate();
        if (autoLoadFirstTrialOnStart)
            LoadTrial(ResolveStartTrialIndex());

        InitSnapButton(bestButton, "Best", snapIdleColor, snapTextColor);
        InitSnapButton(upperButton, "Upper", snapIdleColor, snapTextColor);
        InitSnapButton(lowerButton, "Lower", snapIdleColor, snapTextColor);

        RefreshPlayButtonLabel();
        RefreshLabel();
        RefreshPythonStatusLabel();
        RefreshForceBar(0f);
        BuildHotkeyHelpText();
        RefreshTrialInfoLabel();
        RefreshTrialButtonsAndNextState();

        if (controlMode == ControlMode.PythonDriven && autoInitializeOnStart)
            StartCoroutine(AutoInitializeAfterDelay());
    }

    void Update()
    {
        if (controlMode == ControlMode.PythonDriven)
        {
            if (pythonBridge != null)
            {
                latestForce = pythonBridge.latestForce;
                latestMotorDisplacement = pythonBridge.latestDisplacement;
                hardwareReady = pythonBridge.hardwareReady;

                // Debug.Log("D" + latestMotorDisplacement);
            }            

            RefreshPythonStatusLabel();
            RefreshForceBar(latestForce);

            float driveForce = latestForce;
            float driveDisp = latestMotorDisplacement;
            float visualK = CurrentStiffness();

            if (mode == DeformMode.SizeOnlyHooke)
            {
                if (deformInputSource == DeformInputSource.Force)
                    ApplySizeOnlyFromForce(driveForce, visualK);
                else
                    ApplySizeOnlyFromDisplacement(driveDisp, visualK);

                ResetMeshToBase();
            }
            else
            {
                ResetScaleToBase();

                if (deformInputSource == DeformInputSource.Force)
                    ApplyDeformationFromForce(driveForce, visualK);
                else
                    ApplyDeformationFromDisplacement(driveDisp, visualK);
            }

            if (enableHardwareHotkeys)
            {
                HandleHardwareHotkeys();
            }

            HandleTrialHotkeys();
        }
    }

    void OnDestroy()
    {
        if (pythonBridge != null)
        {
            pythonBridge.OnHardwareReady -= HandlePythonHardwareReady;
            pythonBridge.OnForceUpdated -= HandlePythonForceUpdated;
            pythonBridge.OnDisplacementUpdated -= HandlePythonDisplacementUpdated;
        }
    }

    private void HandlePythonHardwareReady()
    {
        hardwareReady = true;
        RefreshPythonStatusLabel();

        if (logHardwareReady)
            Debug.Log("[FiveFingerGrasp] Hardware ready (K received)");
    }

    private void HandlePythonForceUpdated(float force)
    {
        latestForce = force;
    }

    private void HandlePythonDisplacementUpdated(float displacement)
    {
        latestMotorDisplacement = displacement;
    }

    private void HandleHardwareHotkeys()
    {
        if (Input.GetKeyDown(KeyCode.B))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey B");
            SendBaseline();
        }

        if (Input.GetKeyDown(KeyCode.N))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey N");
            SendInitial();
        }

        if (Input.GetKeyDown(KeyCode.M))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey M");
            SendNext();
        }

        if (Input.GetKeyDown(KeyCode.Q))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey Q");
            SendQuit();
        }

        if (Input.GetKeyDown(KeyCode.Alpha1) || Input.GetKeyDown(KeyCode.Keypad1))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey 1 -> bs");
            StartTrialBS();
        }

        if (Input.GetKeyDown(KeyCode.Alpha2) || Input.GetKeyDown(KeyCode.Keypad2))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey 2 -> bm");
            StartTrialBM();
        }

        if (Input.GetKeyDown(KeyCode.Alpha3) || Input.GetKeyDown(KeyCode.Keypad3))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey 3 -> bh");
            StartTrialBH();
        }

        if (Input.GetKeyDown(KeyCode.Alpha4) || Input.GetKeyDown(KeyCode.Keypad4))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey 4 -> es");
            StartTrialES();
        }

        if (Input.GetKeyDown(KeyCode.Alpha5) || Input.GetKeyDown(KeyCode.Keypad5))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey 5 -> em");
            StartTrialEM();
        }

        if (Input.GetKeyDown(KeyCode.Alpha6) || Input.GetKeyDown(KeyCode.Keypad6))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey 6 -> eh");
            StartTrialEH();
        }

        if (Input.GetKeyDown(KeyCode.P))
        {
            Debug.Log("[FiveFingerGrasp] Hotkey P");
            StartCurrentTrialFromButton();
        }
    }

    private void HandleTrialHotkeys()
    {
        if (EventSystem.current != null)
        {
            var go = EventSystem.current.currentSelectedGameObject;
            if (go && (go.GetComponent<TMP_InputField>() || go.GetComponent<InputField>()))
                return;
        }

        if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter)|| Input.GetKeyDown(KeyCode.V))
        {
            OnPlayOrNextTrialButtonClicked();
        }

        if (!adjustmentsUnlocked || isAnimating) return;

        if (Input.GetKeyDown(KeyCode.W) || Input.GetKeyDown(KeyCode.UpArrow))
            Nudge(+stepSmall);

        if (Input.GetKeyDown(KeyCode.S) || Input.GetKeyDown(KeyCode.DownArrow))
            Nudge(-stepSmall);

        if (Input.GetKeyDown(KeyCode.E))
            Nudge(+stepLarge);

        if (Input.GetKeyDown(KeyCode.D))
            Nudge(-stepLarge);

        if (Input.GetKeyDown(KeyCode.X) && CanPickBest)
            SnapBest();

        if (Input.GetKeyDown(KeyCode.Z) && CanPickLower)
            SnapLower();

        if (Input.GetKeyDown(KeyCode.C) && CanPickUpper)
            SnapUpper();
    }

    private IEnumerator AutoInitializeAfterDelay()
    {
        yield return new WaitForSeconds(startupInitDelay);
        InitializeHardware();
    }

    public void InitializeHardware()
    {
        if (controlMode != ControlMode.PythonDriven) return;
        if (pythonBridge == null) return;
        if (isInitializingHardware) return;

        StartCoroutine(InitializeHardwareRoutine());
    }

    public void InitializeHardwareAndStartCurrentTrial()
    {
        if (controlMode != ControlMode.PythonDriven) return;
        if (pythonBridge == null) return;
        if (isInitializingHardware) return;

        StartCoroutine(InitializeAndStartTrialRoutine());
    }

    private IEnumerator InitializeHardwareRoutine()
    {
        isInitializingHardware = true;
        hardwareReady = false;
        RefreshPythonStatusLabel();

        _ = pythonBridge.SendBaselineAsync();
        yield return new WaitForSeconds(baselineToInitialDelay);

        _ = pythonBridge.SendInitialAsync();

        float timeout = 10f;
        float t = 0f;
        while (pythonBridge != null && !pythonBridge.hardwareReady && t < timeout)
        {
            t += Time.deltaTime;
            yield return null;
        }

        hardwareReady = (pythonBridge != null && pythonBridge.hardwareReady);
        RefreshPythonStatusLabel();
        isInitializingHardware = false;

        if (!hardwareReady)
            Debug.LogWarning("[FiveFingerGrasp] InitializeHardwareRoutine timeout: did not receive K.");
    }

    private IEnumerator InitializeAndStartTrialRoutine()
    {
        yield return StartCoroutine(InitializeHardwareRoutine());

        if (!hardwareReady) yield break;
        if (string.IsNullOrWhiteSpace(currentTrialCode)) yield break;

        _ = pythonBridge.SendTrialAsync(currentTrialCode);
    }

    public void PlayOnce()
    {
        if (controlMode != ControlMode.LocalPreview)
        {
            StartCurrentTrialFromButton();
            return;
        }

        if (isAnimating) return;
        if (loopCo != null) { StopCoroutine(loopCo); loopCo = null; }
        StartCoroutine(PlayOnceRoutine());
    }

    public async void SendBaseline()
    {
        if (pythonBridge == null) return;
        await pythonBridge.SendBaselineAsync();
    }

    public async void SendInitial()
    {
        if (pythonBridge == null) return;
        hardwareReady = false;
        RefreshPythonStatusLabel();
        await pythonBridge.SendInitialAsync();
    }

    public async void SendNext()
    {
        if (pythonBridge == null) return;
        hardwareReady = false;
        RefreshPythonStatusLabel();
        await pythonBridge.SendNextAsync();
    }

    public async void SendQuit()
    {
        if (pythonBridge == null) return;
        await pythonBridge.SendQuitAsync();
    }

    public void SelectTrialBS() => SetCurrentTrial("bs");
    public void SelectTrialBM() => SetCurrentTrial("bm");
    public void SelectTrialBH() => SetCurrentTrial("bh");
    public void SelectTrialES() => SetCurrentTrial("es");
    public void SelectTrialEM() => SetCurrentTrial("em");
    public void SelectTrialEH() => SetCurrentTrial("eh");

    public void SetCurrentTrial(string code)
    {
        currentTrialCode = code;
        RefreshPythonStatusLabel();
    }

    public void StartCurrentTrialFromButton()
    {
        if (controlMode != ControlMode.PythonDriven) return;
        if (pythonBridge == null) return;

        if (string.IsNullOrWhiteSpace(currentTrialCode))
        {
            currentTrialCode = "bs";
            Debug.LogWarning("[FiveFingerGrasp] currentTrialCode was empty, defaulting to bs.");
        }

        StartCoroutine(StartTrialWithAutoInitRoutine());
    }

    private IEnumerator StartTrialWithAutoInitRoutine()
    {
        if (pythonBridge == null) yield break;

        if (!pythonBridge.hardwareReady)
            yield return StartCoroutine(InitializeHardwareRoutine());

        if (!pythonBridge.hardwareReady) yield break;

        _ = pythonBridge.SendTrialAsync(currentTrialCode);
    }

    public async void StartTrialBS()
    {
        Debug.Log("[FiveFingerGrasp] StartTrialBS()");
        SetCurrentTrial("bs");
        if (pythonBridge == null) return;

        if (!pythonBridge.hardwareReady)
            InitializeHardwareAndStartCurrentTrial();
        else
            await pythonBridge.SendTrialAsync("bs");
    }

    public async void StartTrialBM()
    {
        Debug.Log("[FiveFingerGrasp] StartTrialBM()");
        SetCurrentTrial("bm");
        if (pythonBridge == null) return;

        if (!pythonBridge.hardwareReady)
            InitializeHardwareAndStartCurrentTrial();
        else
            await pythonBridge.SendTrialAsync("bm");
    }

    public async void StartTrialBH()
    {
        Debug.Log("[FiveFingerGrasp] StartTrialBH()");
        SetCurrentTrial("bh");
        if (pythonBridge == null) return;

        if (!pythonBridge.hardwareReady)
            InitializeHardwareAndStartCurrentTrial();
        else
            await pythonBridge.SendTrialAsync("bh");
    }

    public async void StartTrialES()
    {
        Debug.Log("[FiveFingerGrasp] StartTrialES()");
        SetCurrentTrial("es");
        if (pythonBridge == null) return;

        if (!pythonBridge.hardwareReady)
            InitializeHardwareAndStartCurrentTrial();
        else
            await pythonBridge.SendTrialAsync("es");
    }

    public async void StartTrialEM()
    {
        Debug.Log("[FiveFingerGrasp] StartTrialEM()");
        SetCurrentTrial("em");
        if (pythonBridge == null) return;

        if (!pythonBridge.hardwareReady)
            InitializeHardwareAndStartCurrentTrial();
        else
            await pythonBridge.SendTrialAsync("em");
    }

    public async void StartTrialEH()
    {
        Debug.Log("[FiveFingerGrasp] StartTrialEH()");
        SetCurrentTrial("eh");
        if (pythonBridge == null) return;

        if (!pythonBridge.hardwareReady)
            InitializeHardwareAndStartCurrentTrial();
        else
            await pythonBridge.SendTrialAsync("eh");
    }

    private void OnSliderChanged(float _)
    {
        if (!adjustmentsUnlocked || isAnimating) return;
        ClampSlider();
        RefreshLabel();
        RefreshTrialButtonsAndNextState();
    }

    private void Nudge(float step)
    {
        if (!deltaSlider || !adjustmentsUnlocked || isAnimating) return;
        float v = Mathf.Clamp(deltaSlider.value + step, deltaSlider.minValue, deltaSlider.maxValue);
        deltaSlider.value = v;
        RefreshTrialButtonsAndNextState();
    }

    private void RefreshPlayButtonLabel()
    {
        if (!playButton) return;
        var txt = playButton.GetComponentInChildren<TextMeshProUGUI>(true);
        if (!txt) return;

        txt.text = _hasStartedAnyTrial ? "Next" : "Start";
    }

    public void OnPlayOrNextTrialButtonClicked()
    {
        if (!_trialLoaded)
        {
            Debug.LogWarning("[FiveFingerGrasp] No trial loaded.");
            return;
        }

        if (!_trialRunning)
        {
            StartCurrentCsvTrial();
            return;
        }

        if (!CanGoNext)
        {
            if (nextGuardLabel) nextGuardLabel.text = nextGuardMessage;
            Debug.LogWarning("[FiveFingerGrasp] Best / Upper / Lower not complete yet.");
            return;
        }

        FinishAndAdvanceTrial();
    }

    private IEnumerator PlayOnceRoutine()
    {
        isAnimating = true;
        SetAllControls(false);

        if (holdAtStartSeconds > 0f)
            yield return new WaitForSeconds(holdAtStartSeconds);

        float t = 0f;
        while (t < 1f)
        {
            t += Time.deltaTime / Mathf.Max(0.0001f, squeezeDuration);
            float grip = squeezeCurve.Evaluate(Mathf.Clamp01(t));

            if (mode == DeformMode.SizeOnlyHooke)
            {
                ApplySizeOnlyFromForce(fullGripForce * grip, CurrentStiffness());
                ResetMeshToBase();
            }
            else
            {
                ResetScaleToBase();
                ApplyDeformation(grip, CurrentStiffness());
            }

            RefreshForceBar(fullGripForce * grip);
            yield return null;
        }

        if (mode != DeformMode.SizeOnlyHooke)
            ApplyDeformation(1f, CurrentStiffness());
        RefreshForceBar(fullGripForce);

        t = 0f;
        while (t < 1f)
        {
            t += Time.deltaTime / Mathf.Max(0.0001f, releaseDuration);
            float grip = releaseCurve.Evaluate(Mathf.Clamp01(t));

            if (mode == DeformMode.SizeOnlyHooke)
            {
                ApplySizeOnlyFromForce(fullGripForce * grip, CurrentStiffness());
                ResetMeshToBase();
            }
            else
            {
                ResetScaleToBase();
                ApplyDeformation(grip, CurrentStiffness());
            }

            RefreshForceBar(fullGripForce * grip);
            yield return null;
        }

        ApplyBaseShape();
        RefreshForceBar(0f);

        if (holdAtEndSeconds > 0f)
            yield return new WaitForSeconds(holdAtEndSeconds);

        isAnimating = false;
        adjustmentsUnlocked = true;
        SetAllControls(true);

        loopCo = StartCoroutine(LoopRoutine());
    }

    private IEnumerator LoopRoutine()
    {
        while (true)
        {
            if (holdAtStartSeconds > 0f)
                yield return new WaitForSeconds(holdAtStartSeconds);

            float t = 0f;
            while (t < 1f)
            {
                t += Time.deltaTime / Mathf.Max(0.0001f, squeezeDuration);
                float grip = squeezeCurve.Evaluate(Mathf.Clamp01(t));

                if (mode == DeformMode.SizeOnlyHooke)
                {
                    ApplySizeOnlyFromForce(fullGripForce * grip, CurrentStiffness());
                    ResetMeshToBase();
                }
                else
                {
                    ResetScaleToBase();
                    ApplyDeformation(grip, CurrentStiffness());
                }

                RefreshForceBar(fullGripForce * grip);
                yield return null;
            }

            t = 0f;
            while (t < 1f)
            {
                t += Time.deltaTime / Mathf.Max(0.0001f, releaseDuration);
                float grip = releaseCurve.Evaluate(Mathf.Clamp01(t));

                if (mode == DeformMode.SizeOnlyHooke)
                {
                    ApplySizeOnlyFromForce(fullGripForce * grip, CurrentStiffness());
                    ResetMeshToBase();
                }
                else
                {
                    ResetScaleToBase();
                    ApplyDeformation(grip, CurrentStiffness());
                }

                RefreshForceBar(fullGripForce * grip);
                yield return null;
            }

            if (holdAtEndSeconds > 0f)
                yield return new WaitForSeconds(holdAtEndSeconds);
        }
    }

    private void ApplyBaseShape()
    {
        ResetScaleToBase();
        ResetMeshToBase();
    }

    private void ResetScaleToBase()
    {
        if (ballMeshFilter != null)
            ballMeshFilter.transform.localScale = _baseLocalScale;
    }

    private void ResetMeshToBase()
    {
        if (_mesh == null || _baseVerts == null) return;
        _mesh.SetVertices(_baseVerts);
        _mesh.RecalculateNormals();
        _mesh.RecalculateBounds();
    }

    private void ApplyDeformation(float grip01, float stiffness)
    {
        if (_mesh == null || _baseVerts == null) return;
        if (contactPoints == null || contactPoints.Length == 0) return;

        var tr = ballMeshFilter.transform;
        int cpCount = contactPoints.Length;
        Vector3[] cps = new Vector3[cpCount];
        for (int i = 0; i < cpCount; i++)
            cps[i] = contactPoints[i] ? tr.InverseTransformPoint(contactPoints[i].position) : Vector3.zero;

        float sigmaGlobal = contactRadiusByRadius * _sphereRadius;
        float sigmaLocal = localPinchRadiusByRadius * _sphereRadius;

        float hookeTotalIndent = 0f;
        float paramIndentMax = 0f;

        if (mode == DeformMode.HookeLike)
        {
            float x = (fullGripForce * Mathf.Clamp01(grip01)) / Mathf.Max(0.0001f, stiffness);
            hookeTotalIndent = x * forceToUnitScale;
        }
        else
        {
            paramIndentMax = baseMaxIndentByRadius * _sphereRadius * (initialStiffness / Mathf.Max(0.0001f, stiffness));
        }

        for (int vi = 0; vi < _baseVerts.Length; vi++)
        {
            Vector3 v0 = _baseVerts[vi];
            Vector3 n0 = (_baseNormals != null && _baseNormals.Length == _baseVerts.Length)
                         ? _baseNormals[vi].normalized
                         : v0.normalized;

            float wGlobal = 0f;
            float wLocalSharp = 0f;

            for (int j = 0; j < cpCount; j++)
            {
                if (!contactPoints[j]) continue;
                float d = (v0 - cps[j]).magnitude;

                wGlobal += Mathf.Exp(-(d * d) / (2f * sigmaGlobal * sigmaGlobal));
                wLocalSharp += Mathf.Exp(-(d * d) / (2f * sigmaLocal * sigmaLocal));
            }

            float softNorm = (wGlobal <= 1e-6f) ? 0f : (wGlobal / (1f + wGlobal));
            float sharpNorm = Mathf.Clamp01(wLocalSharp);

            float indent;
            float cap;

            if (mode == DeformMode.HookeLike)
            {
                float combinedWeight = globalSoftnessWeight * softNorm + localPinchStrength * sharpNorm;
                indent = hookeTotalIndent * combinedWeight;
                cap = perVertexIndentCapFactor * hookeTotalIndent * (1f + localPinchStrength);
            }
            else
            {
                indent = paramIndentMax * Mathf.Clamp01(grip01) * (globalSoftnessWeight * softNorm + localPinchStrength * sharpNorm);
                cap = perVertexIndentCapFactor * paramIndentMax * (1f + localPinchStrength);
            }

            indent = Mathf.Min(indent, cap);
            _workVerts[vi] = v0 - n0 * indent;
        }

        _mesh.SetVertices(_workVerts);
        _mesh.RecalculateNormals();
        _mesh.RecalculateBounds();
    }

    private void ApplyDeformationFromForce(float force, float stiffness)
    {
        if (_mesh == null || _baseVerts == null) return;
        if (contactPoints == null || contactPoints.Length == 0) return;

        var tr = ballMeshFilter.transform;
        int cpCount = contactPoints.Length;
        Vector3[] cps = new Vector3[cpCount];
        for (int i = 0; i < cpCount; i++)
            cps[i] = contactPoints[i] ? tr.InverseTransformPoint(contactPoints[i].position) : Vector3.zero;

        float sigmaGlobal = contactRadiusByRadius * _sphereRadius;
        float sigmaLocal = localPinchRadiusByRadius * _sphereRadius;

        float clampedForce = Mathf.Max(0f, Mathf.Abs(force));

        float hookeTotalIndent = 0f;
        float paramIndentMax = 0f;

        if (mode == DeformMode.HookeLike)
        {
            float x = clampedForce / Mathf.Max(0.0001f, stiffness);
            hookeTotalIndent = x * forceToUnitScale;
        }
        else
        {
            float grip = Mathf.Clamp01(clampedForce / Mathf.Max(0.0001f, maxExpectedForce));
            paramIndentMax = baseMaxIndentByRadius * _sphereRadius *
                             (initialStiffness / Mathf.Max(0.0001f, stiffness)) * grip;
        }

        for (int vi = 0; vi < _baseVerts.Length; vi++)
        {
            Vector3 v0 = _baseVerts[vi];
            Vector3 n0 = (_baseNormals != null && _baseNormals.Length == _baseVerts.Length)
                         ? _baseNormals[vi].normalized
                         : v0.normalized;

            float wGlobal = 0f;
            float wLocalSharp = 0f;

            for (int j = 0; j < cpCount; j++)
            {
                if (!contactPoints[j]) continue;
                float d = (v0 - cps[j]).magnitude;

                wGlobal += Mathf.Exp(-(d * d) / (2f * sigmaGlobal * sigmaGlobal));
                wLocalSharp += Mathf.Exp(-(d * d) / (2f * sigmaLocal * sigmaLocal));
            }

            float softNorm = (wGlobal <= 1e-6f) ? 0f : (wGlobal / (1f + wGlobal));
            float sharpNorm = Mathf.Clamp01(wLocalSharp);

            float indent;
            float cap;

            if (mode == DeformMode.HookeLike)
            {
                float combinedWeight = globalSoftnessWeight * softNorm + localPinchStrength * sharpNorm;
                indent = hookeTotalIndent * combinedWeight;
                cap = perVertexIndentCapFactor * hookeTotalIndent * (1f + localPinchStrength);
            }
            else
            {
                indent = paramIndentMax * (globalSoftnessWeight * softNorm + localPinchStrength * sharpNorm);
                cap = perVertexIndentCapFactor * paramIndentMax * (1f + localPinchStrength);
            }

            indent = Mathf.Min(indent, cap);
            _workVerts[vi] = v0 - n0 * indent;
        }

        _mesh.SetVertices(_workVerts);
        _mesh.RecalculateNormals();
        _mesh.RecalculateBounds();
    }

    private void ApplySizeOnlyFromForce(float force, float stiffness)
    {
        if (ballMeshFilter == null) return;

        float f = Mathf.Abs(force);
        float x = f / Mathf.Max(0.0001f, stiffness);
        float delta = x * sizeModeScaleFactor;

        float multiplier;
        if (positiveForceMakesBallBigger)
            multiplier = 1f + delta;
        else
            multiplier = 1f - delta;

        multiplier = Mathf.Clamp(multiplier, minScaleMultiplier, maxScaleMultiplier);
        ballMeshFilter.transform.localScale = _baseLocalScale * multiplier;
    }

    private void ApplyDeformationFromDisplacement(float displacementM, float stiffness)
    {
        if (_mesh == null || _baseVerts == null) return;
        if (contactPoints == null || contactPoints.Length == 0) return;

        var tr = ballMeshFilter.transform;
        int cpCount = contactPoints.Length;
        Vector3[] cps = new Vector3[cpCount];
        for (int i = 0; i < cpCount; i++)
            cps[i] = contactPoints[i] ? tr.InverseTransformPoint(contactPoints[i].position) : Vector3.zero;

        float sigmaGlobal = contactRadiusByRadius * _sphereRadius;
        float sigmaLocal = localPinchRadiusByRadius * _sphereRadius;
        float stiffnessScale = displacementReferenceStiffness / Mathf.Max(0.0001f, stiffness);
        float hookeTotalIndent = Mathf.Abs(displacementM) * displacementToUnitScale * stiffnessScale;

        for (int vi = 0; vi < _baseVerts.Length; vi++)
        {
            Vector3 v0 = _baseVerts[vi];
            Vector3 n0 = (_baseNormals != null && _baseNormals.Length == _baseVerts.Length)
                        ? _baseNormals[vi].normalized
                        : v0.normalized;

            float wGlobal = 0f;
            float wLocalSharp = 0f;

            for (int j = 0; j < cpCount; j++)
            {
                if (!contactPoints[j]) continue;
                float d = (v0 - cps[j]).magnitude;

                wGlobal += Mathf.Exp(-(d * d) / (2f * sigmaGlobal * sigmaGlobal));
                wLocalSharp += Mathf.Exp(-(d * d) / (2f * sigmaLocal * sigmaLocal));
            }

            float softNorm = (wGlobal <= 1e-6f) ? 0f : (wGlobal / (1f + wGlobal));
            float sharpNorm = Mathf.Clamp01(wLocalSharp);

            float combinedWeight = globalSoftnessWeight * softNorm + localPinchStrength * sharpNorm;
            float indent = hookeTotalIndent * combinedWeight;
            float cap = perVertexIndentCapFactor * hookeTotalIndent * (1f + localPinchStrength);

            indent = Mathf.Min(indent, cap);
            _workVerts[vi] = v0 - n0 * indent;
        }

        _mesh.SetVertices(_workVerts);
        _mesh.RecalculateNormals();
        _mesh.RecalculateBounds();
    }

    private void ApplySizeOnlyFromDisplacement(float displacementM, float stiffness)
    {
        // param. displacementM, 馬達位移 ； param. stiffness 視覺stiffnessk
        if (ballMeshFilter == null) return;

        // 參考 / 視覺k
        float stiffnessScale = displacementReferenceStiffness / Mathf.Max(0.0001f, stiffness);

        float delta = Mathf.Abs(displacementM) * displacementToUnitScale * stiffnessScale;

        // Debug.Log("delta"+ delta);

        float multiplier;

        // 動態判斷變形方向
        // 如果 Trial Code 是以 'e' 開頭 (Extension)，則為正相關 (1 + delta)
        // 如果是 'b' 開頭 (Inter-finger/Both)，則維持負相關 (1 - delta)
        // if (!string.IsNullOrEmpty(currentTrialCode) && currentTrialCode.StartsWith("b"))
        // {
            // Extension 模式：越拉位移越大，球越大
            multiplier = 1f + delta;
        // }
        // else
        // {
            // 預設或 Inter-finger 模式：越抓位移絕對值越大，球越小
        multiplier = 1f - delta;
        // }


        multiplier = Mathf.Clamp(multiplier, minScaleMultiplier, maxScaleMultiplier);
        ballMeshFilter.transform.localScale = _baseLocalScale * multiplier;
        
        // Debug 資訊，方便你確認目前的 multiplier 數值
        // Debug.Log("B"+ ballMeshFilter.transform.localScale);
        // Debug.Log("B"+ ballMeshFilter.transform.localScale + "M" + multiplier);
    }

    private void RefreshForceBar(float force)
    {
        float clamped = Mathf.Clamp(Mathf.Abs(force), barMinForce, barMaxForce);
        float norm = Mathf.InverseLerp(barMinForce, barMaxForce, clamped);

        if (forceFillRect != null && forceTrackRect != null)
        {
            float fullH = forceTrackRect.rect.height;
            float h = fullH * norm;

            forceFillRect.anchorMin = new Vector2(0f, 0f);
            forceFillRect.anchorMax = new Vector2(1f, 0f);
            forceFillRect.pivot = new Vector2(0.5f, 0f);

            forceFillRect.anchoredPosition = Vector2.zero;
            forceFillRect.sizeDelta = new Vector2(0f, h);
        }

        if (forceFillImage != null)
        {
            if (clamped <= 0f)
            {
                forceFillImage.color = zeroColor;
            }
            else if (clamped >= barTargetForce)
            {
                forceFillImage.color = highColor;
            }
            else
            {
                float rawT = Mathf.InverseLerp(0f, barTargetForce, clamped);
                float slowT = Mathf.Pow(rawT, Mathf.Max(1f, barColorCurvePower));
                forceFillImage.color = Color.Lerp(zeroColor, highColor, slowT);
            }
        }

        if (forceMarkerLine != null && forceTrackRect != null)
        {
            float targetNorm = Mathf.InverseLerp(barMinForce, barMaxForce, barTargetForce);
            float y = targetNorm * forceTrackRect.rect.height;
            forceMarkerLine.anchoredPosition = new Vector2(forceMarkerLine.anchoredPosition.x, y);
        }

        if (forceValueLabel != null)
            forceValueLabel.text = clamped.ToString("0.000");
    }

    public void SetTargetForceLine(float target)
    {
        barTargetForce = target;
        RefreshForceBar(latestForce);
    }

    private void RefreshPythonStatusLabel()
    {
        if (pythonStatusLabel == null) return;

        if (controlMode != ControlMode.PythonDriven)
        {
            pythonStatusLabel.text = "Mode: LocalPreview";
            return;
        }

        if (pythonBridge == null)
        {
            pythonStatusLabel.text = "Python: No Bridge";
            return;
        }

        string ready = hardwareReady ? "Ready" : "Waiting";
        string trial = string.IsNullOrWhiteSpace(currentTrialCode) ? "-" : currentTrialCode;
        string init = isInitializingHardware ? " | Init..." : "";
        pythonStatusLabel.text = $"Python: {ready} | Trial: {trial}{init}";
    }

    private void LoadTrial(int rowIdx)
    {
        if (_rows == null || _rows.Count == 0)
        {
            Debug.LogError("[FiveFingerGrasp] CSV has no rows.");
            _trialLoaded = false;
            return;
        }

        currentTrialIndex = Mathf.Clamp(rowIdx, 0, _rows.Count - 1);

        _snapBest = null;
        _snapUpper = null;
        _snapLower = null;
        _trialRunning = false;
        _trialLoaded = true;

        if (deltaSlider)
            deltaSlider.value = 0f;

        InitSnapButton(bestButton, "Best", snapIdleColor, snapTextColor);
        InitSnapButton(upperButton, "Upper", snapIdleColor, snapTextColor);
        InitSnapButton(lowerButton, "Lower", snapIdleColor, snapTextColor);

        RefreshLabel();
        RefreshTrialInfoLabel();
        RefreshTrialButtonsAndNextState();
        RefreshPlayButtonLabel();
    }

    private string BuildTrialCodeFromCsvRow(int rowIdx)
    {
        string method = GetStringFor(rowIdx, colMethod).Trim().ToLowerInvariant();
        string k = GetStringFor(rowIdx, colK).Trim().ToLowerInvariant();

        if (string.IsNullOrEmpty(method) || string.IsNullOrEmpty(k))
            return "";

        if (k != "s" && k != "m" && k != "h")
            return "";

        if (method == "inter-finger only")
            return "b" + k;       // bs / bm / bh

        if (method == "extension only")
            return "e" + k;       // es / em / eh

        if (method == "both")
            return "b" + k + "e" + k;   // bses / bmem / bheh

        Debug.LogWarning($"[FiveFingerGrasp] Unknown Method='{method}' or k='{k}' at row {rowIdx}");
        return "";
    }

    public void StartCurrentCsvTrial()
    {
        if (controlMode != ControlMode.PythonDriven)
        {
            Debug.LogWarning("[FiveFingerGrasp] CSV trial flow is intended for PythonDriven mode.");
            return;
        }

        if (!_trialLoaded) return;

        _currentPythonTrialCode = BuildTrialCodeFromCsvRow(currentTrialIndex);

        if (string.IsNullOrWhiteSpace(_currentPythonTrialCode))
        {
            Debug.LogWarning($"[FiveFingerGrasp] Trial {currentTrialIndex} has empty/invalid Method or k; visual-only trial.");
            _trialRunning = true;
            _hasStartedAnyTrial = true;
            _trialStartTime = Time.time;
            RefreshPlayButtonLabel();
            RefreshTrialButtonsAndNextState();
            return;
        }

        currentTrialCode = _currentPythonTrialCode;
        _trialStartTime = Time.time;

        StartCoroutine(StartTrialWithAutoInitRoutine());

        _trialRunning = true;
        _hasStartedAnyTrial = true;
        RefreshPlayButtonLabel();
        RefreshTrialButtonsAndNextState();
        RefreshTrialInfoLabel();
    }

    public void SnapBest()
    {
        if (!CanPickBest) return;

        _snapBest = CurrentStiffness();
        SetButtonCaptured(bestButton, true);

        Debug.Log($"[FiveFingerGrasp] BEST = {_snapBest.Value:0.###}");
        RefreshTrialButtonsAndNextState();
    }

    public void SnapUpper()
    {
        if (!CanPickUpper) return;

        _snapUpper = CurrentStiffness();
        SetButtonCaptured(upperButton, true);

        Debug.Log($"[FiveFingerGrasp] UPPER = {_snapUpper.Value:0.###}");

        if (!HasLower && HasBest)
            SetVisualKDirect(_snapBest.Value);

        RefreshTrialButtonsAndNextState();
    }

    public void SnapLower()
    {
        if (!CanPickLower) return;

        _snapLower = CurrentStiffness();
        SetButtonCaptured(lowerButton, true);

        Debug.Log($"[FiveFingerGrasp] LOWER = {_snapLower.Value:0.###}");

        if (!HasUpper && HasBest)
            SetVisualKDirect(_snapBest.Value);

        RefreshTrialButtonsAndNextState();
    }

    private void FinishAndAdvanceTrial()
    {
        if (!_trialLoaded) return;
        if (!CanGoNext) return;

        WriteBackTrialResult(currentTrialIndex);
        SaveCsv();

        float duration = Time.time - _trialStartTime;
        Debug.Log($"[FiveFingerGrasp] Trial {currentTrialIndex} done. duration={duration:0.###} sec");

        int nextIdx = currentTrialIndex + 1;
        if (nextIdx >= _rows.Count)
        {
            _trialRunning = false;
            if (nextGuardLabel) nextGuardLabel.text = "Experiment completed.";
            RefreshTrialButtonsAndNextState();
            Debug.Log("[FiveFingerGrasp] All trials completed.");
            return;
        }

        LoadTrial(nextIdx);
    }

    private void WriteBackTrialResult(int rowIdx)
    {
        if (rowIdx < 0 || rowIdx >= _rows.Count) return;

        EnsureColumn(colBest);
        EnsureColumn(colLower);
        EnsureColumn(colUpper);
        EnsureColumn(colDuration);

        var row = _rows[rowIdx];

        if (_snapBest.HasValue) row[_colIndex[colBest]] = _snapBest.Value.ToString("0.######", CultureInfo.InvariantCulture);
        if (_snapLower.HasValue) row[_colIndex[colLower]] = _snapLower.Value.ToString("0.######", CultureInfo.InvariantCulture);
        if (_snapUpper.HasValue) row[_colIndex[colUpper]] = _snapUpper.Value.ToString("0.######", CultureInfo.InvariantCulture);

        float duration = Mathf.Max(0f, Time.time - _trialStartTime);
        row[_colIndex[colDuration]] = duration.ToString("0.###", CultureInfo.InvariantCulture);
    }

    private void LoadCsvOrCreate()
    {
        string full = ResolvedCsvPath;

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
                colUserId, colTrialNo, colK, colMethod, colObjectType,
                colRep, colBlock, colBest, colLower, colUpper, colDuration
            };
            _rows = new List<string[]>();
            _colIndex = BuildIndex(_headers);
            File.WriteAllLines(full, new[] { JoinCsvLine(_headers.ToArray()) });
            Debug.Log($"[FiveFingerGrasp] CSV created: {full}");
            return;
        }

        var lines = File.ReadAllLines(full);
        if (lines.Length == 0)
        {
            _headers = new List<string>
            {
                colUserId, colTrialNo, colK, colMethod, colObjectType,
                colRep, colBlock, colBest, colLower, colUpper, colDuration
            };
            _rows = new List<string[]>();
            _colIndex = BuildIndex(_headers);
            File.WriteAllLines(full, new[] { JoinCsvLine(_headers.ToArray()) });
            return;
        }

        _headers = new List<string>(ParseCsvLine(lines[0]));
        EnsureColumn(colUserId);
        EnsureColumn(colTrialNo);
        EnsureColumn(colK);
        EnsureColumn(colMethod);
        EnsureColumn(colObjectType);
        EnsureColumn(colRep);
        EnsureColumn(colBlock);
        EnsureColumn(colBest);
        EnsureColumn(colLower);
        EnsureColumn(colUpper);
        EnsureColumn(colDuration);
        _colIndex = BuildIndex(_headers);

        _rows = new List<string[]>();
        for (int i = 1; i < lines.Length; i++)
        {
            var row = ParseCsvLine(lines[i]).ToArray();
            ArrayResizeOrPad(ref row, _headers.Count);
            _rows.Add(row);
        }

        Debug.Log($"[FiveFingerGrasp] CSV loaded: {full}, rows={_rows.Count}");
    }

    private void ApplyStartSelection()
    {
        if (!overrideStartUserAndTrial)
            return;

        if (!string.IsNullOrWhiteSpace(startUserId))
            userId = startUserId.Trim();

        startTrialNumber = Mathf.Max(1, startTrialNumber);
    }

    private int ResolveStartTrialIndex()
    {
        if (_rows == null || _rows.Count == 0)
            return 0;

        if (!overrideStartUserAndTrial)
            return Mathf.Clamp(currentTrialIndex, 0, _rows.Count - 1);

        if (!startFromCsvTrialNo)
        {
            int rowIndex = Mathf.Clamp(startTrialNumber - 1, 0, _rows.Count - 1);
            currentTrialIndex = rowIndex;
            return rowIndex;
        }

        for (int i = 0; i < _rows.Count; i++)
        {
            string trialNoStr = GetStringFor(i, colTrialNo).Trim();
            if (int.TryParse(trialNoStr, out int trialNo) && trialNo == startTrialNumber)
            {
                currentTrialIndex = i;
                return i;
            }
        }

        Debug.LogWarning($"[FiveFingerGrasp] Trial_no={startTrialNumber} not found for user {userId}. Fallback to first row.");
        currentTrialIndex = 0;
        return 0;
    }

    private void SaveCsv()
    {
        string full = ResolvedCsvPath;
        var outLines = new List<string>();
        outLines.Add(JoinCsvLine(_headers.ToArray()));

        foreach (var r in _rows)
        {
            var row = r;
            if (row.Length != _headers.Count)
                ArrayResizeOrPad(ref row, _headers.Count);
            outLines.Add(JoinCsvLine(row));
        }

        File.WriteAllLines(full, outLines.ToArray());
        Debug.Log($"[FiveFingerGrasp] CSV saved: {full}");
    }

    private float GetValueFor(int rowIdx, string colName)
    {
        if (rowIdx < 0 || rowIdx >= _rows.Count) return 0f;
        if (!_colIndex.TryGetValue(colName, out int c)) return 0f;

        string s = _rows[rowIdx][c]?.Trim();
        if (string.IsNullOrEmpty(s)) return 0f;

        if (float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v)) return v;
        if (float.TryParse(s, out v)) return v;
        return 0f;
    }

    private string GetStringFor(int rowIdx, string colName)
    {
        if (rowIdx < 0 || rowIdx >= _rows.Count) return "";
        if (!_colIndex.TryGetValue(colName, out int c)) return "";
        return _rows[rowIdx][c] ?? "";
    }

    private void EnsureColumn(string name)
    {
        int idx = _headers.FindIndex(h => string.Equals(h, name, StringComparison.OrdinalIgnoreCase));
        if (idx >= 0)
        {
            _colIndex = BuildIndex(_headers);
            return;
        }

        _headers.Add(name);
        for (int i = 0; i < _rows.Count; i++)
        {
            var row = _rows[i];
            ArrayResizeOrPad(ref row, _headers.Count);
            _rows[i] = row;
        }

        _colIndex = BuildIndex(_headers);
    }

    private static Dictionary<string, int> BuildIndex(List<string> headers)
    {
        var map = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        for (int i = 0; i < headers.Count; i++)
            map[headers[i]] = i;
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
                if (ch == '"')
                {
                    if (i + 1 < line.Length && line[i + 1] == '"')
                    {
                        sb.Append('"');
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
                else if (ch == '"')
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
        var outList = new List<string>(cells.Length);
        foreach (var s in cells)
        {
            string t = s ?? "";
            bool needQuote = t.Contains(",") || t.Contains("\"") || t.Contains("\n") || t.Contains("\r");
            if (needQuote)
                t = "\"" + t.Replace("\"", "\"\"") + "\"";
            outList.Add(t);
        }
        return string.Join(",", outList);
    }

    private void RefreshTrialInfoLabel()
    {
        if (trialInfoLabel == null) return;

        string trialNo = GetStringFor(currentTrialIndex, colTrialNo);
        string method = GetStringFor(currentTrialIndex, colMethod);
        string obj = GetStringFor(currentTrialIndex, colObjectType);
        string rep = GetStringFor(currentTrialIndex, colRep);
        string block = GetStringFor(currentTrialIndex, colBlock);
        string k = GetStringFor(currentTrialIndex, colK);
        string code = BuildTrialCodeFromCsvRow(currentTrialIndex);

        trialInfoLabel.text =
            $"Trial {currentTrialIndex + 1}/{_rows.Count}\n" +
            $"Trial_no={trialNo} | Block={block} | Method={method} | Obj={obj} | Rep={rep}\n" +
            $"k={k} | cmd={code}";
    }

    private void BuildHotkeyHelpText()
    {
        if (!hotkeyHelpLabel) return;

        hotkeyHelpLabel.text =
            "W / ↑ : +" + stepSmall + " u/s\n" +
            "S / ↓ : -" + stepSmall + " u/s\n" +
            "E : +" + stepLarge + " u/s   D : -" + stepLarge + " u/s\n" +
            "Z: lower   X: best   C: upper\n" +
            "Enter: Start & Next";
    }

    private void RefreshTrialButtonsAndNextState()
    {
        SetButtonState(bestButton, CanPickBest, HasBest, "Best");
        SetButtonState(upperButton, CanPickUpper, HasUpper, "Upper");
        SetButtonState(lowerButton, CanPickLower, HasLower, "Lower");

        if (playButton)
            playButton.interactable = !_trialLoaded ? false : (!_trialRunning || CanGoNext);

        if (nextGuardLabel)
        {
            if (!_trialRunning)
                nextGuardLabel.text = "";
            else if (CanGoNext)
                nextGuardLabel.text = "";
            else if (!HasBest)
                nextGuardLabel.text = "Select Best first.";
            else if (!HasUpper || !HasLower)
                nextGuardLabel.text = "Select the remaining Upper / Lower.";
            else
                nextGuardLabel.text = "";
        }
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
        cb.disabledColor = bg;
        cb.colorMultiplier = 1f;
        btn.colors = cb;
    }

    private void SetButtonCaptured(Button btn, bool captured)
    {
        if (!btn) return;

        var img = btn.GetComponent<Image>();
        Color c = captured ? snapDoneColor : snapIdleColor;
        if (img) img.color = c;

        var cb = btn.colors;
        cb.normalColor = c;
        cb.highlightedColor = c;
        cb.pressedColor = c * 0.9f;
        cb.selectedColor = c;
        cb.disabledColor = c;
        cb.colorMultiplier = 1f;
        btn.colors = cb;

        var txt = btn.GetComponentInChildren<TextMeshProUGUI>(true);
        if (txt) txt.enabled = true;
    }

    private void SetButtonState(Button btn, bool interactable, bool captured, string label)
    {
        if (!btn) return;

        btn.interactable = interactable;

        var img = btn.GetComponent<Image>();
        var txt = btn.GetComponentInChildren<TextMeshProUGUI>(true);

        Color colorToUse;
        if (captured)
            colorToUse = snapDoneColor;
        else if (interactable)
            colorToUse = snapIdleColor;
        else
            colorToUse = disabledOverlayColor;

        if (img) img.color = colorToUse;

        if (txt)
        {
            txt.text = label;
            txt.color = snapTextColor;
            txt.enabled = true;
        }

        var cb = btn.colors;
        cb.normalColor = colorToUse;
        cb.highlightedColor = colorToUse;
        cb.pressedColor = colorToUse * 0.9f;
        cb.selectedColor = colorToUse;
        cb.disabledColor = colorToUse;
        cb.colorMultiplier = 1f;
        btn.colors = cb;
    }

    private void SetVisualKDirect(float targetK)
    {
        if (!deltaSlider) return;

        float delta = Mathf.Clamp(targetK, minStiffness, maxStiffness) - initialStiffness;
        deltaSlider.value = Mathf.Clamp(delta, deltaSlider.minValue, deltaSlider.maxValue);
        RefreshLabel();
    }

    private void ClampSlider()
    {
        if (!deltaSlider) return;
        float minDelta = minStiffness - initialStiffness;
        float maxDelta = maxStiffness - initialStiffness;
        deltaSlider.value = Mathf.Clamp(deltaSlider.value, minDelta, maxDelta);
    }

    private void RefreshLabel()
    {
        if (stiffnessLabel) stiffnessLabel.text = $"Stiffness (k): {CurrentStiffness():0.##}";
    }

    private void SetAdjustControls(bool on)
    {
        if (deltaSlider) deltaSlider.interactable = on;
        if (plus5Button) plus5Button.interactable = on;
        if (minus5Button) minus5Button.interactable = on;
    }

    private void SetAllControls(bool on)
    {
        if (playButton) playButton.interactable = on;
        SetAdjustControls(on && adjustmentsUnlocked);
    }

#if UNITY_EDITOR
    [ContextMenu("Auto-Place 5 Contacts (rough)")]
    private void AutoPlaceContacts()
    {
        if (!ballMeshFilter) return;
        var tr = ballMeshFilter.transform;
        if (contactPoints == null || contactPoints.Length != 5) contactPoints = new Transform[5];

        string[] names = { "ThumbCP", "IndexCP", "MiddleCP", "RingCP", "LittleCP" };
        Vector3[] locals = {
            new Vector3(-0.25f,  0.05f,  0.10f),
            new Vector3( 0.22f,  0.10f,  0.18f),
            new Vector3( 0.18f, -0.02f,  0.22f),
            new Vector3( 0.12f, -0.10f,  0.18f),
            new Vector3( 0.05f, -0.15f,  0.12f)
        };

        for (int i = 0; i < 5; i++)
        {
            if (contactPoints[i] == null)
            {
                var go = new GameObject(names[i]);
                go.transform.SetParent(tr, false);
                contactPoints[i] = go.transform;
            }

            contactPoints[i].localPosition = locals[i];
        }

        Debug.Log("Placed 5 contact points under Ball.");
    }
#endif
}
