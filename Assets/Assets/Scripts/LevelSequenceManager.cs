using UnityEngine;

public class LevelSequenceManager : MonoBehaviour
{
    [Header("Level Targets (1-based order)")]
    [Tooltip("依序填入每一關要顯示的互動物件，例如：Element 0 = Ball, Element 1 = Cat")]
    public GameObject[] levelTargets;

    [Header("Start")]
    [Tooltip("進入 Unity Play Mode 時從第幾關開始（1-based）")]
    public int startLevel = 1;

    [Header("Keyboard Shortcuts")]
    public bool enableKeyboardShortcuts = true;

    [Tooltip("按下此鍵，從第一關重新開始")]
    public KeyCode restartLevel1Key = KeyCode.Alpha1;

    [Tooltip("按下此鍵，直接從第二關開始")]
    public KeyCode startLevel2Key = KeyCode.Alpha2;

    [Tooltip("按下此鍵，直接從第三關開始")]
    public KeyCode startLevel3Key = KeyCode.Alpha3;

    [Tooltip("按下此鍵，直接從第四關開始")]
    public KeyCode startLevel4Key = KeyCode.Alpha4;

    [Header("End Behavior")]
    [Tooltip("最後一關完成後是否循環回第一關")]
    public bool loopAfterLastLevel = false;

    public int CurrentLevelIndex { get; private set; } = -1; // 0-based

    private void Start()
    {
        if (levelTargets == null || levelTargets.Length == 0)
        {
            Debug.LogWarning("[LevelSequence] levelTargets is empty.");
            return;
        }

        StartFromLevel(startLevel);
    }

    private void Update()
    {
        if (!enableKeyboardShortcuts) return;

        if (Input.GetKeyDown(restartLevel1Key) || Input.GetKeyDown(KeyCode.Keypad1))
        {
            StartFromLevel(1);
        }

        if (Input.GetKeyDown(startLevel2Key) || Input.GetKeyDown(KeyCode.Keypad2))
        {
            StartFromLevel(2);
        }

        if (Input.GetKeyDown(startLevel3Key) || Input.GetKeyDown(KeyCode.Keypad3))
        {
            StartFromLevel(3);
        }

        if (Input.GetKeyDown(startLevel4Key) || Input.GetKeyDown(KeyCode.Keypad4))
        {
            StartFromLevel(4);
        }
    }

    public void StartFromLevel(int levelNumber)
    {
        if (levelTargets == null || levelTargets.Length == 0)
        {
            Debug.LogWarning("[LevelSequence] Cannot start level: levelTargets is empty.");
            return;
        }

        int targetIndex = Mathf.Clamp(levelNumber - 1, 0, levelTargets.Length - 1);
        SetActiveLevelByIndex(targetIndex);
    }

    public void AdvanceToNextLevel()
    {
        if (levelTargets == null || levelTargets.Length == 0)
        {
            Debug.LogWarning("[LevelSequence] Cannot advance level: levelTargets is empty.");
            return;
        }

        if (CurrentLevelIndex < 0)
        {
            SetActiveLevelByIndex(0);
            return;
        }

        int nextIndex = CurrentLevelIndex + 1;
        if (nextIndex >= levelTargets.Length)
        {
            if (!loopAfterLastLevel)
            {
                Debug.Log("[LevelSequence] Already at last level.");
                return;
            }

            nextIndex = 0;
        }

        SetActiveLevelByIndex(nextIndex);
    }

    private void SetActiveLevelByIndex(int levelIndex)
    {
        CurrentLevelIndex = levelIndex;

        for (int i = 0; i < levelTargets.Length; i++)
        {
            if (levelTargets[i] == null) continue;
            levelTargets[i].SetActive(i == levelIndex);
        }

        GameObject activeTarget = levelTargets[levelIndex];
        if (activeTarget != null)
        {
            BallFlyController ballFly = activeTarget.GetComponentInChildren<BallFlyController>(true);
            if (ballFly != null)
            {
                ballFly.RestartFromLevelStart();
            }
        }

        Debug.Log($"[LevelSequence] Active Level: {CurrentLevelIndex + 1}");
    }
}
