using UnityEngine;

/// 左手 Touch 控制器輸入 → 控制 SizePlayController
/// 使用 OVRInput（Meta All-in-One SDK）
/// 只讀「左手」按鍵與搖桿
[DefaultExecutionOrder(200)]
public class OVRLeftControllerInputForSizePlay : MonoBehaviour
{
    [Header("Target")]
    public SizePlayController controller;

    [Header("Thumbstick Repeat")]
    [Tooltip("左搖桿上下判定死區")]
    [Range(0.2f, 0.95f)]
    public float stickDeadzone = 0.5f;

    [Tooltip("首次連發延遲(秒)")]
    public float firstDelay = 0.25f;

    [Tooltip("連發間隔(秒)")]
    public float repeatInterval = 0.08f;

    [Header("Trigger Thresholds")]
    [Tooltip("IndexTrigger 視為按下的臨界值(0~1)")]
    public float indexThreshold = 0.55f;

    [Tooltip("Hand(GRIP) 視為按下的臨界值(0~1)")]
    public float handThreshold  = 0.55f;

    [Header("Debug")]
    public bool logEvents = false;

    // 內部狀態
    private int   _stickDir = 0;        // -1 / 0 / +1
    private float _nextRepeatTime = 0f; // 連發時間點
    private float _prevIndex = 0f, _prevHand = 0f;

    void Reset() => controller = FindObjectOfType<SizePlayController>();

    void Update()
    {
        if (!controller || !controller.enabled) return;

        // 若休息面板開著 → 禁用所有輸入（和鍵盤邏輯一致）
        if (controller.restPanel && controller.restPanel.activeSelf)
        {
            _stickDir = 0;
            return;
        }

        // ─── Play（搖桿按下） ───
        if (OVRInput.GetDown(OVRInput.RawButton.LThumbstick))
        {
            if (logEvents) Debug.Log("[OVRLeft] Play (LThumbstick click)");
            controller.PlayOnce();
        }

        // ─── Next（Index + Hand 同時過門檻） ───
        float idx = OVRInput.Get(OVRInput.RawAxis1D.LIndexTrigger);
        float hnd = OVRInput.Get(OVRInput.RawAxis1D.LHandTrigger);

        bool comboNow = (idx >= indexThreshold) && (hnd >= handThreshold);
        bool justCrossed = (_prevIndex < indexThreshold && idx >= indexThreshold) ||
                           (_prevHand  < handThreshold  && hnd >= handThreshold);

        if (comboNow && justCrossed)
        {
            if (logEvents) Debug.Log("[OVRLeft] Next (Index+Hand)");
            controller.TryNext();
        }
        _prevIndex = idx;
        _prevHand  = hnd;

        // 下面這些只在「調整階段」啟用（和鍵盤版一致）
        if (!(controller && controller.enabled && IsAdjustPhase())) return;

        // ─── Snap（X = Lower、Y = Upper、X+Y = Best） ───
        bool xNow  = OVRInput.Get(OVRInput.RawButton.X);
        bool yNow  = OVRInput.Get(OVRInput.RawButton.Y);
        bool xDown = OVRInput.GetDown(OVRInput.RawButton.X);
        bool yDown = OVRInput.GetDown(OVRInput.RawButton.Y);

        // 同時：Best（優先）
        if ( (xDown && yNow) || (yDown && xNow) )
        {
            if (logEvents) Debug.Log("[OVRLeft] Snap Best (X+Y)");
            controller.SnapBest();
        }
        else
        {
            if (xDown)
            {
                if (logEvents) Debug.Log("[OVRLeft] Snap Lower (X)");
                controller.SnapLower();
            }
            if (yDown)
            {
                if (logEvents) Debug.Log("[OVRLeft] Snap Upper (Y)");
                controller.SnapUpper();
            }
        }

        // ─── Nudge（左搖桿上/下，長押連發） ───
        Vector2 axis = OVRInput.Get(OVRInput.RawAxis2D.LThumbstick);
        int dir = 0;
        if      (axis.y >  stickDeadzone) dir = +1;
        else if (axis.y < -stickDeadzone) dir = -1;

        if (dir == 0)
        {
            _stickDir = 0; // 放開：歸零
        }
        else
        {
            if (_stickDir != dir)
            {
                // 方向剛改變 → 立刻觸發一次 & 設定首次延遲
                controller.Nudge(controller.stepSmall * dir);
                _stickDir = dir;
                _nextRepeatTime = Time.time + firstDelay;
                if (logEvents) Debug.Log($"[OVRLeft] Nudge {(dir>0?"+":"-")}stepSmall");
            }
            else if (Time.time >= _nextRepeatTime)
            {
                controller.Nudge(controller.stepSmall * dir);
                _nextRepeatTime = Time.time + repeatInterval;
            }
        }
    }

    private bool IsAdjustPhase()
    {
        // 調整階段：動畫播完、休息面板關閉
        bool inRest = controller.restPanel && controller.restPanel.activeSelf;
        return controller.enabled && !inRest && GetAdjustUnlocked();
    }

    private bool GetAdjustUnlocked()
    {
        // 利用 Next 按鈕是否可互動來當 proxy（也可以在 SizePlayController 暴露一個 public 只讀屬性）
        // 若你不想依賴 UI，可在 SizePlayController 暴露 public bool AdjustmentsUnlocked { get; }
        // 這裡用 try/catch 保守處理
        try
        {
            // 在你那份邏輯：進入調整階段時，三個 snap 按鈕會變成 interactable
            return (controller.lowerButton && controller.lowerButton.interactable) ||
                   (controller.bestButton  && controller.bestButton.interactable)  ||
                   (controller.upperButton && controller.upperButton.interactable);
        }
        catch { return false; }
    }
}
