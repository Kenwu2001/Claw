using UnityEngine;

/**
 * Left-controller input mapper for BallFlyController.
 * - Stick Up/Down: continuous ±stepSmall (per second, scaled by axis)
 * - X: Lowerbound
 * - Y: Upperbound
 * - X+Y within combo window: Best
 * - LThumbstick Click: Play
 * - (IndexTrigger + HandTrigger) together: Next
 * 
 * Notes:
 * - Only LEFT controller is read.
 * - If restPanel is assigned and active, inputs are ignored except "Next" (so你可以用手把繼續下一個 block)。
 */
public class OVRLeftControllerInputForBallFly : MonoBehaviour
{
    [Header("Target Controller")]
    public BallFlyController controller;

    [Header("Optional Rest Panel (to gate inputs)")]
    public GameObject restPanel;

    [Header("Nudge (match BallFlyController's steps)")]
    public float stepSmall = 1f;              // 對應 controller.Nudge 的基本單位（百分比）
    public float stickDeadzone = 0.2f;        // 搖桿死區
    public float repeatPerSecondAtFullTilt = 8f; // 搖桿推到底時，每秒觸發幾個 stepSmall

    [Header("Combo")]
    public float comboWindowSeconds = 0.20f;  // X 與 Y 的組合鍵容忍時間

    [Header("Triggers → Next")]
    public float triggerPressThreshold = 0.75f;

    // 内部狀態
    private bool _bothTriggersDownLast;
    private bool _xPending, _yPending;
    private float _comboTimer;

    private static readonly OVRInput.Controller L = OVRInput.Controller.LTouch;

    private bool IsResting =>
        restPanel != null && restPanel.activeInHierarchy;

    void Update()
    {
        if (controller == null) return;

        // 休息畫面：只允許 Next（兩扳機同時）
        if (IsResting)
        {
            HandleNextByTriggers();
            return;
        }

        // 1) 搖桿連續微調（只有調整階段才會起作用，控制權交給 controller 自己判斷也 OK）
        var stick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, L);
        float y = stick.y;
        if (Mathf.Abs(y) > stickDeadzone)
        {
            float stepsPerSec = repeatPerSecondAtFullTilt * Mathf.InverseLerp(stickDeadzone, 1f, Mathf.Abs(y));
            float delta = Mathf.Sign(y) * stepSmall * stepsPerSec * Time.deltaTime;
            controller.Nudge(delta);
        }

        // 2) X / Y / X+Y（組合鍵）
        bool xDown = OVRInput.GetDown(OVRInput.Button.Three, L); // X on left
        bool yDown = OVRInput.GetDown(OVRInput.Button.Four,  L); // Y on left

        if (xDown)
        {
            _xPending = true;
            _comboTimer = comboWindowSeconds;
        }
        if (yDown)
        {
            _yPending = true;
            _comboTimer = comboWindowSeconds;
        }

        if (_xPending && _yPending)
        {
            // 組合成立 → Best
            controller.SnapBest();
            _xPending = _yPending = false;
            _comboTimer = 0f;
            // 將按鈕顏色/狀態交由 controller 自己處理
        }
        else if (_comboTimer > 0f)
        {
            _comboTimer -= Time.deltaTime;
            if (_comboTimer <= 0f)
            {
                // 時間到：若只按了一個，落單者各自觸發
                if (_xPending && !_yPending) controller.SnapLower();
                if (_yPending && !_xPending) controller.SnapUpper();
                _xPending = _yPending = false;
            }
        }

        // 3) 搖桿按下：Play
        if (OVRInput.GetDown(OVRInput.Button.PrimaryThumbstick, L))
        {
            controller.PlayOnce();
        }

        // 4) 兩扳機同時：Next
        HandleNextByTriggers();
    }

    private void HandleNextByTriggers()
    {
        float idx  = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, L);
        float grip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger,  L);
        bool both  = (idx >= triggerPressThreshold) && (grip >= triggerPressThreshold);

        if (both && !_bothTriggersDownLast)
        {
            controller.TryNext(); // 走防護檢查（沒填滿就不會進）
        }
        _bothTriggersDownLast = both;
    }
}
