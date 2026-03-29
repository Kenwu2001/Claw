using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 靠近端點時，用「假手」取代「真手可見模型」；離開還原。只切顯示與互動，不關閉追蹤根。
/// 先用於右手（Hand Tracking，無控制器）。
/// </summary>
public class HandTakeoverSwitch : MonoBehaviour
{
    [Header("Scene References")]
    [Tooltip("追蹤根（保持啟用），例如 RightHand 的根，用來量距離")]
    public Transform trackedHandRoot;

    [Tooltip("真手『可見模型根』（會被隱藏），通常是 RightHand/Visuals 或含 SkinnedMeshRenderer 的物件")]
    public GameObject trackedHandVisualRoot;

    [Tooltip("事先擺好固定接球姿勢的『假手』（初始請關閉 SetActive(false)）")]
    public GameObject fakeHandRoot;

    [Tooltip("端點中心：手靠近這裡進入接管模式")]
    public Transform takeoverCenter;

    [Header("Settings")]
    [Tooltip("觸發半徑(公尺)")]
    public float takeoverRadius = 0.18f;

    [Tooltip("切到假手時，要停用的互動腳本（抓取/Ray/Poke等）")]
    public List<MonoBehaviour> handInteractorsToDisable = new();

    [Header("Debug")]
    [Tooltip("允許鍵盤 Shift 在 Editor 手動切換")]
    public bool allowShiftToggle = true;

    public bool IsInTakeover { get; private set; } = false;

    private Renderer[] _trackedRenderers;

    void Awake()
    {
        if (!trackedHandRoot) Debug.LogWarning("[HandTakeoverSwitch] trackedHandRoot 未指定。");
        if (!trackedHandVisualRoot) Debug.LogWarning("[HandTakeoverSwitch] trackedHandVisualRoot 未指定。");
        if (!fakeHandRoot) Debug.LogWarning("[HandTakeoverSwitch] fakeHandRoot 未指定。");
        if (!takeoverCenter) Debug.LogWarning("[HandTakeoverSwitch] takeoverCenter 未指定。");

        _trackedRenderers = trackedHandVisualRoot
            ? trackedHandVisualRoot.GetComponentsInChildren<Renderer>(true)
            : new Renderer[0];

        SetTakeover(false, force:true); // 初始：真手可見、假手關閉
    }

    void Update()
    {
        // Editor 偵錯：Shift 手動切換
        if (allowShiftToggle && (Input.GetKeyDown(KeyCode.LeftShift) || Input.GetKeyDown(KeyCode.RightShift)))
        {
            SetTakeover(!IsInTakeover);
            return;
        }

        // 近距離自動切換
        if (trackedHandRoot && takeoverCenter)
        {
            float d = Vector3.Distance(trackedHandRoot.position, takeoverCenter.position);
            bool shouldTakeover = d <= takeoverRadius;
            if (shouldTakeover != IsInTakeover)
            {
                SetTakeover(shouldTakeover);
            }
        }
    }

    private void SetTakeover(bool on, bool force=false)
    {
        if (!force && on == IsInTakeover) return;
        IsInTakeover = on;

        if (fakeHandRoot) fakeHandRoot.SetActive(on);

        if (_trackedRenderers != null)
            foreach (var r in _trackedRenderers) if (r) r.enabled = !on;

        if (handInteractorsToDisable != null)
            foreach (var m in handInteractorsToDisable) if (m) m.enabled = !on;
    }

    void OnDrawGizmosSelected()
    {
        if (!takeoverCenter) return;
        Gizmos.color = new Color(0f, 0.6f, 1f, 0.25f);
        Gizmos.DrawSphere(takeoverCenter.position, takeoverRadius);
        Gizmos.color = new Color(0f, 0.6f, 1f, 1f);
        Gizmos.DrawWireSphere(takeoverCenter.position, takeoverRadius);
    }
}
