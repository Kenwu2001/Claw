using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandPoseBlender : MonoBehaviour
{
    [Header("Roots")]
    public Transform liveHandRoot;
    public Transform impactPoseRoot;

    [Header("Blend Timing")]
    [Tooltip("撞擊後多久內滑到 impact pose")]
    public float blendToImpactDuration = 0.08f;

    [Header("Pauses (seconds)")]
    public float holdAtCollision = 1f;

    [Tooltip("停在 impact pose 多久")]
    public float holdDuration = 0.08f;

    [Tooltip("是否自動回到原本 live pose")]
    public bool autoReturn = false;

    [Tooltip("回到原本 pose 的時間")]
    public float returnDuration = 0.12f;

    [Header("Debug")]
    public bool logMapping = false;

    private class BonePair
    {
        public Transform live;
        public Transform impact;
        public Vector3 initialLocalPos;
        public Quaternion initialLocalRot;
    }

    private readonly List<BonePair> _pairs = new List<BonePair>();
    private Coroutine _blendCo;

    void Awake()
    {
        BuildBoneMapping();
        CacheInitialPose();
    }

    [ContextMenu("Rebuild Bone Mapping")]
    public void BuildBoneMapping()
    {
        _pairs.Clear();

        if (liveHandRoot == null || impactPoseRoot == null)
        {
            Debug.LogError("[HandPoseBlender] liveHandRoot / impactPoseRoot 未指定");
            return;
        }

        Dictionary<string, Transform> impactMap = new Dictionary<string, Transform>();
        Transform[] impactBones = impactPoseRoot.GetComponentsInChildren<Transform>(true);
        foreach (var t in impactBones)
        {
            string path = GetRelativePath(impactPoseRoot, t);
            if (!impactMap.ContainsKey(path))
                impactMap.Add(path, t);
        }

        Transform[] liveBones = liveHandRoot.GetComponentsInChildren<Transform>(true);
        int matched = 0;

        foreach (var live in liveBones)
        {
            string path = GetRelativePath(liveHandRoot, live);

            if (impactMap.TryGetValue(path, out Transform impact))
            {
                _pairs.Add(new BonePair
                {
                    live = live,
                    impact = impact,
                    initialLocalPos = live.localPosition,
                    initialLocalRot = live.localRotation
                });

                matched++;

                if (logMapping)
                    Debug.Log($"[HandPoseBlender] Matched: {path}");
            }
        }

        Debug.Log($"[HandPoseBlender] Bone mapping built. Matched = {matched}");
    }

    [ContextMenu("Cache Initial Pose")]
    public void CacheInitialPose()
    {
        if (_pairs.Count == 0)
        {
            Debug.LogError("[HandPoseBlender] 尚未 Cache Initial Pose");
            return;
        }

        foreach (var p in _pairs)
        {
            if (p.live == null) continue;
            p.initialLocalPos = p.live.localPosition;
            p.initialLocalRot = p.live.localRotation;
        }

        Debug.Log("[HandPoseBlender] Initial pose cached.");
    }

    public void PlayImpactPose()
    {        
        if (!isActiveAndEnabled)
            return;

        if (_pairs.Count == 0)
        {
            Debug.LogError("[HandPoseBlender] No bone mapping. Please assign roots correctly.");
            return;
        }

        if (_blendCo != null)
            StopCoroutine(_blendCo);

        _blendCo = StartCoroutine(PlayImpactPoseCo());
    }

    private IEnumerator PlayImpactPoseSequence()
    {
        if (holdAtCollision > 0f)
            yield return new WaitForSeconds(holdAtCollision);

        yield return PlayImpactPoseCo();

        _blendCo = null;
    }

    public void ResetToInitialImmediately()
    {
        if (_blendCo != null)
        {
            StopCoroutine(_blendCo);
            _blendCo = null;
        }

        foreach (var p in _pairs)
        {
            if (p.live == null) continue;
            p.live.localPosition = p.initialLocalPos;
            p.live.localRotation = p.initialLocalRot;
        }
    }

    private IEnumerator PlayImpactPoseCo()
    {
        // 為了避免多次觸發時「回彈基準點」怪掉，
        // 每次播放前都先記目前 live pose，這樣可以從當下姿勢平滑進 impact。
        List<Vector3> startPos = new List<Vector3>(_pairs.Count);
        List<Quaternion> startRot = new List<Quaternion>(_pairs.Count);

        foreach (var p in _pairs)
        {
            startPos.Add(p.live.localPosition);
            startRot.Add(p.live.localRotation);
        }

        // Step 1: blend 到 impact pose
        float t = 0f;
        float dur = Mathf.Max(0.0001f, blendToImpactDuration);

        while (t < 1f)
        {
            t += Time.deltaTime / dur;
            float w = Mathf.Clamp01(t);

            for (int i = 0; i < _pairs.Count; i++)
            {
                var p = _pairs[i];
                if (p.live == null || p.impact == null) continue;

                p.live.localPosition = Vector3.Lerp(startPos[i], p.impact.localPosition, w);
                p.live.localRotation = Quaternion.Slerp(startRot[i], p.impact.localRotation, w);
            }

            yield return null;
        }

        // 最後強制貼齊 impact pose
        foreach (var p in _pairs)
        {
            if (p.live == null || p.impact == null) continue;
            p.live.localPosition = p.impact.localPosition;
            p.live.localRotation = p.impact.localRotation;
        }

        if (holdDuration > 0f)
            yield return new WaitForSeconds(holdDuration);

        // Step 2: 自動回原 pose
        if (autoReturn)
        {
            List<Vector3> impactPos = new List<Vector3>(_pairs.Count);
            List<Quaternion> impactRot = new List<Quaternion>(_pairs.Count);

            foreach (var p in _pairs)
            {
                impactPos.Add(p.live.localPosition);
                impactRot.Add(p.live.localRotation);
            }

            t = 0f;
            dur = Mathf.Max(0.0001f, returnDuration);

            while (t < 1f)
            {
                t += Time.deltaTime / dur;
                float w = Mathf.Clamp01(t);

                for (int i = 0; i < _pairs.Count; i++)
                {
                    var p = _pairs[i];
                    if (p.live == null) continue;

                    p.live.localPosition = Vector3.Lerp(impactPos[i], p.initialLocalPos, w);
                    p.live.localRotation = Quaternion.Slerp(impactRot[i], p.initialLocalRot, w);
                }

                yield return null;
            }

            foreach (var p in _pairs)
            {
                if (p.live == null) continue;
                p.live.localPosition = p.initialLocalPos;
                p.live.localRotation = p.initialLocalRot;
            }
        }

        _blendCo = null;
    }

    private string GetRelativePath(Transform root, Transform target)
    {
        if (target == root) return "";

        List<string> names = new List<string>();
        Transform cur = target;

        while (cur != null && cur != root)
        {
            names.Add(cur.name);
            cur = cur.parent;
        }

        names.Reverse();
        return string.Join("/", names);
    }
}