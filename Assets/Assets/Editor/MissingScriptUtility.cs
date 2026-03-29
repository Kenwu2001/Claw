#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

public static class MissingScriptUtility
{
    [MenuItem("Tools/Find Missing Scripts In Scene")]
    static void FindMissing()
    {
        int goCount = 0, compCount = 0, missingCount = 0;
        foreach (var go in GameObject.FindObjectsOfType<GameObject>(true))
        {
            goCount++;
            var comps = go.GetComponents<Component>();
            foreach (var c in comps)
            {
                compCount++;
                if (c == null)
                {
                    missingCount++;
                    Debug.LogWarning($"[Missing Script] {GetPath(go)}", go);
                }
            }
        }
        Debug.Log($"[Missing Script] Scanned {goCount} GOs, {compCount} comps, {missingCount} missing.");
    }

    [MenuItem("Tools/Remove Missing Scripts From Selection")]
    static void RemoveFromSelection()
    {
        foreach (var go in Selection.gameObjects)
            GameObjectUtility.RemoveMonoBehavioursWithMissingScript(go);
        Debug.Log("[Missing Script] Removed from selection & children.");
    }

    static string GetPath(GameObject go)
    {
        string path = go.name;
        var t = go.transform;
        while (t.parent != null) { t = t.parent; path = t.name + "/" + path; }
        return path;
    }
}
#endif
