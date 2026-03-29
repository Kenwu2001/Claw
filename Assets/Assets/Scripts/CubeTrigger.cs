using UnityEngine;

public class CubeTrigger : MonoBehaviour
{
    [Header("Hand Detection")]
    public Transform handRoot;
    public string handTag = "Hand";
    public bool requireHandHit = true;

    [Header("Level Sequence")]
    public LevelSequenceManager levelSequenceManager;
    public bool advanceLevelOnHit = true;

    private bool IsHandHit(Transform other)
    {
        if (other == null) return false;

        if (handRoot != null)
            return other == handRoot || other.IsChildOf(handRoot);

        return other.CompareTag(handTag);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (requireHandHit && !IsHandHit(other.transform)) return;
        Debug.Log("!!!!!!!!!![CubeTrigger] Trigger hit: " + other.name);

        if (advanceLevelOnHit && levelSequenceManager != null)
        {
            levelSequenceManager.AdvanceToNextLevel();
        }
    }
}
