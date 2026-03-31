using UnityEngine;

public static class BaselineRequirementState
{
    public static bool IsReady { get; private set; }
    public static bool IsRecording { get; private set; }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void ResetOnPlayStart()
    {
        IsReady = false;
        IsRecording = false;
    }

    public static bool TryBeginRecording()
    {
        if (IsReady || IsRecording)
            return false;

        IsRecording = true;
        return true;
    }

    public static void CompleteRecording(bool success)
    {
        IsRecording = false;
        if (success)
            IsReady = true;
    }

    public static void Reset()
    {
        IsReady = false;
        IsRecording = false;
    }
}