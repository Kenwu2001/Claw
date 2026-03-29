using UnityEngine;

public class BallCollisionProbe : MonoBehaviour
{
    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log("[Probe] OnCollisionEnter -> " + collision.gameObject.name);
    }

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log("[Probe] OnTriggerEnter -> " + other.gameObject.name);
    }
}