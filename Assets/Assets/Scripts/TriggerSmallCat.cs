using UnityEngine;

public class TriggerSmallCat : MonoBehaviour
{
	[Header("Hand Detection")]
	public Transform handRoot;
	public string handTag = "Hand";

	[Header("Cat Motion")]
	[Tooltip("要移動的小貓；若空白就移動這個物件")]
	public Transform catRoot;

	[Tooltip("X 方向來回的半徑，0.05 = 5 公分")]
	public float moveDistance = 0.05f;

	[Tooltip("來回速度（循環頻率）")]
	public float moveSpeed = 1f;

	private bool _isMoving;
	private float _movePhase;
	private Vector3 _startLocalPosition;

	private void Awake()
	{
		if (catRoot == null)
			catRoot = transform;

		_startLocalPosition = catRoot.localPosition;
	}

	private void Update()
	{
		if (!_isMoving || catRoot == null)
			return;

		_movePhase += Time.deltaTime * Mathf.Max(0f, moveSpeed) * Mathf.PI * 2f;
		float offset = Mathf.Sin(_movePhase) * Mathf.Max(0f, moveDistance);
		catRoot.localPosition = _startLocalPosition + Vector3.right * offset;
	}

	private bool IsHandHit(Transform other)
	{
		if (other == null) return false;

		if (handRoot != null)
			return other == handRoot || other.IsChildOf(handRoot);

		return other.CompareTag(handTag);
	}

	private void OnTriggerEnter(Collider other)
	{
		if (!IsHandHit(other.transform)) return;

		Debug.Log("[TriggerSmallCat] Hit hand (trigger): " + other.name);
		_isMoving = true;
	}

	private void OnTriggerExit(Collider other)
	{
		if (!IsHandHit(other.transform)) return;

		_isMoving = false;
	}
}
