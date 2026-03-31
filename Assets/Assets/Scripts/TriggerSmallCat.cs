using UnityEngine;
using System.Collections;

public class TriggerSmallCat : MonoBehaviour
{
	[Header("Hand Detection")]
	public Transform handRoot;
	public string handTag = "Hand";

	[Header("Python Bridge")]
	public PythonMotorBridge pythonBridge;

	[Header("Raw Command")]
	[Tooltip("碰到手時送出的原始指令")]
	public string commandOnTrigger = "m";

	[Header("Baseline Guard")]
	[Tooltip("進入關卡後需先按 B 記錄 baseline，才能觸發")]
	public bool requireBaselineBeforeTrigger = true;

	public bool enableBaselineHotkey = true;

	[Header("Cat Motion")]
	[Tooltip("要移動的小貓；若空白就移動這個物件")]
	public Transform catRoot;

	[Tooltip("前進/後退距離，0.20 = 20 公分")]
	public float moveDistance = 0.20f;

	[Tooltip("基礎移動速度（m/s）；實際會再放慢一倍")]
	public float moveSpeed = 0.2f;

	[Tooltip("碰到手後延遲多久才開始動作（秒）")]
	public float startDelaySeconds = 0.5f;

	[Tooltip("開始動作後幾秒消失在場景中")]
	public float disappearAfterSeconds = 3f;

	private bool _alreadyTriggered;
	private bool _isSending;
	private Vector3 _startLocalPosition;
	private Coroutine _motionCo;

	private void Awake()
	{
		if (catRoot == null)
			catRoot = transform;

		_startLocalPosition = catRoot.localPosition;
	}

	private void OnEnable()
	{
		if (catRoot == null)
			catRoot = transform;

		_alreadyTriggered = false;
		_isSending = false;

		if (_motionCo != null)
		{
			StopCoroutine(_motionCo);
			_motionCo = null;
		}

		catRoot.localPosition = _startLocalPosition;
	}

	private void Update()
	{
		if (enableBaselineHotkey && Input.GetKeyDown(KeyCode.B))
			RecordBaselineFromUnity();
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
		if (requireBaselineBeforeTrigger && !BaselineRequirementState.IsReady)
		{
			Debug.LogWarning("[TriggerSmallCat] Please press B to record baseline before triggering.");
			return;
		}
		if (_isSending || _alreadyTriggered) return;

		Debug.Log("[TriggerSmallCat] Hit hand (trigger): " + other.name);
		_alreadyTriggered = true;
		SendCommand();
		if (_motionCo == null)
			_motionCo = StartCoroutine(MotionSequenceCo());
	}

	private IEnumerator MotionSequenceCo()
	{
		if (catRoot == null) yield break;

		catRoot.localPosition = _startLocalPosition;

		if (startDelaySeconds > 0f)
			yield return new WaitForSeconds(startDelaySeconds);

		float speed = Mathf.Max(0.01f, moveSpeed * 0.5f);

		// 前進 5cm
		Vector3 target1 = _startLocalPosition + Vector3.right * Mathf.Max(0f, moveDistance);
		yield return MoveToCo(target1, speed);

		// 後退回原位
		yield return MoveToCo(_startLocalPosition, speed);

		// 持續往前，3 秒後消失
		float elapsed = 0f;
		while (elapsed < Mathf.Max(0f, disappearAfterSeconds))
		{
			elapsed += Time.deltaTime;
			if (catRoot != null)
				catRoot.localPosition += Vector3.right * speed * Time.deltaTime;
			yield return null;
		}

		_motionCo = null;
		gameObject.SetActive(false);
	}

	private IEnumerator MoveToCo(Vector3 targetLocalPos, float speed)
	{
		while (catRoot != null && Vector3.Distance(catRoot.localPosition, targetLocalPos) > 0.0005f)
		{
			catRoot.localPosition = Vector3.MoveTowards(catRoot.localPosition, targetLocalPos, speed * Time.deltaTime);
			yield return null;
		}
		if (catRoot != null)
			catRoot.localPosition = targetLocalPos;
	}

	private async void RecordBaselineFromUnity()
	{
		if (pythonBridge == null)
		{
			Debug.LogError("[TriggerSmallCat] pythonBridge is null.");
			return;
		}

		if (!BaselineRequirementState.TryBeginRecording())
			return;

		string result = await pythonBridge.RecordBaselineAsync();
		if (result == "K")
		{
			BaselineRequirementState.CompleteRecording(true);
			Debug.Log("[TriggerSmallCat] Baseline recorded successfully.");
		}
		else
		{
			BaselineRequirementState.CompleteRecording(false);
			Debug.LogError("[TriggerSmallCat] Record baseline failed: " + result);
		}
	}

	private async void SendCommand()
	{
		if (pythonBridge == null)
		{
			Debug.LogError("[TriggerSmallCat] pythonBridge is null.");
			return;
		}

		string cmd = string.IsNullOrWhiteSpace(commandOnTrigger) ? "m" : commandOnTrigger.Trim().ToLowerInvariant();
		if (string.IsNullOrEmpty(cmd))
		{
			Debug.LogWarning("[TriggerSmallCat] command is empty, skip sending.");
			return;
		}

		_isSending = true;
		_alreadyTriggered = true;

		string result = await pythonBridge.SendRawCommandAsync(cmd);
		if (string.IsNullOrEmpty(result) || !result.StartsWith("K"))
			Debug.LogError($"[TriggerSmallCat] Send failed: cmd={cmd}, result={result}");
		else
			Debug.Log($"[TriggerSmallCat] Send success: cmd={cmd}, result={result}");

		_isSending = false;
	}

	private void OnTriggerExit(Collider other) { }
}
