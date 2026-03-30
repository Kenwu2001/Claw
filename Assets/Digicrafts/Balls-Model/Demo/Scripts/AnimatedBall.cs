using UnityEngine;
using System.Collections;

public class AnimatedBall : MonoBehaviour {

	private Quaternion _startRotation;

	// Use this for initialization
	void Start () {

		_startRotation = gameObject.transform.rotation;
	}
	
	// Update is called once per frame
	void Update () {
	
		if(demo.ballsRotation){
			Vector3 v = gameObject.transform.rotation.eulerAngles;
			v.y += 2;
			_startRotation.eulerAngles=v;
			gameObject.transform.rotation =_startRotation;
		}
	}
}
