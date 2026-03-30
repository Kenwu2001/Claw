using UnityEngine;
using System.Collections;

public class demo : MonoBehaviour {


	public static bool ballsRotation = true;

	public GameObject _lights;
	private string address = "https://www.assetstore.unity3d.com/en/#!/search/page=1/sortby=popularity/query=publisher:20415";

	public void openURL(){
		Application.OpenURL(address);
	}


	public void toggleRotation(){
		ballsRotation=!ballsRotation;
	}

	public void toggleLight(){
		_lights.SetActive(!_lights.activeSelf);
	}

	public void toggleWireframe(){
		
	}
}
