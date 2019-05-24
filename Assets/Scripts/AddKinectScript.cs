using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddKinectScript : MonoBehaviour {

    Transform canvas;
    KinectIPSettings ipSettings;

    private void Awake()
    {
        canvas = this.transform.parent.parent;
        ipSettings = canvas.GetComponent<KinectIPSettings>();
    }

	public void OnRemoveKinect () {

        if (!this.gameObject.activeSelf)
        {
            this.gameObject.SetActive(true);
        }

        this.transform.Translate(0, 45, 0);

        ipSettings.ipAddresses.RemoveAt(ipSettings.ipAddresses.Count-1);
	}
	
	// Update is called once per frame
	public void OnAddKinect () {

        Transform kinectPanel2 = this.transform.parent.Find("pnlKinect2");
        Transform kinectPanel3 = this.transform.parent.Find("pnlKinect3");
        Transform kinectPanel4 = this.transform.parent.Find("pnlKinect4");
        Transform kinectPanel5 = this.transform.parent.Find("pnlKinect5");

        this.transform.Translate(0, -45, 0);

        if (!kinectPanel2.gameObject.activeSelf)
        {
            kinectPanel2.gameObject.SetActive(true);
        }
        else if (!kinectPanel3.gameObject.activeSelf)
        {
            kinectPanel3.gameObject.SetActive(true);
            kinectPanel2.Find("btnRemoveKinect2").gameObject.SetActive(false);
        }
        else if (!kinectPanel4.gameObject.activeSelf)
        {
            kinectPanel4.gameObject.SetActive(true);
            kinectPanel3.Find("btnRemoveKinect3").gameObject.SetActive(false);
        }
        else if (!kinectPanel5.gameObject.activeSelf)
        {
            kinectPanel5.gameObject.SetActive(true);
            kinectPanel4.Find("btnRemoveKinect4").gameObject.SetActive(false);
            this.gameObject.SetActive(false);
        }

        ipSettings.ipAddresses.Add("");
    }
}
