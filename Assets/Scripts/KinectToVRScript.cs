using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;
using Valve.VR.InteractionSystem;

public class KinectToVRScript : MonoBehaviour {

    SkeletonFusion skeletonFusionScript;

    GameObject vrCamera;
    GameObject vrRightHand;
    GameObject vrLeftHand;
    Transform head;
    Transform rightHand;
    Transform leftHand;

    Vector3 cameraPosition;

    Vector3[] skeletonPoints;
    Vector4[] vrPoints;

    KabschSolver solver = new KabschSolver();

    bool scalingComplete = false;

    // Use this for initialization
    void Start ()
    {
        skeletonFusionScript = GameObject.Find("KinectFusionManager").GetComponent<SkeletonFusion>();
        vrCamera = GameObject.Find("VRCamera");
        vrRightHand = GameObject.Find("RightHand");
        vrLeftHand = GameObject.Find("LeftHand");
        head = this.transform.Find("Head");
        rightHand = this.transform.Find("HandRight");
        leftHand = this.transform.Find("HandLeft");

        skeletonPoints = new Vector3[3];
        vrPoints = new Vector4[3];
	}
	
	// Update is called once per frame
	void LateUpdate ()
    {
        if (SteamVR_Input._default.inActions.ReScale.GetStateDown(SteamVR_Input_Sources.LeftHand) || skeletonFusionScript.readyToScale && scalingComplete == false)
        {
            if (SteamVR_Input._default.inActions.ScaleTrigger.GetStateDown(SteamVR_Input_Sources.LeftHand) &&
                SteamVR_Input._default.inActions.ScaleTrigger.GetStateDown(SteamVR_Input_Sources.RightHand))
            {
                Debug.Log("Should scale now");
                Scale2VR();
            }
        }
		
	}

    void Scale2VR()
    {
        cameraPosition = vrCamera.transform.position;
        float scaleBy = cameraPosition.y / head.position.y;
        this.transform.parent.localScale = new Vector3(this.transform.parent.localScale.x * scaleBy, this.transform.parent.localScale.y * scaleBy, this.transform.parent.localScale.z * scaleBy);


        //this.transform.parent.position = new Vector3(cameraPosition.x - head.position.x, cameraPosition.y / 2 - head.position.y, cameraPosition.z - head.position.x);

        //this.transform.parent.rot
        //Get the latest Tranform positions
        skeletonPoints[0] = head.position;
        skeletonPoints[1] = rightHand.position;
        skeletonPoints[2] = leftHand.position;

        vrPoints[0] = new Vector4(vrCamera.transform.position.x, vrCamera.transform.position.y, vrCamera.transform.position.z, vrCamera.transform.localScale.x);
        vrPoints[1] = new Vector4(vrRightHand.transform.position.x, vrRightHand.transform.position.y, vrRightHand.transform.position.z, vrRightHand.transform.localScale.x);
        vrPoints[2] = new Vector4(vrLeftHand.transform.position.x, vrLeftHand.transform.position.y, vrLeftHand.transform.position.z, vrLeftHand.transform.localScale.x);

        Matrix4x4 kabschTransform = solver.SolveKabsch(skeletonPoints, vrPoints);

        head.position = kabschTransform.MultiplyPoint3x4(skeletonPoints[0]);
        rightHand.position = kabschTransform.MultiplyPoint3x4(skeletonPoints[1]);
        leftHand.position = kabschTransform.MultiplyPoint3x4(skeletonPoints[2]);

        scalingComplete = true;
    }
}
