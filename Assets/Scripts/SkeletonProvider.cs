using UnityEngine;
using System.Collections;
using System;

/// <summary>
/// An abstract class that provides the basic functionality for handling and drawing Kinect skeletons in the Unity Editor. 
/// </summary>
public abstract class SkeletonProvider : MonoBehaviour
{
    public Material BoneMaterial;
    public Material JointMaterial;

    //parameters for Skeleton Debug view (in Scene)
    protected Color jointColor;
    //protected Color faceColor = Color.yellow;
    protected float jointRadius = 0.025f;
    //protected float eyeRadius = 0.015f;
    //protected float noseRadius = 0.01f;
    //protected float mouthRadius = 0.008f;
    protected int lineWidth = 2;

    public bool showSkeletons = true;

    public abstract KinectSkeleton GetKinectSkeleton(int n);
    public abstract Vector3 KinectToWorld(Vector3 pos);

    public Transform[] joints = new Transform[HumanTopology.JOINT_NAMES.Length];

    /*private void drawEye(Vector3 pos, DetectionResult closed, Quaternion rotation)
    {
        Color color = faceColor;
        if (closed == DetectionResult.Unknown)
            color.a *= 0.4f;
        if (closed == DetectionResult.Maybe)
            color.a *= 0.8f;
        Gizmos.color = color;
        if (closed != DetectionResult.Yes)
            Gizmos.DrawSphere(pos, eyeRadius);
        else
        {
            Gizmos.matrix = transform.localToWorldMatrix * Matrix4x4.TRS(pos, rotation, new Vector3(0.9f, 0.2f, 0.9f));
            Gizmos.DrawSphere(Vector3.zero, eyeRadius);
            Gizmos.matrix = transform.localToWorldMatrix;
        }

    }*/

    public void CreateSkeleton(User user)
    {
        if (Application.isPlaying && showSkeletons)
        {
            KinectSkeleton skeleton = GetKinectSkeleton(user.skeletonID);
            if (skeleton != null && (skeleton.valid))
            {
                for (int i = 0; i < skeleton.jointPositions3D.Length; i++)
                {
                    GameObject jointObj = GameObject.CreatePrimitive((PrimitiveType.Sphere));

                    LineRenderer lr = jointObj.AddComponent<LineRenderer>();
                    lr.positionCount = 2;
                    lr.material = BoneMaterial;
                    lr.startWidth = 0.005f;
                    lr.endWidth = 0.005f;

                    JointMaterial = jointObj.GetComponent<Renderer>().material;
                    if (user.userID == 0)
                        JointMaterial.color = Color.red;
                    else if (user.userID == 1)
                        JointMaterial.color = Color.green;
                    else if (user.userID == 2)
                        JointMaterial.color = Color.yellow;
                    else if (user.userID == 3)
                        JointMaterial.color = Color.cyan;
                    else if (user.userID == 4)
                        JointMaterial.color = Color.magenta;

                    jointObj.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
                    jointObj.name = HumanTopology.JOINT_NAMES[i];
                    jointObj.transform.parent = user.gameObject.transform;

                    joints[i] = jointObj.transform;
                }
            }
        }
    }

    public void RefreshBodyObject(User user)//Kinect.Body body, GameObject bodyObject)
    {
        if (Application.isPlaying && showSkeletons)
        {
            KinectSkeleton skeleton = GetKinectSkeleton(user.skeletonID);
            if (skeleton != null && (skeleton.valid))
            {
                for (int i = 0; i < skeleton.jointPositions3D.Length; i++)
                {
                    KinectSkeleton.TrackingState state = skeleton.jointStates[i];
                    bool tracked = state != KinectSkeleton.TrackingState.NotTracked;
                    //JointMaterial = joints[i].GetComponent<Renderer>().material;
                    //JointMaterial.color = GetColorForState(state);
                    if (tracked)
                    {
                        joints[i].localPosition = skeleton.jointPositions3D[i];
                    }
                }

                for (int k = 0; k < HumanTopology.BONE_CONNECTIONS.Length; k++)
                {
                    BoneConnection bone = HumanTopology.BONE_CONNECTIONS[k];
                    JointType joint1 = bone.fromJoint;
                    JointType joint2 = bone.toJoint;
                    LineRenderer lr = joints[(int)joint2].GetComponent<LineRenderer>();

                    lr.SetPosition(0, joints[(int)joint1].position);
                    lr.SetPosition(1, joints[(int)joint2].position);
                }
                /*for (int i = 0; i < HumanTopology.BONE_CONNECTIONS.Length; i++)
                {
                    BoneConnection bone = HumanTopology.BONE_CONNECTIONS[i];
                    JointType joint1 = bone.fromJoint;
                    JointType joint2 = bone.toJoint;
                    KinectSkeleton.TrackingState state1 = skeleton.jointStates[(int)joint1];
                    KinectSkeleton.TrackingState state2 = skeleton.jointStates[(int)joint2];
                    LineRenderer lr = joints[i].GetComponent<LineRenderer>();
                    bool tracked = state1 != KinectSkeleton.TrackingState.NotTracked && state2 != KinectSkeleton.TrackingState.NotTracked;
                    if (tracked)
                    {
                        lr.SetPosition(0, skeleton.jointPositions3D[(int)joint1]);
                        lr.SetPosition(1, skeleton.jointPositions3D[(int)joint2]);

                        lr.startColor = GetColorForState(state1);
                        lr.endColor = GetColorForState(state2);
                    }

                }*/
            }
        }
    }

    private static Color GetColorForState(KinectSkeleton.TrackingState state)
    {

        switch (state)
        {
            case KinectSkeleton.TrackingState.Tracked:
                return Color.green;

            case KinectSkeleton.TrackingState.Inferred:
                return Color.red;

            default:
                return Color.black;
        }
    }
}





