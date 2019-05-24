using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System;
using System.IO;
using System.Threading;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using System.Collections.Generic;
using System.Text;
using System.Xml.Serialization;
using Utilities;

/// <summary>
/// Behavior that processes image/depth/audio data received over the network from KinectV2Server and makes it available to the scene.  
/// </summary>
public class KinectClient : SkeletonProvider
{
    [Space(10)]
    /// <summary>
    /// Name/IP address of the KinectV2Server. 
    /// </summary>
    public string serverIPAddress = "localhost";

    [Space(10)]
    public bool StreamSkeleton = true;

    [Space(10)]
    // FPS computation (these will not be visible in the inspector)
    private FrameRate fpsUpdate = new FrameRate(1f);
    private FrameRate fpsKinectSkeleton = new FrameRate(1f);
    [Space(10)]

    //[ReadOnly]
    public float updateFPS = 0;
    //[ReadOnly]
    public float skeletonFPS = 0;

    // State
    private bool inited = false;
    byte validBodiesCount;

    [Space(10)]
    // For network streaming of Kinect data
    private TCPNetworkStreamer skeletonClient;
    protected int skeletonPort = 10005;

    [Space(10)]

    /*private bool runThreads = true;
    //Unity Matrices - but RIGTH handed coordinate system (used by Kinect)
    [HideInInspector]
    public Matrix4x4 RGB_Intrinsics;
    [HideInInspector]
    public Matrix4x4 RGB_Extrinsics;
    [HideInInspector]
    public Matrix4x4 IR_Intrinsics;
    [HideInInspector]
    public Vector4 RGB_DistCoef;
    [HideInInspector]
    public Vector4 IR_DistCoef;*/

    // Wait handles for frame streaming events
    private AutoResetEvent nextSkeletonFrameReady = new AutoResetEvent(false);


    private Matrix4x4 localToWorldMatrix;

    //******************************************
    // For detecting frame dropping issues
    protected int numFramesReceived = 0,
        numRGBFramesReceived = 0,
        numFramesSerialized = 0,
        numRGBFramesSerialized = 0,
        numFramesRendered = 0,
        numRGBFramesRendered = 0;

    // Streamed-in frames
    private KinectSkeletonFrame nextSkeletonFrame, currentSkeletonFrame, pendingSkeletonFrame;


    private long streamStartSample;

    //State
    public bool Initialized() { return inited; }
    private bool wantsRestart;

    public void Awake() //guaranteed to be executed before any Start functions
    {

        Initialize();
    }

    /***public void SetPose(RoomAliveToolkit.Matrix pose)
    {
        // Create the world transform node
        Matrix4x4 worldToLocal = RAT2Unity.Convert(pose);// transpose matrix
        worldToLocal = UnityUtilities.ConvertRHtoLH(worldToLocal);

        transform.localPosition = worldToLocal.ExtractTranslation();
        transform.localRotation = worldToLocal.ExtractRotation();
    }*/

    public void Start()
    {
        Thread.Sleep(5);
    }

    public void Initialize()
    {
        // Create current and next frames
        currentSkeletonFrame = new KinectSkeletonFrame();
        nextSkeletonFrame = new KinectSkeletonFrame();
        pendingSkeletonFrame = new KinectSkeletonFrame();

        string[] ip = this.gameObject.name.Split('_');
        serverIPAddress = ip[1];

        // Initialize streaming of Kinect input data from file or network
        //connect to the configuration server
        if (StreamSkeleton)
        {
            skeletonClient = new TCPNetworkStreamer();
            skeletonClient.ConnectToSever(serverIPAddress, skeletonPort);
            skeletonClient.ReceivedMessage += new TCPNetworkStreamer.ReceivedMessageEventHandler(RemoteComputer_NewSkeletonFrame);
        }

        inited = true;

        Debug.Log("KinectClient Initialized: " + serverIPAddress + ":" + skeletonPort);
    }


    public unsafe void Update()
    {
        if (!inited)
        {
            return;
        }
        else
        {

            localToWorldMatrix = transform.localToWorldMatrix;

            fpsUpdate.Tick();

            //SKELETON
            if (nextSkeletonFrameReady.WaitOne(1))
            {

                lock (nextSkeletonFrame)
                    Swap<KinectSkeletonFrame>(ref nextSkeletonFrame, ref currentSkeletonFrame);

                fpsKinectSkeleton.Tick();
            }
        }
        updateFPS = (float)fpsUpdate.Framerate;
        skeletonFPS = (float)fpsKinectSkeleton.Framerate;
    }

    public override KinectSkeleton GetKinectSkeleton(int n)
    {
        if (currentSkeletonFrame == null)
            return null;
        else if (n >= 0 && n < 6)
            return currentSkeletonFrame.skeletons[n];
        else
            return null;
    }

    public List<KinectSkeleton> GetKinectSkeletons()
    {
        return currentSkeletonFrame.skeletons;
    }

    public void OnApplicationQuit()
    {
        Debug.Log("KinectClient Closing: " + serverIPAddress);

        inited = false;
        //??runThreads = false;

        if (skeletonClient != null)
        {
            skeletonClient.ReceivedMessage -= RemoteComputer_NewSkeletonFrame;
            skeletonClient.Close();
        }


        // Make sure all Kinect data streams are closed
        // if (skeletonStream != null)
        //     skeletonStream.Close();

    }

    private void RemoteComputer_NewSkeletonFrame(object state, ReceivedMessageEventArgs args)
    {
        //DateTime now = DateTime.Now;
        MemoryStream memStream = new MemoryStream(args.data);
        BinaryReader reader = new BinaryReader(memStream);

        long timeStampSkeleton = reader.ReadInt64();
        validBodiesCount = reader.ReadByte();

        for (int cnt = 0; cnt < 6; cnt++)
        {
            KinectSkeleton skeleton = pendingSkeletonFrame.skeletons[cnt];

            if (cnt < validBodiesCount)
            {
                skeleton.valid = true;
                skeleton.ID = (ulong)reader.ReadInt64();
                skeleton.jointPositions3D = new Vector3[25];

                for (int i = 0; i < KinectSkeleton.JOINT_COUNT; i++)
                {
                    var position = new Vector3();
                    position.x = -reader.ReadSingle();
                    position.y = reader.ReadSingle();
                    position.z = reader.ReadSingle();
                    skeleton.jointPositions3D[i] = position;

                    skeleton.jointStates[i] = (KinectSkeleton.TrackingState)reader.ReadByte();
                }
                skeleton.handLeftConfidence = reader.ReadByte();  //0=not confident, 1= confident
                skeleton.handLeftState = reader.ReadByte();
                skeleton.handRightConfidence = reader.ReadByte();
                skeleton.handRightState = reader.ReadByte();
            }
            else
            {
                skeleton.valid = false;
            }
        }


        //accelerometer reading
        float x, y, z, w;
        x = reader.ReadSingle();
        y = reader.ReadSingle();
        z = reader.ReadSingle();
        w = reader.ReadSingle();
        pendingSkeletonFrame.deviceAcceleration = KinectToWorldNoT(new Vector3(x / w, y / w, z / w)); //mProCamUnit.KinectToWorldNoT(new Vector3(x / w, y / w, z / w));
        pendingSkeletonFrame.timeStampSkeleton = timeStampSkeleton;

        reader.Close();

        lock (nextSkeletonFrame)
        {
            Swap<KinectSkeletonFrame>(ref nextSkeletonFrame, ref pendingSkeletonFrame);

            /*/ Skeletal data serialization
            if (StreamToFile)
            {
                // Serialize skeleton frame to file
                formatter.Serialize(skeletonStream, nextSkeletonFrame);
            }*/
        }

        nextSkeletonFrameReady.Set();

    }


    public static void Swap<T>(ref T lhs, ref T rhs)
    {
        T temp;
        temp = lhs;
        lhs = rhs;
        rhs = temp;
    }

    public byte GetValidBodyCount()
    {
        lock (nextSkeletonFrame)
        {
            return currentSkeletonFrame.validBodiesCount;
        }
    }

    public unsafe Vector3 GetAccReading()
    {
        return new Vector3(currentSkeletonFrame.deviceAcceleration.x,
            currentSkeletonFrame.deviceAcceleration.y,
            currentSkeletonFrame.deviceAcceleration.z);
    }

    #region Transform Conversions
        //Helper function which converts a position in the Kinect coordinate space to the world coordinates
        public override Vector3 KinectToWorld(Vector3 pos)
        {
            Vector4 headPos4W = localToWorldMatrix * new Vector4(-pos.x, pos.y, pos.z, 1.0f);  //also converts it from RH to LH
            return new Vector3(headPos4W.x / headPos4W.w, headPos4W.y / headPos4W.w, headPos4W.z / headPos4W.w);
        }


        //Helper function which converts a position in the Kinect coordinate space to the world coordinates
        //reset translation (used for accelerations and velocities)
        public Vector3 KinectToWorldNoT(Vector3 pos)
        {
            Matrix4x4 tmp = localToWorldMatrix;
            tmp.m30 = tmp.m31 = tmp.m32 = 0; //set translation to 0
            tmp.m03 = tmp.m13 = tmp.m23 = 0; //set translation to 0
            Vector4 headPos4W = tmp * new Vector4(-pos.x, pos.y, pos.z, 1.0f);
            return new Vector3(headPos4W.x / headPos4W.w, headPos4W.y / headPos4W.w, headPos4W.z / headPos4W.w);
        }

        //Helper function which converts a position in world coordinates to the Kinect coordinate space
        public Vector3 WorldToKinect(Vector3 pos)
        {
            Vector4 headPos4 = localToWorldMatrix * new Vector4(pos.x, pos.y, pos.z, 1.0f); //also converts it from RH to LH
            return new Vector3(-headPos4.x / headPos4.w, headPos4.y / headPos4.w, headPos4.z / headPos4.w);
        }

        //Helper function which converts a position in world coordinates to the Kinect coordinate space
        //reset translation (used for accelerations and velocities)
        public Vector3 WorldToKinectNoT(Vector3 pos)
        {
            Matrix4x4 tmp = localToWorldMatrix;
            tmp.m30 = tmp.m31 = tmp.m32 = 0; //set translation to 0
            tmp.m03 = tmp.m13 = tmp.m23 = 0; //set translation to 0
            Vector4 headPos4 = tmp * new Vector4(pos.x, pos.y, pos.z, 1.0f);
            return new Vector3(-headPos4.x / headPos4.w, headPos4.y / headPos4.w, headPos4.z / headPos4.w);  //also converts it from LH to RH
        }

        Quaternion GetLocalRotation()
        {
            Matrix4x4 m = transform.localToWorldMatrix;
            // Adapted from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
            Quaternion q = new Quaternion();
            q.w = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2;
            q.x = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2;
            q.y = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2;
            q.z = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2;
            q.x *= Mathf.Sign(q.x * (m[2, 1] - m[1, 2]));
            q.y *= Mathf.Sign(q.y * (m[0, 2] - m[2, 0]));
            q.z *= Mathf.Sign(q.z * (m[1, 0] - m[0, 1]));
            return q;
        }
        #endregion
}



#region Serialization Helper stuff

[Serializable]
public class KinectSkeletonFrame
{
    public KinectSkeletonFrame()
    {
        skeletons = new List<KinectSkeleton>(6);
        for (int i = 0; i < 6; i++) skeletons.Add(new KinectSkeleton());
    }

    public long timeStampSkeleton;
    public byte validBodiesCount;
    public List<KinectSkeleton> skeletons;

    [NonSerializedAttribute]
    public Vector3 deviceAcceleration = new Vector3();
    [NonSerializedAttribute]
    public bool flipped = false;
}

#endregion

