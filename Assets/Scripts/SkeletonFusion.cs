using System.Collections;
using System.Collections.Generic;
using System.Net;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using pointmatcher.net;
using Kalman;

public class SkeletonFusion : MonoBehaviour {

    GameObject sceneCamera;
    GameObject canvas;
    public GameObject popupCanvas;
    KinectIPSettings ipSettings;
    List<string> ipAddresses;
    GameObject kinectModel;
    GameObject fusedSkeleton;
    Transform[] fusedJoints;

    GameObject[] kinects;
    User[] users;

    Vector3[][] pointSets;
    Vector4[] targetPointSet;

    System.Numerics.Vector3 v;
    List <DataPoint>[] dataPointLists;
    EuclideanTransform[] initialTransforms;

    public bool kabsch = false;
    public bool kalman = true;
    public bool iterativeClosestPoint = true;

    public bool readyToScale = false;

    KabschSolver solver = new KabschSolver();
    ICP icp = new ICP();
    KalmanFilter[] filters;

    private void Awake()
    {
        sceneCamera = GameObject.Find("Camera");
        canvas = GameObject.Find("SettingsCanvas");
        ipSettings = canvas.GetComponent<KinectIPSettings>();
    }

    private void OnEnable()
    {
        ipAddresses = ipSettings.ipAddresses;

        kinects = new GameObject[ipAddresses.Count];
        users = new User[ipAddresses.Count];

        IPAddress address;

        //Test if the IP addresses are valid and promt to check the values if not.
        for (int i = 0; i < ipAddresses.Count; i++)
        {
            if (!IPAddress.TryParse(ipAddresses[i], out address))
            {
                popupCanvas.SetActive(true);
                this.gameObject.SetActive(false);
                return;
            }
        }

        kinectModel = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Models/Kinect.obj");

        //Create the Kinect GameObjects and Clients and provide the IP addresses.
        for (int i = 0; i < ipAddresses.Count; i++)
        { 
            GameObject kinectGameObject = new GameObject("Kinect_" + ipAddresses[i]);
            kinectGameObject.transform.parent = transform;
            KinectClient kinect = kinectGameObject.AddComponent<KinectClient>();

            //3D model for the Kinect device.
            GameObject model = Instantiate(kinectModel);
            model.name = "Kinect3DModel";
            model.transform.parent = kinectGameObject.transform;
            model.transform.localPosition = Vector3.zero;
            model.transform.localRotation = Quaternion.identity;

            //Create the game object for the skeleton
            GameObject skeleton = new GameObject("User");
            skeleton.transform.parent = kinectGameObject.transform;
            User user = skeleton.AddComponent<User>();
            user.skeletonProvider = kinect;// transform.GetComponentInChildren<KinectClient>(true);
            user.userID = i;

            kinects[i] = kinectGameObject;
            users[i] = user;
        }

        //Disable the ip settings canvas
        canvas.SetActive(false);
    }

    // Use this for initialization
    void Start()
    {
        fusedSkeleton = GameObject.Find("FusedSkeleton");
        fusedJoints = new Transform[HumanTopology.JOINT_NAMES.Length];
        for (int i = 0; i < fusedSkeleton.transform.childCount; i++)
        {
            fusedJoints[i] = fusedSkeleton.transform.GetChild(i);
        }

        //Kabsch
        pointSets = new Vector3[users.Length][];
        targetPointSet = new Vector4[HumanTopology.JOINT_NAMES.Length];
        for (int i = 0; i < users.Length ; i++)
        {
            pointSets[i] = new Vector3[HumanTopology.JOINT_NAMES.Length];
        }

        //ICP
        dataPointLists = new List<DataPoint>[users.Length];
        for (int j = 0; j < users.Length; j++)
        {
            dataPointLists[j] = new List<DataPoint>();
        }
        initialTransforms = new EuclideanTransform[users.Length];
        icp.ReadingDataPointsFilters = new RandomSamplingDataPointsFilter(0.1f);
        icp.ReferenceDataPointsFilters = new SamplingSurfaceNormalDataPointsFilter(SamplingMethod.RandomSampling, 0.2f);
        icp.OutlierFilter = new TrimmedDistOutlierFilter(0.5f);

        //Kalman
        #region Kalman filter
        if (kalman)
        {
            filters = new KalmanFilter[HumanTopology.JOINT_NAMES.Length];

            for (int i = 0; i < HumanTopology.JOINT_NAMES.Length; i++)
            {
                filters[i] = new KalmanFilter(new double[,]
                    {{0.0}, //XPos
		         {0.0}, //YPos
		         {0.0}, //ZPos
		         {0.0}, //XVel
		         {0.0}, //YVel
		         {0.0}, //ZVel
		         {0.0}, //XAccel
		         {0.0}, //YAccel
		         {0.0}, //ZAccel
                    },

                     //Measurement Matrix
                     //Tells the filter which of the above we're actually sensing
                     //Turns out you can just straight up comment out the lines that 
                     //you want the filter to solve for
                     new double[,]
                      {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //XPos
                   {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //YPos - Not Measured!
                   {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //ZPos - Not Measured!
                   {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //XVel - Not Measured!
                   {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}, //YVel - Not Measured!
                   {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, //ZVel - Not Measured!
                   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, //XAccel
                   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}, //YAccel - Not Measured!
                   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0} //ZAccel - Not Measured!
                      },

                     //Process Noise; how much each value will deviate from the predicted value
                     //Quick gaussian distribution intuition
                     //If the ProcessNoise is 0.1 then you're saying that
                     //68% of the time, the actual system will deviate less than sqrt(0.1) *from the predicted value*
                     //and 95% of the time, less than 2*sqrt(0.1) etc.
                     //It's a measure of how good your StateTransitionMatrix is
                     //Ours sucks since it doesn't factor in that a meddling sine wave is moving the tracker around
                     //So our numbers need to be kind of high
                     new double[,]
                      {{.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //XPos
		           {0.0, .25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //YPos
		           {0.0, 0.0, .25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //ZPos
		           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //XVel
		           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //YVel
		           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //ZVel
		           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0}, //Constant XAccel
		           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0}, //Constant YAccel
		           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025} //Constant ZAccel
                      },

                     //Same as above, except now it's the measurement deviating from actual position (not the predicted position)
                     new double[,]
                      {{60.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //XPos
                   {0.0, 60.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //YPos
                   {0.0, 0.0, 60.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //ZPos
                   //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //XVel
                   //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //YVel
                   //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, //ZVel
                   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0}, //XAccel
                   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0005, 0.0}, //YAccel
                   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0005} //ZAccel
                      },

                     //Initial Error measurement; 0.0 = slow initial convergence, 1.0 = fast initial convergence
                     0.1
                 );
            }
        }
        #endregion
    }

    private void Update()
    {
        //Update the kinect data
        for (int i = 0; i < users.Length; i++)
        {
            if (users[i].skeletonProvider != null)
            {
                KinectSkeleton skeleton = users[i].GetSkeleton();

                if (skeleton != null && skeleton.valid)
                {
                    if (!users[i].created)
                    {
                        users[i].skeletonProvider.CreateSkeleton(users[i]);
                        users[i].created = true;
                    }

                    if (users[i].updateFromKinect)
                    {
                        /*transform.position = getHeadPosition();
                        if (lookAt != null)
                        {
                            transform.LookAt(lookAt.transform.position);
                        }
                        //else transform.localRotation = Quaternion.identity;*/

                        users[i].skeletonProvider.RefreshBodyObject(users[i]);
                    }

                }
            }
        }

        #region Kabsch implementation
        if (kabsch)
        {
            //Get the latest Tranform positions
            for (int i = 0; i < users.Length; i++)
            {
                if (users[i].GetSkeleton() != null && users[i].GetSkeleton().valid)
                {
                    for (int j = 0; j < users[i].skeletonProvider.joints.Length; j++)
                    {
                        pointSets[i][j] = users[i].skeletonProvider.joints[j].position;
                    }
                }
            }
            for (int j = 0; j < targetPointSet.Length; j++)
            {
                if (users[0].GetSkeleton() != null && users[0].GetSkeleton().valid)
                {
                    targetPointSet[j] = new Vector4(users[0].skeletonProvider.joints[j].position.x, users[0].skeletonProvider.joints[j].position.y, users[0].skeletonProvider.joints[j].position.z, users[0].skeletonProvider.joints[j].localScale.x);
                }
            }

            //Calculate new positions for the skeletons
            for (int k = 1; k < users.Length; k++)
            {
                if (users[k].GetSkeleton() != null && users[k].GetSkeleton().valid)
                {
                    Matrix4x4 kabschTransform = solver.SolveKabsch(pointSets[k], targetPointSet);

                    for (int i = 0; i < users[k].skeletonProvider.joints.Length; i++)
                    {
                        users[k].skeletonProvider.joints[i].position = kabschTransform.MultiplyPoint3x4(pointSets[k][i]);

                        for (int j = 0; j < HumanTopology.BONE_CONNECTIONS.Length; j++)
                        {
                            BoneConnection bone = HumanTopology.BONE_CONNECTIONS[j];
                            JointType joint1 = bone.fromJoint;
                            JointType joint2 = bone.toJoint;
                            LineRenderer lr = users[k].skeletonProvider.joints[(int)joint2].GetComponent<LineRenderer>();

                            lr.SetPosition(0, users[k].skeletonProvider.joints[(int)joint1].position);
                            lr.SetPosition(1, users[k].skeletonProvider.joints[(int)joint2].position);
                        }
                        /*for (int j = 0; j < HumanTopology.BONE_CONNECTIONS.Length; j++)
                        {
                            BoneConnection bone = HumanTopology.BONE_CONNECTIONS[j];
                            JointType joint1 = bone.fromJoint;
                            JointType joint2 = bone.toJoint;
                            KinectSkeleton.TrackingState state1 = users[k].GetSkeleton().jointStates[(int)joint1];
                            KinectSkeleton.TrackingState state2 = users[k].GetSkeleton().jointStates[(int)joint2];
                            LineRenderer lr = users[k].skeletonProvider.joints[i].GetComponent<LineRenderer>();
                            bool tracked = state1 != KinectSkeleton.TrackingState.NotTracked && state2 != KinectSkeleton.TrackingState.NotTracked;
                            if (tracked)
                            {
                                lr.SetPosition(0, users[k].skeletonProvider.joints[(int)joint1].position);
                                lr.SetPosition(1, users[k].skeletonProvider.joints[(int)joint2].position);
                            }
                        }*/
                    }
                }
            }
        }
        #endregion

        #region Kalman implementation
        if (kalman)
        {
            for (int i = 0; i < HumanTopology.JOINT_NAMES.Length; i++)
            {
                for (int j = 0; j < users.Length; j++)
                {
                    if (users[j].GetSkeleton() != null && users[j].GetSkeleton().valid)
                    {
                        filters[i].PredictState(Time.deltaTime);

                        filters[i].UpdateState(new double[,]
                            {{users[j].skeletonProvider.joints[i].position.x},
                        {users[j].skeletonProvider.joints[i].position.y},
                        {users[j].skeletonProvider.joints[i].position.z},
                        {0.0 },
                        {0.0 },
                        {0.0 },
                        {0.0 },
                        {0.0 },
                        {0.0 } });

                        double oldPos = filters[i].StateMatrix[0, 0];

                        /*for (int k = 1; k < 10; k++)
                        {
                            double[,] PredictedState = filters[i].SafePredictState(0.1 * k);
                            Debug.DrawLine(new Vector3((float)oldPos, k * 0.02f, 0f), new Vector3((float)PredictedState[0, 0], (k + 1) * 0.02f, 0f), Color.green);
                            oldPos = PredictedState[0, 0];

                            Debug.DrawLine(new Vector3(Mathf.Sin((Time.time + (k * 0.1f))), k * 0.02f, 0f),
                                           new Vector3(Mathf.Sin((Time.time + ((k + 1f) * 0.1f))), (k + 1) * 0.02f, 0f), Color.red);
                        }*/

                        fusedJoints[i].position = new Vector3((float)filters[i].StateMatrix[0, 0], (float)filters[i].StateMatrix[1, 0], (float)filters[i].StateMatrix[2, 0]);
                    }
                }
            }

            for (int k = 0; k < HumanTopology.BONE_CONNECTIONS.Length; k++)
            {
                BoneConnection bone = HumanTopology.BONE_CONNECTIONS[k];
                JointType joint1 = bone.fromJoint;
                JointType joint2 = bone.toJoint;
                LineRenderer lr = fusedJoints[(int)joint2].GetComponent<LineRenderer>();

                lr.SetPosition(0, fusedJoints[(int)joint1].position);
                lr.SetPosition(1, fusedJoints[(int)joint2].position);
            }

            readyToScale = true;
        }
        #endregion

    }

    // Update is called once per frame
    void LateUpdate()
    {
        

        #region ICP implementation
        if(iterativeClosestPoint && users[0].GetSkeleton() != null && users[0].GetSkeleton().valid && users[1].GetSkeleton() != null && users[1].GetSkeleton().valid)
        {
            //Clear the old positions from the list
            for (int n = 0; n < users.Length; n++)
            {
                dataPointLists[n].Clear();
            }
            
            //Get the latest joint positions and construct DataPoints of them
            for (int i = 0; i < users.Length; i++)
            {
                if (users[i].GetSkeleton() != null && users[i].GetSkeleton().valid)
                {
                    for (int j = 0; j < HumanTopology.JOINT_NAMES.Length; j++)
                    {
                        v = new System.Numerics.Vector3(users[i].skeletonProvider.joints[j].position.x, users[i].skeletonProvider.joints[j].position.y, users[i].skeletonProvider.joints[j].position.z);

                        dataPointLists[i].Add(new DataPoint
                        {
                            point = v, //new System.Numerics.Vector3(users[i].skeletonProvider.joints[j].position.x, users[i].skeletonProvider.joints[j].position.y, users[i].skeletonProvider.joints[j].position.z),
                            normal = System.Numerics.Vector3.Normalize(v), //new System.Numerics.Vector3(users[i].skeletonProvider.joints[j].position.normalized.x, users[i].skeletonProvider.joints[j].position.normalized.y, users[i].skeletonProvider.joints[j].position.normalized.z),
                        });
                    }
                }
            }
            DataPoints[] dataPointSets = new DataPoints[users.Length];
            for (int k = 0; k < users.Length; k++)
            {
                dataPointSets[k] = new DataPoints
                {
                    points = dataPointLists[k].ToArray(),
                    contiansNormals = true,
                };
            }

            //Calculate the initial guesses of the transforms between the target skeleton and the other skeletons
            //We won't calculate the transform for the first skeleton since it is the target skeleton.
            //The transform is calculated by the first joint on the list which is the SpineBase, which should be visible from all the sensors.
            for (int m = 1; m < users.Length; m++)
            {
                if (users[m].GetSkeleton() != null && users[m].GetSkeleton().valid)
                {
                    initialTransforms[m] = new EuclideanTransform();
                    initialTransforms[m].translation = new System.Numerics.Vector3((dataPointSets[0].points[0].point.X - dataPointSets[m].points[0].point.X), (dataPointSets[0].points[0].point.Y - dataPointSets[m].points[0].point.Y), (dataPointSets[0].points[0].point.Z - dataPointSets[m].points[0].point.Z)); //System.Numerics.Vector3.Subtract(dataPointSets[0].points[0].point, dataPointSets[m].points[0].point);
                    initialTransforms[m].rotation = CalculateQuaternion(dataPointSets[m], dataPointSets[0]);
                    initialTransforms[m].rotation = System.Numerics.Quaternion.Normalize(initialTransforms[m].rotation);

                    //Compute the transform between the skeletons
                    //Debug.Log("Skeleton pisteet: " + dataPointSets[m].points[0].point.ToString() + ", " + dataPointSets[m].points[0].normal.ToString());
                    //Debug.Log("Target: " + dataPointSets[0].points.ToString());
                    //Debug.Log("Translation: " + initialTransforms[m].translation.ToString());
                    //Debug.Log("Rotation: " + initialTransforms[m].rotation.ToString());
                    var transform = icp.Compute(dataPointSets[m], dataPointSets[0], initialTransforms[m]);

                    //move and rotate the skeleton according to the transform
                    users[m].transform.position = new Vector3(transform.translation.X, transform.translation.Y, transform.translation.Z);
                    users[m].transform.rotation = new Quaternion(transform.rotation.X, transform.rotation.Y, transform.rotation.Z, transform.rotation.W);
                }
            }

            iterativeClosestPoint = false;   
        }
        #endregion

        
    }

    System.Numerics.Quaternion CalculateQuaternion(DataPoints datapoints, DataPoints targetpoints)
    {
        System.Numerics.Quaternion q = new System.Numerics.Quaternion();
        System.Numerics.Vector3 cp = System.Numerics.Vector3.Cross(datapoints.points[0].point, targetpoints.points[0].point);
        q.X = cp.X;
        q.Y = cp.Y;
        q.Z = cp.Z;
        q.W = Mathf.Sqrt(Mathf.Pow(datapoints.points[0].point.Length(), 2) * Mathf.Pow(targetpoints.points[0].point.Length(), 2)) + System.Numerics.Vector3.Dot(datapoints.points[0].point, targetpoints.points[0].point);
        return q;
    }
}
