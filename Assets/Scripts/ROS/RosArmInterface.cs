using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using System;
using RosMessageTypes.Geometry;
using UnityEngine.Rendering;
using Unity.VisualScripting;
// using Preliy.Flange;

public class RosArmInterface : MonoBehaviour
{
    private ArticulationBody[] articulationChain;

    // ROS Connector
    ROSConnection m_Ros;

    // Robot6ROffsetWrist flangeRobot;
    List<string> jointNames;

    public float forceLimit;

    // [SerializeField]
    // Camera gameCamera;

    [SerializeField]
    BoxCollider gripperTriggerCollider;

    [Range(1, 100)]
    public int jointStatePublishRateHz;

    ArmPositionController armPositionController;

    // Start is called before the first frame update
    void Start()
    {
        armPositionController = GetComponent<ArmPositionController>();

        // Set up the articulation chain
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }

        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();

        m_Ros.RegisterPublisher<JointStateMsg>("/joint_states");
        m_Ros.RegisterPublisher<RosMessageTypes.Geometry.PointMsg>("/ecosim/clicked_point");

        m_Ros.Subscribe<JointStateMsg>("/joint_commands", JointCommandCb);

        // Publish joint states at rate jointStatePublishRateHz
        InvokeRepeating("PublishJointStates", 0.01f, (float)1 / jointStatePublishRateHz);
        // InvokeRepeating("GetMouseTarget", 0.01f, 0.1f);
    }

    // void GetMouseTarget()
    // {
    //     if (Input.GetMouseButton(0) == false) return;

    //     Ray ray = gameCamera.ScreenPointToRay(Input.mousePosition);

    //     RaycastHit hit;

    //     LayerMask terrainMask = LayerMask.GetMask("Terrain");

    //     if (Physics.Raycast(ray, out hit, terrainMask))
    //     {
    //         Debug.DrawLine(transform.position, hit.point, Color.green, 0.1f);

    //         // Debug.Log(hit.point);
    //         Vector3 localPoint = transform.InverseTransformPoint(hit.point);
    //         // localPoint.z = 0.0f;
    //         // Debug.Log($"localPoint = {localPoint}");

    //         // Send to ROS
    //         PointMsg msg = localPoint.To<FLU>();
    //         m_Ros.Publish("/ecosim/clicked_point", msg);
    //     }


    // }

    void JointCommandCb(JointStateMsg jointCommand)
    {

        double[] angles = new double[6];

        // Print all strings in array on the same line
        for (int i = 0; i < jointCommand.name.Length; i++)
        {
            // Debug.Log(jointCommand.name[i] + " " + jointCommand.position[i]);
            int jointIndex = jointCommand.name[i][jointCommand.name[i].Length - 1] - '0';
            angles[jointIndex - 1] = (float)jointCommand.position[i];
        }


        armPositionController.SetTargetAngles(angles);

        // m_Ros.Publish("/ecosim/joint_states", jointCommand);
    }

    void PublishJointStates()
    {
        JointStateMsg msg = armPositionController.GetJointStates();
        // JointStateMsg msg = new();

        m_Ros.Publish("/joint_states", msg);
    }

    // Update is called once per frame
    void Update()
    {
    }
}
