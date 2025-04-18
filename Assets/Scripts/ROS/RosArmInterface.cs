using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;
using RosMessageTypes.Geometry;
using UnityEngine.Rendering;
using Unity.VisualScripting;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Tf2;
// using Preliy.Flange;

public class RosArmInterface : MonoBehaviour
{
    private ArticulationBody[] articulationChain;

    // ROS Connector
    ROSConnection m_Ros;

    // Robot6ROffsetWrist flangeRobot;
    List<string> jointNames;

    public float forceLimit;

    [SerializeField]
    Camera gameCamera;

    [SerializeField]
    BoxCollider gripperTriggerCollider;

    [Range(1, 100)]
    public int jointStatePublishRateHz;

    ArmPositionController armPositionController;

    TimeMsg currentTime = new TimeMsg();

    private GameObject clickIndicator;
    private float indicatorDuration = 2.0f; // How long to show the indicator


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

        m_Ros.RegisterPublisher<JointStateMsg>("/arm_joint_states");
        m_Ros.RegisterPublisher<RosMessageTypes.Geometry.PointMsg>("/ecosim/clicked_point");

        m_Ros.Subscribe<JointStateMsg>("/joint_commands", JointCommandCb);
        m_Ros.Subscribe<TFMessageMsg>("/tf", TransformCb);

        // Publish joint states at rate jointStatePublishRateHz
        InvokeRepeating("PublishJointStates", 0.01f, (float)1 / jointStatePublishRateHz);
        InvokeRepeating("GetMouseTarget", 0.01f, 0.1f);
    }

    void GetMouseTarget()
    {
        if (Input.GetMouseButton(0) == false) return;

        Ray ray = gameCamera.ScreenPointToRay(Input.mousePosition);

        RaycastHit hit;

        LayerMask terrainMask = LayerMask.GetMask("Terrain");

        if (Physics.Raycast(ray, out hit, terrainMask))
        {

            // Draw an arrow at the hit point using the normal
            Vector3 arrowStart = hit.point;
            Vector3 arrowEnd = hit.point + hit.normal * 0.2f; // Small arrow along the normal
            Debug.DrawLine(arrowStart, arrowEnd, Color.red, 0.1f);

            // Draw small lines to create arrowhead
            Vector3 right = Vector3.Cross(hit.normal, Vector3.up).normalized * 0.05f;
            if (right.magnitude < 0.01f) // If normal is parallel to up
                right = Vector3.Cross(hit.normal, Vector3.forward).normalized * 0.05f;
            Vector3 up = Vector3.Cross(hit.normal, right).normalized * 0.05f;
            Debug.DrawLine(arrowEnd, arrowEnd - hit.normal * 0.05f + right, Color.red, 0.1f);
            Debug.DrawLine(arrowEnd, arrowEnd - hit.normal * 0.05f - right, Color.red, 0.1f);
            Debug.DrawLine(arrowEnd, arrowEnd - hit.normal * 0.05f + up, Color.red, 0.1f);
            Debug.DrawLine(arrowEnd, arrowEnd - hit.normal * 0.05f - up, Color.red, 0.1f);

            // Debug.Log(hit.point);
            Vector3 localPoint = transform.InverseTransformPoint(hit.point);
            // localPoint.z = 0.0f;
            // Debug.Log($"localPoint = {localPoint}");

            // Send to ROS
            PointMsg msg = localPoint.To<FLU>();
            m_Ros.Publish("/ecosim/clicked_point", msg);
        }


    }

    void TransformCb(TFMessageMsg tfMessage)
    {
        // Debug.Log("TransformCb called");

        currentTime = tfMessage.transforms[0].header.stamp;
    }

    void JointCommandCb(JointStateMsg jointCommand)
    {

        double[] angles = new double[6];

        // Print all strings in array on the same line
        for (int i = 0; i < jointCommand.name.Length; i++)
        {
            // Debug.Log(jointCommand.name[i] + " " + jointCommand.position[i]);
            int jointIndex = jointCommand.name[i][jointCommand.name[i].Length - 1] - '0';
            angles[jointIndex - 1] = (float)jointCommand.position[i];
            Debug.Log($"Joint {jointIndex} angle: {angles[jointIndex - 1]}");
        }


        armPositionController.SetTargetAngles(angles);

        // m_Ros.Publish("/ecosim/joint_state", jointCommand);
    }

    void PublishJointStates()
    {
        // JointStateMsg msg = armPositionController.GetJointStates();
        // JointStateMsg msg = new();
        // msg.header.stamp = currentTime;

        // m_Ros.Publish("/arm_joint_states", msg);
    }

    // Update is called once per frame
    void Update()
    {
    }
}
