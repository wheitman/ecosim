using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using Preliy.Flange;

public class RosArmInterface : MonoBehaviour
{
    private ArticulationBody[] articulationChain;

    // ROS Connector
    ROSConnection m_Ros;

    Robot6ROffsetWrist flangeRobot;
    List<string> jointNames;

    public float forceLimit;

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

        m_Ros.RegisterPublisher<RosMessageTypes.Sensor.JointStateMsg>("/joint_states");

        m_Ros.Subscribe<RosMessageTypes.Sensor.JointStateMsg>("/joint_states", HandleJointStateMessage);
    }


    void HandleJointStateMessage(RosMessageTypes.Sensor.JointStateMsg jointStateMsg)
    {

        double[] angles = new double[6];

        // Print all strings in array on the same line
        for (int i = 0; i < jointStateMsg.name.Length; i++)
        {
            Debug.Log(jointStateMsg.name[i] + " " + jointStateMsg.position[i]);
            int jointIndex = jointStateMsg.name[i][jointStateMsg.name[i].Length - 1] - '0';
            angles[jointIndex - 1] = (float)jointStateMsg.position[i];
        }


        armPositionController.SetTargetAngles(angles);
    }

    // Update is called once per frame
    void Update()
    {

        // Publish joint states to ROS
        // var jointStateMsg = new RosMessageTypes.Sensor.JointStateMsg();
        // jointStateMsg.name = jointNames.ToArray();

        // List<double> jointPositions = new List<double>();
        // foreach (var joint in flangeRobot.Joints)
        // {
        //     jointPositions.Add(joint.Position.Value);
        //     Debug.Log(joint.name + " " + joint.Position);
        // }

        // jointStateMsg.position = jointPositions.ToArray();
        // m_Ros.Publish("/joint_states", jointStateMsg);
    }
}
