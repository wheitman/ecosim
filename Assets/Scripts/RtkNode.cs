using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Nav;

public class RtkNode : MonoBehaviour
{
    // ROS Connector
    ROSConnection m_Ros;

    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();

        m_Ros.RegisterPublisher<RosMessageTypes.Nav.OdometryMsg>("/gnss/odom");
    }

    // Update is called once per frame
    void Update()
    {
        // Form an Odometry message to hold our RTK position
        OdometryMsg msg = new OdometryMsg();
        msg.header.frame_id = "map";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = transform.position.x;
        msg.pose.pose.position.y = transform.position.y;
        msg.pose.pose.position.z = transform.position.z;
        msg.pose.pose.orientation.x = transform.rotation.x;
        msg.pose.pose.orientation.y = transform.rotation.y;
        msg.pose.pose.orientation.z = transform.rotation.z;
        msg.pose.pose.orientation.w = transform.rotation.w;

        m_Ros.Publish("/gnss/odom", msg);
    }
}
