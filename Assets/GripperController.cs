using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class GripperController : MonoBehaviour
{

    ROSConnection m_Ros;

    bool gripperClosed = false;


    // Start is called before the first frame update
    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.Subscribe<BoolMsg>("/close_gripper", CloseGripperCb);
    }

    void CloseGripperCb(BoolMsg msg)
    {
        gripperClosed = msg.data;

        if (msg.data)
            Debug.Log("Gripper is now closed");
        else
            Debug.Log("Gripper is now open");

    }

    // Update is called once per frame
    void Update()
    {

    }

    void OnTriggerStay(Collider other)
    {
        if (other.name.IndexOf("finger") >= 0 || other.name == "Terrain")
            return; // Ignore fingers and terrain

        Debug.Log(other.name);
        if (!other.transform.parent.GetComponent<Rigidbody>())
            return; // Only consider collisions with RBs

        // Debug.Log(other.name);
        // Debug.Log("BOOM");

        other.transform.parent.SetParent(transform);

        if (gripperClosed) // Pick up
        {
            other.transform.parent.SetParent(transform);
            other.transform.parent.GetComponent<Rigidbody>().isKinematic = true;
        }
        else // Release
        {
            Debug.Log($"Releasing {other.transform.parent.name}");
            other.transform.parent.SetParent(null);
            other.transform.parent.GetComponent<Rigidbody>().isKinematic = false;
        }
    }

}
