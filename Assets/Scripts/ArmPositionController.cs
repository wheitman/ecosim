using System;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;


// public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };

public class ArmPositionController : MonoBehaviour
{
    private ArticulationBody[] articulationChain;
    private int previousIndex;
    public float forceLimit;
    public float speed = 5f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2


    void Start()
    {
        // previousIndex = selectedIndex = 1;
        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            // joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.stiffness = 10000.0f;
            currentDrive.damping = 100.0f;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
        // DisplaySelectedJoint(selectedIndex);
    }


    void Update()
    {
        bool SelectionInput1 = Input.GetKeyDown("right");
        bool SelectionInput2 = Input.GetKeyDown("left");

        // SetSelectedJointIndex(selectedIndex); // to make sure it is in the valid range
        // UpdateDirection(selectedIndex);

        // if (SelectionInput2)
        // {
        //     SetSelectedJointIndex(selectedIndex - 1);
        // }
        // else if (SelectionInput1)
        // {
        //     SetSelectedJointIndex(selectedIndex + 1);
        // }

        // UpdateDirection(selectedIndex);
        // for (int i = 0; i < articulationChain.Length; i++)
        // {
        //     ArticulationBody joint = articulationChain[i];
        //     ArticulationDrive xDrive = joint.xDrive;
        //     xDrive.target = 1.5f;
        //     Debug.Log($"Joint {joint.name} target: {xDrive.target}");

        //     joint.xDrive = xDrive;
        // }
    }

    public void SetTargetAngles(double[] angles)
    {
        for (int i = 0; i < angles.Length; i++)
        {

            // Get the matching articulation body from the chain
            for (int j = 0; j < articulationChain.Length; j++)
            {
                if (articulationChain[j].name == "link" + (i + 1))
                {
                    // Set the target angle for the joint
                    ArticulationBody joint = articulationChain[j];
                    ArticulationDrive xDrive = joint.xDrive;
                    xDrive.target = (float)angles[i] * Mathf.Rad2Deg;
                    joint.xDrive = xDrive;
                    break;
                }
                if (j == articulationChain.Length - 1)
                {
                    Debug.LogWarning($"Joint with name 'link{i + 1}' not found in the articulation chain.");
                }
            }

        }
    }


    // /// <summary>
    // /// Sets the direction of movement of the joint on every update
    // /// </summary>
    // /// <param name="jointIndex">Index of the link selected in the Articulation Chain</param>
    // private void UpdateDirection(int jointIndex)
    // {
    //     if (jointIndex < 0 || jointIndex >= articulationChain.Length)
    //     {
    //         return;
    //     }

    //     float moveDirection = Input.GetAxis("Vertical");
    //     JointControl current = articulationChain[jointIndex].GetComponent<JointControl>();
    //     if (previousIndex != jointIndex)
    //     {
    //         JointControl previous = articulationChain[previousIndex].GetComponent<JointControl>();
    //         previous.direction = RotationDirection.None;
    //         previousIndex = jointIndex;
    //     }

    //     if (current.controltype != ControlType.PositionControl)
    //     {
    //         UpdateControlType(current);
    //     }

    //     // if (moveDirection > 0)
    //     // {
    //     //     current.direction = RotationDirection.Positive;
    //     // }
    //     // else if (moveDirection < 0)
    //     // {
    //     //     current.direction = RotationDirection.Negative;
    //     // }
    //     // else
    //     // {
    //     //     current.direction = RotationDirection.None;
    //     // }


    // }

    // public void UpdateControlType(JointControl joint)
    // {
    //     ArticulationDrive drive = joint.joint.xDrive;
    //     drive.stiffness = stiffness;
    //     drive.damping = damping;
    //     joint.joint.xDrive = drive;
    // }
}
