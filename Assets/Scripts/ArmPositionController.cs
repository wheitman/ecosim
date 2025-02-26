using System;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using RosMessageTypes.Sensor;
using System.Linq;
using System.Collections.Generic;
using UnityEngine.Assertions;


public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
public enum ControlType { PositionControl };

public class ArmPositionController : MonoBehaviour
{
    private ArticulationBody[] articulationChain;
    // Stores original colors of the part being highlighted
    private Color[] prevColor;
    private int previousIndex;

    [InspectorReadOnly(hideInEditMode: true)]
    public string selectedJoint;
    [HideInInspector]
    public int selectedIndex;

    public ControlType control = (ControlType)Unity.Robotics.UrdfImporter.Control.ControlType.PositionControl;
    public float stiffness;
    public float damping;
    public float forceLimit;
    public float speed = 5f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2

    [Range(-90, 90)]
    public double jointAngle1;
    [Range(-90, 90)]
    public double jointAngle2;
    [Range(-90, 90)]
    public double jointAngle3;
    [Range(-90, 90)]
    public double jointAngle4;
    [Range(-90, 90)]
    public double jointAngle5;
    [Range(-90, 90)]
    public double jointAngle6;


    [Tooltip("Color to highlight the currently selected join")]
    public Color highLightColor = new Color(1.0f, 0, 0, 1.0f);

    void Start()
    {
        previousIndex = selectedIndex = 1;
        this.gameObject.AddComponent<FKRobot>();
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
    }

    void SetSelectedJointIndex(int index)
    {
        if (articulationChain.Length > 0)
        {
            selectedIndex = (index + articulationChain.Length) % articulationChain.Length;
        }
    }

    void Update()
    {
        // bool SelectionInput1 = Input.GetKeyDown("right");
        // bool SelectionInput2 = Input.GetKeyDown("left");

        // SetSelectedJointIndex(selectedIndex); // to make sure it is in the valid range

        // // for (int i = 0; i < 6; i++)
        // // {
        // //     UpdateDirection(i);
        // // }

        // if (SelectionInput2)
        // {
        //     SetSelectedJointIndex(selectedIndex - 1);
        //     Highlight(selectedIndex);
        // }
        // else if (SelectionInput1)
        // {
        //     SetSelectedJointIndex(selectedIndex + 1);
        //     Highlight(selectedIndex);
        // }

        // UpdateDirection(selectedIndex);
    }

    public JointStateMsg GetJointStates()
    {
        JointStateMsg msg = new();

        List<string> names = new List<string>();
        List<double> positions = new List<double>();
        List<double> velocities = new List<double>();
        List<double> efforts = new List<double>();
        for (int jointIndex = 0; jointIndex < articulationChain.Length; jointIndex++)
        {

            JointControl current = articulationChain[jointIndex].GetComponent<JointControl>();

            // Debug.Log(current.joint.xDrive.target);
            if (current.joint.dofCount != 1) continue; // Only consider joints with 1 DOF

            positions.Add(current.joint.jointPosition[0]);
            names.Add(current.name);
            // Debug.Log($"names.Count = {names.Count}");

            velocities.Add(current.joint.jointVelocity[0]);
            efforts.Add(current.joint.jointForce[0]);
            // Debug.Log($"{current.name} = {current.joint.jointPosition[0]}");
        }

        Debug.Assert(names.Count == positions.Count, "Joint names and positions do not have same length");

        if (positions.Count < 0)
        {
            Debug.Log("Positions was empty!");
        }
        if (names.Count < 0)
        {
            Debug.Log("Names was empty!");
        }

        msg.name = names.ToArray();

        // Debug.Log($"msg.name.Length = {msg.name.Length}");

        msg.position = positions.ToArray();
        msg.velocity = velocities.ToArray();
        msg.effort = efforts.ToArray();

        return msg;
    }

    public void SetTargetAngles(double[] targetAngles)
    {

        // // Initialize six joint angles to zero
        // targetAngles = new double[6] { 0, 22.6, 86.4, 7.3, -10.1, -16.3 };
        // // targetAngles = new double[6] { 0, 0, 0, 0, 0, 0 };


        // JointControl current = articulationChain[0].GetComponent<JointControl>();
        // Debug.Log(current.joint.xDrive.target);
        // ArticulationDrive drive = current.joint.xDrive;
        // drive.target = (float)jointAngle1;
        // current.joint.xDrive = drive;

        // current = articulationChain[1].GetComponent<JointControl>();
        // Debug.Log(current.joint.xDrive.target);
        // drive = current.joint.xDrive;
        // drive.target = (float)jointAngle2;
        // current.joint.xDrive = drive;


        for (int jointIndex = 0; jointIndex < targetAngles.Length; jointIndex++)
        {

            JointControl current = articulationChain[jointIndex + 1].GetComponent<JointControl>();

            // Debug.Log(current.joint.xDrive.target);

            ArticulationDrive drive = current.joint.xDrive;

            // Debug.Log($"{current.name} = {targetAngles[jointIndex]}");

            drive.target = (float)targetAngles[jointIndex] * Mathf.Rad2Deg;
            current.joint.xDrive = drive;

            // Debug.Log(current.joint.xDrive.target);
            // UpdateDirection(jointIndex);
        }
    }

    public void UpdateControlType(JointControl joint)
    {
        joint.controltype = (Unity.Robotics.UrdfImporter.Control.ControlType)control;
        if (control == ControlType.PositionControl)
        {
            ArticulationDrive drive = joint.joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            joint.joint.xDrive = drive;
        }
    }
}
