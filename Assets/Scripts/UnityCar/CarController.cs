using System;
using System.Collections;
using System.Collections.Generic;
using ROS2;
using UnityEngine;

namespace UnityCar
{

public class CarController : MonoBehaviour
{
[SerializeField] private float mass = 1200.0f;
[SerializeField] private Vector3 coG = new Vector3(0.0f, 0.435f, -2.5f);
[SerializeField] private Vector3 inertiaTensor = new Vector3(3600.0f, 3900.0f, 800.0f);

public Transform frWheelModel;


public float GetVel {
	get { return vel; }
}
public float GetMass {
	get { return mass; }
}
public Vector3 GetCoG {
	get { return coG; }
}
public Rigidbody GetRB {
	get { return rB; }
}

private WheelCollider[] wC;
private Rigidbody rB;
private float vel;
private float steerAngle = 0.0f;

// private AeroDynamics aeroDynamics;
private Brakes brakes;
private Engine engine;
private Steering steering;
private Suspension suspension;
private Transmission transmission;
private UserInput userInput;
private MotionCommandListener rosInput;

void Awake()
{
	// aeroDynamics = GetComponent<AeroDynamics>();
	brakes = GetComponent<Brakes>();
	engine = GetComponent<Engine>();
	steering = GetComponent<Steering>();
	suspension = GetComponent<Suspension>();
	transmission = GetComponent<Transmission>();
	userInput = GetComponent<UserInput>();
	rosInput = GetComponent<MotionCommandListener>();


	// set the physics clock to 120 Hz
	Time.fixedDeltaTime = 0.008333f;
	// Find the wheel colliders and put them in an array
	// Do not re-order the CD_Colliders in the vehicle model, everything depends on them being RL RR FL FR
	// WC sequence RL RR FL FR
	wC = gameObject.GetComponentsInChildren<WheelCollider>();
	// Get and configure the vehicle rigidbody
	rB = GetComponent<Rigidbody>();
	rB.mass = 800;
	rB.centerOfMass = coG;
	rB.inertiaTensor = inertiaTensor;
	rB.isKinematic = false;
}


void FixedUpdate()
{
	// see if there is a controller script active and if so take input from it
	float inputX = 0.0f;
	float inputY = 0.0f;
	float inputR = 0.0f;
	float inputH = 0.0f;

	if (rosInput.enabled) {
		inputX = rosInput.ControllerInputX;
		inputY = rosInput.ControllerInputY;
		inputR = rosInput.ControllerInputReverse;
		inputH = rosInput.ControllerInputHandBrake;
	} else if (userInput.enabled)
	{
		inputX = userInput.ControllerInputX;
		inputY = userInput.ControllerInputY;
		inputR = userInput.ControllerInputReverse;
		inputH = userInput.ControllerInputHandBrake;
	}
	Debug.Log($"X: {inputX} // Y: {inputY} // R {inputR} // H {inputH}");

	// calculate vehicle velocity in the forward direction
	vel = transform.InverseTransformDirection(rB.velocity).z;

	// update aerodynamic drag and lift
	// aeroDynamics.ApplyAeroDrag(vel);
	// aeroDynamics.ApplyAeroLift(vel);

	// update steering angle due to input, correct for ackermann and apply steering (if we have a steering script)
	steerAngle = steering.SteerAngle(vel, inputX, steerAngle);
	//wC[2].steerAngle = steering.AckerAdjusted(steerAngle, suspension.GetWheelBase, suspension.GetTrackFront, true);
	//wC[3].steerAngle = steering.AckerAdjusted(steerAngle, suspension.GetWheelBase, suspension.GetTrackFront, false);


	// update lateral load transfer from anti-roll bars (sway bars)
	suspension.ApplyLLT();

	// if automatic, select gear appropriate for vehicle speed (unless reverse requested)
	if (transmission.GetAutomatic)
	{
		if (inputY < 0f) transmission.SelectReverse();
		else transmission.SetGear(suspension.GetNoSlipWheelRPM(vel), engine.GetEngineRPMMaxPower);
	}
	float original_inputY = 1*inputY;

	if (inputY == 0)
	{
		inputY = -1;
	}
	else { inputY = Math.Abs(inputY); }

	// update engine rpm and available torque
	float transmissionRatio = transmission.GetTransmissionRatio();
	float engineClutchLockRPM = transmission.GetEngineClutchLockRPM;
	engine.UpdateEngineSpeedRPM(suspension.GetNoSlipWheelRPM(vel), (inputY), transmissionRatio, engineClutchLockRPM);
	float engineTorque = engine.GetMaxEngineTorque();

	// get requested engine torque

	if (inputX != 0 && original_inputY == 0)
	{
		engine.UpdateEngineSpeedRPM(suspension.GetNoSlipWheelRPM(vel), Math.Abs(inputX), transmissionRatio, engineClutchLockRPM);
		engineTorque = engine.GetMaxEngineTorque();
		engineTorque *= Math.Abs(inputX);
	}
	else if (original_inputY != 0f && Math.Abs(inputY) < 0.2f)
	{
		engineTorque = 0;
	}
	else if (inputY != 0f)
	{
		engineTorque *= (inputY);
	}


	// get requested wheel torques
	float[] wheelTorques = transmission.GetWheelTorques(engineTorque);

	// get traction control torque updates
	// if you want to add a traction control module, this would be a good place to use it

	// get requested brake torques
	float[] brakeTorques = brakes.GetBrakeTorques((inputY));
	if (original_inputY == 0 && inputX != 0)
	{
		brakeTorques = brakes.GetBrakeTorques(Math.Abs(inputX));
	}

	// if handbrake is on, brake rear wheels
	if (inputH > 0.1f) brakes.ApplyHandbrake(brakeTorques);

	// Calculate a wheel rpm limit based on engine limit and transmission
	float wheelRPMLimit = Mathf.Abs(engine.GetEngineRPMMax / transmission.GetTransmissionRatio()) * 1.01f;

	// check if wheel should be hitting engine rpm limit, if so, cut power to prevent over revving of wheel
	int iDrivenWheelID;
	for (int j = 0; j < transmission.GetDrivenWheels.Count; j++)
	{
		iDrivenWheelID = transmission.GetDrivenWheels[j];
		if (wC[transmission.GetDrivenWheels[j]].rpm > wheelRPMLimit) wheelTorques[iDrivenWheelID] = 0.0f;
	}

	// update brake and motor torques on wheel colliders
	// RL RR FL FR
	// initialise all to 0.0f
	if (inputX == 0)
	{
		for (int i = 0; i < 4; i++)
		{
			wC[i].brakeTorque = brakeTorques[i];
			wC[i].motorTorque = wheelTorques[i];
		}
	}
	else if ((inputX < 0f || inputX > 0f) && original_inputY == 0f)
	{
		int dir;
		if (inputX > 0f) { dir = 1; }
		else { dir = -1; }

		for (int i = 0; i < 4; i++)
		{
			int wheel = i % 2;
			if (wheel == 0)
			{
				wC[i].brakeTorque = dir * brakeTorques[i];
				wC[i].motorTorque = dir * wheelTorques[i];
			}
			else
			{
				wC[i].brakeTorque = -dir * brakeTorques[i];
				wC[i].motorTorque = -dir * wheelTorques[i];
			}
		}
	}
	else if ((inputX < 0f || inputX > 0f) && original_inputY != 0f)
	{
		int dir;
		if (inputX > 0f) { dir = 1; }
		else { dir = -1; }

		for (int i = 0; i < 4; i++)
		{
			int wheel = i % 2;
			if (wheel == 0)
			{
				wC[i].brakeTorque = (brakeTorques[i]*1/2) + (dir * brakeTorques[i])/2;
				wC[i].motorTorque = (wheelTorques[i]*1/2) + (dir * wheelTorques[i])/2;
			}
			else
			{
				wC[i].brakeTorque = (brakeTorques[i]*1/2) + (-dir * brakeTorques[i])/2;
				wC[i].motorTorque = (wheelTorques[i] *1/2) + (-dir * wheelTorques[i])/2;
			}
		}
	}

}


void Update()
{
	Transform transWheel;

	// if transmission is manual then check for gear change request
	// Button up/down detection happens at display frequency, so the check is performed here
	if (!transmission.GetAutomatic)
	{
		if (Input.GetButtonDown("GearShiftUp")) transmission.GearShiftUp();
		if (Input.GetButtonDown("GearShiftDown")) transmission.GearShiftDown();
	}

	// update wheel mesh positions to match wheel collider positions
	// slightly convoluted correction for tyre compression which must be corrected locally when WC position update is only available globally

	for (int i = 0; i < 4; i++)
	{
		wC[i].GetWorldPose(out Vector3 wcPosition, out Quaternion wcRotation);


		transWheel = wC[i].gameObject.transform.GetChild(0);
		transWheel.transform.position = wcPosition;


		float offset = 0.4f;
		if (i % 2 == 0)
			transWheel.transform.localPosition = new Vector3(transWheel.transform.localPosition.y- offset, transWheel.transform.localPosition.y, transWheel.transform.localPosition.z);
		else
			transWheel.transform.localPosition = new Vector3(transWheel.transform.localPosition.y+ offset, transWheel.transform.localPosition.y, transWheel.transform.localPosition.z);

		transWheel.transform.rotation = wcRotation;

		// frWheelModel.transform.position = wcPosition;
		// frWheelModel.transform.rotation = wcRotation;
	}
}

}
}