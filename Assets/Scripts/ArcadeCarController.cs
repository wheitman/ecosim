using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.VisualScripting.FullSerializer;
using UnityEngine;

using RosTwist = RosMessageTypes.Geometry.TwistMsg;

public class ArcadeCarController : MonoBehaviour
{

	[SerializeField] bool showDebug;
	[SerializeField] bool enableManualControl;
	[SerializeField] GameObject[] tires;
	[SerializeField] float wheelRadius;
	[SerializeField] float suspensionDistance;
	[SerializeField] float springDamper;
	[SerializeField] float tireGripFactor;     // 0 to 1. How much the tire resists slip in the side direction.
	[SerializeField] float forwardGripFactor;     // 0 to 1. How much the tire resists slip in the forward direction.
	[SerializeField] float tireMass;
	[SerializeField] float distanceFromWheelToEgoCenter;
	[SerializeField] float mass;
	[SerializeField] float throttleMultiplier;
	[SerializeField] float speedLimit;     // m/s
	[SerializeField] float forceLimit;     // N
	[SerializeField] float torqueLimit;    // Nm
	[SerializeField] float turnKp;
	[SerializeField] float forwardKp;

	float throttle = 0f;
	public float targetForwardSpeed = 0f;
	public float targetTurnSpeed = 0f;
	float turn = 0f;


	float springStrength;

	private List<Vector3> netForces;

	private ArticulationBody articulationBody;

	WebsocketBridge bridge;

	void Start()
	{
		articulationBody = GetComponent<ArticulationBody>();
		articulationBody.mass = mass;

		springStrength = mass * 9.8f / 4.0f / (suspensionDistance / 2);

		netForces = new() { new Vector3(), new Vector3(), new Vector3(), new Vector3() };

		ROSConnection.GetOrCreateInstance().Subscribe<RosTwist>("cmd_vel", (msg) =>
		{
			targetForwardSpeed = (float)msg.linear.x;
			targetTurnSpeed = (float)msg.angular.z;

			Debug.Log($"Received {targetForwardSpeed}, {targetTurnSpeed}");
		});
	}

	void FixedUpdate()
	{

		// if (enableManualControl)
		GetKeyboardInput();

		if (targetForwardSpeed == 0 && targetTurnSpeed == 0)
			return;
		// else
		// 	GetTeleopInput();
		VisualizeWheelAxes();
		// VisualizeRaycasts();
		AddSuspensionForces();
		AddSteeringForces();
		AddThrottle();
		VisualizeNetForces();
	}

	void ResetNetForces()
	{
		netForces = new() { new Vector3(), new Vector3(), new Vector3(), new Vector3() };
	}


	void ApplyNetForces()
	{

		for (int i = 0; i < netForces.Count; i++)
		{
			Vector3 force = netForces[i];

			float magnitude = force.magnitude;


			// Apply force limits
			force /= magnitude;
			magnitude = Mathf.Clamp(magnitude, -forceLimit, forceLimit);
			force *= magnitude;

			if (float.IsNaN(force.x))
				continue;

			GameObject wheel = tires[i];

			articulationBody.AddForceAtPosition(force, wheel.transform.position);

		}
	}

	void VisualizeNetForces(float scale = 0.002f)
	{
		for (int i = 0; i < tires.Length; i++)
		{
			GameObject wheel = tires[i];
			Vector3 force = netForces[i];

			if (showDebug)
				Debug.DrawLine(wheel.transform.position, wheel.transform.position + force * scale, Color.yellow);
		}
	}

	void VisualizeWheelAxes()
	{
		foreach (GameObject tire in tires)
		{
			Vector3 start = tire.transform.position;

			// x axis in red
			Vector3 end = tire.transform.position;
			end.x += 1;
			Debug.DrawLine(start, end, Color.red);

			// y axis in green
			end = tire.transform.position;
			end.y += 1;
			Debug.DrawLine(start, end, Color.green);

			// z axis in blue
			end = tire.transform.position;
			end.z += 1;
			Debug.DrawLine(start, end, Color.blue);
		}
	}

	private void AddSuspensionForces()
	{
		for (int i = 0; i < tires.Length; i++)
		{
			GameObject tire = tires[i];

			// Cast raypoint down from wheel
			RaycastHit hit;
			Vector3 rayOrigin = tire.transform.position;
			Vector3 rayDirection = -tire.transform.up;
			bool hitGround = Physics.Raycast(rayOrigin, rayDirection, out hit, wheelRadius + suspensionDistance);

			if (!hitGround)
				continue;

			Vector3 springDir = tire.transform.up;
			Vector3 tireWorldVel = articulationBody.GetPointVelocity(tire.transform.position);
			float offset = suspensionDistance - hit.distance;
			float springVelocity = Vector3.Dot(springDir, tireWorldVel);
			float springForce = springStrength * offset - springDamper * springVelocity;

			articulationBody.AddForceAtPosition(springDir * springForce, tire.transform.position);

			if (showDebug)
				Debug.DrawLine(tire.transform.position, tire.transform.position + springDir * springForce, Color.blue);
		}
	}

	private void AddSteeringForces()
	{

		// Add torque to reach desired turn speed
		float currentTurnSpeed = articulationBody.angularVelocity.y;
		float turnError = targetTurnSpeed - currentTurnSpeed;
		// Calculate desired torque from turn error, current torque
		float desiredTorque = turnError * articulationBody.mass * turnKp;

		if (Mathf.Abs(desiredTorque) > torqueLimit)
			Debug.LogWarning($"Desired torque {desiredTorque} exceeds limit {torqueLimit}");

		desiredTorque = Mathf.Clamp(desiredTorque, -torqueLimit, torqueLimit);
		articulationBody.AddTorque(Vector3.up * desiredTorque);

		Debug.Log($"Turn error: {turnError}, Desired torque: {desiredTorque}");

		for (int i = 0; i < tires.Length; i++)
		{
			GameObject tire = tires[i];

			// Cast raypoint down from wheel
			RaycastHit hit;
			Vector3 rayOrigin = tire.transform.position;
			Vector3 rayDirection = -tire.transform.up;
			bool hitGround = Physics.Raycast(rayOrigin, rayDirection, out hit, wheelRadius + suspensionDistance);

			if (!hitGround)
				continue;

			Vector3 steeringDir = tire.transform.right;
			Vector3 tireWorldVel = articulationBody.GetPointVelocity(tire.transform.position);
			float steeringSpeed = Vector3.Dot(steeringDir, tireWorldVel);
			float desiredSpeedChange = -steeringSpeed * tireGripFactor;
			float desiredAccel = desiredSpeedChange / Time.fixedDeltaTime;

			articulationBody.AddForceAtPosition(steeringDir * tireMass * desiredAccel, tire.transform.position);

		}
	}

	private void GetKeyboardInput()
	{
		throttle = Input.GetAxis("Vertical");
		turn = Input.GetAxis("Horizontal");
		targetForwardSpeed = throttle;
		targetTurnSpeed = turn;
	}

	private void AddThrottle()
	{
		for (int i = 0; i < tires.Length; i++)
		{
			GameObject tire = tires[i];

			// Cast raypoint down from wheel
			RaycastHit hit;
			Vector3 rayOrigin = tire.transform.position;
			Vector3 rayDirection = -tire.transform.up;
			bool hitGround = Physics.Raycast(rayOrigin, rayDirection, out hit, wheelRadius + suspensionDistance);

			if (!hitGround)
				continue;

			// Apply force to the articulation body at the tire's position to make the car's forward speed zero
			Vector3 forwardDir = tire.transform.forward;
			Vector3 tireWorldVel = articulationBody.GetPointVelocity(tire.transform.position);
			float forwardSpeed = Vector3.Dot(forwardDir, tireWorldVel);
			float speedError = targetForwardSpeed - forwardSpeed;
			Debug.Log($"Speed error: {speedError}");
			float forwardForce = tireMass * speedError * forwardKp;

			if (Mathf.Abs(forwardForce) > forceLimit)
				Debug.LogWarning($"Desired torque {forwardForce} exceeds limit {forceLimit}");

			forwardForce = Mathf.Clamp(forwardForce, -forceLimit, forceLimit);

			// Apply force to the articulation body at the tire's position
			articulationBody.AddForceAtPosition(forwardDir * forwardForce, tire.transform.position);

		}
	}

	// Update is called once per frame
	void Update()
	{

	}
}
