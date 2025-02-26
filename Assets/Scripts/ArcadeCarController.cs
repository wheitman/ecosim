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
	[SerializeField] GameObject[] rayPoints;
	[SerializeField] GameObject[] wheels;
	[SerializeField] float wheelRadius;
	[SerializeField] float suspensionDistance;
	[SerializeField] float springFactor;     // 0 to 1
	[SerializeField] float springDamper;
	[SerializeField] float sideGridFactor;     // 0 to 1. How much the tire resists slip in the side direction.
	[SerializeField] float forwardGripFactor;     // 0 to 1. How much the tire resists slip in the forward direction.
	[SerializeField] float tireMass;
	[SerializeField] float distanceFromWheelToEgoCenter;
	[SerializeField] float mass;
	[SerializeField] float throttleMultiplier;
	[SerializeField] float turnMultiplier;
	[SerializeField] float speedLimit;     // m/s
	[SerializeField] float turnSpeedLimit;
	[SerializeField] float forceLimit;     // N

	float throttle = 0f;
	public float targetForwardSpeed = 0f;
	public float targetTurnSpeed = 0f;
	float turn = 0f;


	float springStrength;

	private List<Vector3> netForces;

	new Rigidbody rigidbody;
	WebsocketBridge bridge;

	// Start is called before the first frame update
	void Start()
	{
		rigidbody = GetComponent<Rigidbody>();
		rigidbody.mass = mass;

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
		ResetNetForces();

		// if (enableManualControl)
		// 	GetKeyboardInput();
		// else
		// 	GetTeleopInput();
		// VisualizeWheelAxes();
		// VisualizeRaycasts();
		// AddSuspensionForces();
		AddSteeringForces();
		// AddThrottle();
		ApplyNetForces();
		VisualizeNetForces();
	}

	void ResetNetForces()
	{
		netForces = new() { new Vector3(), new Vector3(), new Vector3(), new Vector3() };
	}


	void ApplyNetForces()
	{
		// if (Math.Abs(targetForwardSpeed) < 0.1) {
		// 	targetForwardSpeed = 0;
		// }
		// if (Math.Abs(targetTurnSpeed) < 0.1) {
		// 	targetTurnSpeed = 0;
		// }

		float throttle = Input.GetAxis("Vertical");
		float turn = Input.GetAxis("Horizontal");

		float speedError = targetForwardSpeed - Vector3.Dot(transform.forward, rigidbody.linearVelocity);
		float turnError = targetTurnSpeed - rigidbody.angularVelocity.y;

		// Cap the turn error to the turn speed limit
		turnError = Math.Min(turnError, turnSpeedLimit);
		turnError = Math.Max(turnError, -turnSpeedLimit);

		// rigidbody.AddForce(transform.forward * 12000 * speedError);

		rigidbody.linearVelocity = transform.forward * targetForwardSpeed;
		rigidbody.angularVelocity = new Vector3(0f, -targetTurnSpeed, 0f);

		// for (int i = 0; i < netForces.Count; i++)
		// 	{
		// 		Vector3 force = netForces[i];

		// 		float magnitude = force.magnitude;


		// 		// Apply force limits
		// 		force /= magnitude;
		// 		magnitude = Math.Min(forceLimit, magnitude);
		// 		magnitude = Math.Max(-forceLimit, magnitude);
		// 		force *= magnitude;

		// 		if (float.IsNaN(force.x))
		// 			continue;

		// 		if (Math.Abs(targetForwardSpeed) < 0.1 && Math.Abs(targetTurnSpeed) < 0.1)
		// 		{
		// 			force = new Vector3(0, 0, 0);
		// 			rigidbody.isKinematic = true;
		// 			// Debug.Log("Car is stopped. Making kinematic.");
		// 		}
		// 		else
		// 		{
		// 			rigidbody.isKinematic = false;
		// 		}

		// 		GameObject wheel = rayPoints[i];

		// 		rigidbody.AddForceAtPosition(force, wheel.transform.position);

		// 	}
	}

	void VisualizeNetForces(float scale = 0.002f)
	{
		for (int i = 0; i < rayPoints.Length; i++)
		{
			GameObject wheel = rayPoints[i];
			Vector3 force = netForces[i];

			if (showDebug)
				Debug.DrawLine(wheel.transform.position, wheel.transform.position + force * scale, Color.yellow);
		}
	}

	void VisualizeWheelAxes()
	{
		foreach (GameObject wheel in rayPoints)
		{
			Vector3 start = wheel.transform.position;

			// x axis in red
			Vector3 end = wheel.transform.position;
			end.x += 1;
			Debug.DrawLine(start, end, Color.red);

			// y axis in green
			end = wheel.transform.position;
			end.y += 1;
			Debug.DrawLine(start, end, Color.green);

			// z axis in blue
			end = wheel.transform.position;
			end.z += 1;
			Debug.DrawLine(start, end, Color.blue);
		}
	}

	private void AddSuspensionForces()
	{
		for (int i = 0; i < rayPoints.Length; i++)
		{

			GameObject rayOrigin = rayPoints[i];
			GameObject wheel = wheels[i];

			// Shoot a ray down

			RaycastHit hit;

			float wheelRestDistance = wheelRadius + suspensionDistance;

			if (Physics.Raycast(rayOrigin.transform.position, -Vector3.up, out hit, wheelRestDistance))
			{
				float wheelOffset = hit.distance - wheelRestDistance;

				if (showDebug)
				{
					// Debug.Log($"Ground is {hit.distance} meters below {rayOrigin.name}. Wheel pushed up {wheelOffset}");
					Debug.DrawLine(rayOrigin.transform.position, hit.point, Color.magenta);
				}

				Vector3 wheelWorldVel = rigidbody.GetPointVelocity(rayOrigin.transform.position);

				float offset = wheelRestDistance - hit.distance;
				float vel = Vector3.Dot(rayOrigin.transform.up, wheelWorldVel);

				float force = (offset * springStrength) - (vel * springDamper);

				netForces[i] += rayOrigin.transform.up * force;

				Vector3 wheelPosition = hit.point;
				wheelPosition.y += wheelRadius;
				wheel.transform.position = wheelPosition;
			}
		}
	}

	private void AddSteeringForces()
	{
		for (int i = 0; i < rayPoints.Length; i++)
		{
			GameObject wheel = rayPoints[i];

			Vector3 wheelWorldVel = rigidbody.GetPointVelocity(wheel.transform.position);

			// Direction we don't want to wheel to roll in.
			Vector3 slipDirection = wheel.transform.right;

			float slipVel = Vector3.Dot(slipDirection, wheelWorldVel);

			float desiredVelChange = -slipVel * sideGridFactor;

			float desiredAcceleration = desiredVelChange / Time.fixedDeltaTime;

			netForces[i] += slipDirection * tireMass * desiredAcceleration;
		}
	}

	private void GetKeyboardInput()
	{
		throttle = Input.GetAxis("Vertical");
		turn = Input.GetAxis("Horizontal");
		targetForwardSpeed = throttle;
		targetTurnSpeed = turn;
	}

	private void GetTeleopInput()
	{
		if (throttle > 1)
		{
			Debug.LogWarning($"Throttle of {throttle} was greater than 1. Ignoring");
			throttle = 0f;
		}
		else if (throttle < -1)
		{
			Debug.LogWarning($"Throttle of {throttle} was smaller than -1. Ignoring.");
			throttle = 0f;
		}
		targetForwardSpeed = bridge.throttle;
		targetTurnSpeed = bridge.turn;
		// Debug.Log($"({bridge.throttle}, {bridge.turn})");
	}

	private void AddThrottle()
	{
		// float throttle = Input.GetAxis("Vertical");
		// float turn = Input.GetAxis("Horizontal");

		float carSpeed = Vector3.Dot(transform.forward, rigidbody.linearVelocity);

		// Bang bang baby!
		if (carSpeed < targetForwardSpeed - 0.1f)
		{
			throttle = 1f;
		}
		else if (carSpeed > targetForwardSpeed + 0.1f)
		{
			throttle = -1f;
		}

		if (carSpeed > speedLimit && throttle >= 0f)
		{
			if (showDebug)
				Debug.Log("Car at speed limit. Coasting.");
		}
		else
		{
			for (int i = 0; i < rayPoints.Length; i++)
			{
				GameObject wheel = rayPoints[i];

				Vector3 accelDir = wheel.transform.forward;
				Vector3 wheelWorldVel = rigidbody.GetPointVelocity(wheel.transform.position);

				float wheelForwardSpeed = Vector3.Dot(accelDir, wheelWorldVel);

				// Apply brakes
				if (Math.Abs(throttle) < 0.1f && Math.Abs(turn) < 0.1)
				{
					float desiredVelChange;
					if (rigidbody.linearVelocity.magnitude > .1)
					{
						forwardGripFactor -= 0.1f;
					}
					else
					{
						forwardGripFactor += 0.1f;
					}
					forwardGripFactor = Math.Min(0f, forwardGripFactor);
					forwardGripFactor = Math.Max(5f, forwardGripFactor);
					desiredVelChange = -wheelForwardSpeed * forwardGripFactor;
					float desiredSlipAcceleration = desiredVelChange / Time.fixedDeltaTime;
					netForces[i] += accelDir * tireMass * desiredSlipAcceleration;
				}



				netForces[i] += accelDir * throttle * throttleMultiplier;

			}
		}

		float current_turn = rigidbody.angularVelocity.y;
		if (Mathf.Abs(targetTurnSpeed) < 0.01f)
		{
			// Do nothing
		}
		else if (current_turn < targetTurnSpeed)
		{
			rigidbody.AddTorque(transform.up * 1f * turnMultiplier);
		}
		else if (current_turn > targetTurnSpeed)
		{
			rigidbody.AddTorque(transform.up * -1f * turnMultiplier);
		}

		// Debug.Log($"Lin: {targetForwardSpeed - carSpeed}, ang: {targetTurnSpeed - current_turn}");

		// Spin the wheels visually
		foreach (GameObject wheel in wheels)
		{
			float wheel_angular_vel = (carSpeed + distanceFromWheelToEgoCenter * current_turn) / wheelRadius;
			wheel.transform.Rotate(new Vector3(0f, wheel_angular_vel));
		}
	}

	// Update is called once per frame
	void Update()
	{

	}
}
