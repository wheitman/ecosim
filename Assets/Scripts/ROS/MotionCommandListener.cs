using UnityEngine;
using System;
// using geometry_msgs.msg;
// using sensor_msgs.msg;

namespace ROS2
{
	/// <summary>
	/// The main input to this class is Twist messages received by a ROS subscriber.
	/// The output is simulated user control commands, with are read by our UnityCar scripts.
	/// Recall that UnityCar listens for input through Unity's built-in user input system,
	/// which in turn checks for keyboard presses, gamepads, etc. This script basically
	/// acts as a virtual gamepad/keyboard, thanks to its ControllerInputX/Y getters.
	/// </summary>
	public class MotionCommandListener : MonoBehaviour
	{

		// Start is called before the first frame update
		// private ROS2UnityComponent rosUnityComponent;
		// private ROS2Node rosNode;
		// private ISubscription<Twist> commandVelSub;
		private float current_forward_speed;
		private float current_yaw_rate;

		// Modeled from UserInput.cs
		private float controllerInputX;
		private float controllerInputY;
		private float controllerInputReverse;
		private float controllerInputHandBrake;
		private float linear_twist_x;
		private float angular_twist_z;

		// ROS-specific params
		public string nodeName;
		public string cmdTopic;

		// Other params
		public float speedLimit = 1.0f;

		public float ControllerInputX
		{
			get
			{
				// BANG BANG BABY
				if (controllerInputX > 0.3f)
					return 1f;
				else if (controllerInputX < -0.3f)
					return -1f;
				else if (controllerInputX > 0.1f)
					return 0.5f;
				else if (controllerInputX < -0.1f)
					return -.5f;
				else return 0f;
			}
		}

		public float ControllerInputY
		{
			get
			{
				if (controllerInputY > 0.3f)
					return .5f;
				else if (controllerInputY < -0.3f)
					return -.5f;
				else if (controllerInputY > 0.1f)
					return 0.3f;
				else if (controllerInputY < -0.1f)
					return -.3f;
				else return 0f;
			}
		}

		public float ControllerInputReverse
		{
			get { return 0f; }
		}

		public float ControllerInputHandBrake
		{
			get { return controllerInputHandBrake; }
		}

		void Start()
		{
			// rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();
		}

		void Update()
		{
			// if (rosUnityComponent.Ok())
			// {
			// 	if (rosNode == null)
			// 	{
			// 		// Set up the node and subscriber.
			// 		rosNode = rosUnityComponent.CreateNode(nodeName);

			// 		commandVelSub = rosNode.CreateSubscription<Twist>(
			// 			"/cmd_vel", commandVelCb);
			// 	}
			// }

			// Unity requires us to get our velocity in the main thread-- that's here!
			// Unity "z" is forward-- ROS's "x". Unity "y" is up-- ROS's "z"
			current_forward_speed = GetComponent<Rigidbody>().velocity.magnitude;

			// Negate to make it right-handed
			current_yaw_rate = -1 * GetComponent<Rigidbody>().angularVelocity.y;

			// Unity also requires us to set motor torques in the main thread.
			setControllerInputs();
		}

		/// <summary>
		/// Calculate the current error in linear and angular speed, then apply
		/// appropriate simulated user input to minimize the errors.
		/// </summary>
		void setControllerInputs()
		{
			float linear_speed_error = linear_twist_x - current_forward_speed;
			float yaw_rate_error = angular_twist_z - current_yaw_rate;
			float Kp_linear = 4.0f;
			float Kp_angular = -8.0f;
			float max_input_x = 5.0f;

			// Debug.Log($"Speed: {current_forward_speed}");
			if (current_forward_speed > speedLimit)
			{
				// Debug.Log("Coasting!");
				controllerInputY = 0f;
			}
			else
			{
				controllerInputY = Math.Max(Math.Min(Kp_linear * linear_speed_error, 1.0f), 0);
			}

			controllerInputX = Math.Max(
				Math.Min(
					Kp_angular * yaw_rate_error,
					max_input_x), -max_input_x);
		}

		/// <summary>
		/// Sets our target speeds from a given Twist message.
		/// </summary>
		/// <param name="msg">Twist message from subscriber</param>
		// void commandVelCb(Twist msg)
		// {
		// 	linear_twist_x = (float)msg.Linear.X;
		// 	angular_twist_z = (float)msg.Angular.Z;
		// }
	}
}  // namespace ROS2
