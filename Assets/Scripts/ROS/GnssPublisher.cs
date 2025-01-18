using UnityEngine;
using System.IO;
using System;
// using std_msgs.msg;
// using geometry_msgs.msg;
// using sensor_msgs.msg;
using System.Linq;
using System.Threading.Tasks;
using Unity.Collections;
using System.Security.Cryptography;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine.Assertions;


namespace ROS2
{
	public class GnssPublisher : MonoBehaviour
	{
		// Start is called before the first frame update
		// private ROS2UnityComponent rosUnityComponent;
		// private ROS2Node rosNode;
		// private IPublisher<PoseWithCovarianceStamped> posePublisher;
		// private IPublisher<NavSatFix> navSatFixPublisher;
		// TODO: Transform broadcaster

		// PoseWithCovarianceStamped currentPose;
		// NavSatFix currentFix;
		UnityEngine.Transform mapOrigin;


		public float maxNoiseCm = 10f;
		WebsocketBridge websocketBridge;

		// YES this is hardcoded.
		// NO I don't have time to change it.
		// WSH Oct 22 '24
		float topLeftLat = 40.44823504f;
		float topLeftLon = -79.95278458f;
		float topLeftEastM = 0f;
		float topLeftNorthM = 1177f;
		float botRightLat = 40.43743552f;
		float botRightLon = -79.93809829f;
		float botRightEastM = 1177f;
		float botRightNorthM = 0f;


		void Start()
		{
			websocketBridge = GetComponent<WebsocketBridge>();
		}

		void Update()
		{
			// 1. Get ego latitude and longitude as floats
			float egoEastM = transform.position.x;
			float egoNorthtM = transform.position.z;
			float egoAltM = transform.position.y;

			float latDelta = topLeftLat - botRightLat;
			float lonDelta = botRightLon - topLeftLon; // Positive, ~0.01469
			float eastMDelta = botRightEastM - topLeftEastM;
			float northMDelta = topLeftNorthM - botRightNorthM;

			float eastFraction = (egoEastM - topLeftEastM) / eastMDelta;
			float northFraction = (egoNorthtM - botRightNorthM) / northMDelta;

			// Debug.Log($"{eastFraction}, {northFraction}");

			float egoLon = topLeftLon + lonDelta * eastFraction;
			float egoLat = botRightLat + latDelta * northFraction;

			float yaw = 2 * Mathf.PI - (transform.rotation.eulerAngles[1] / 180f * Mathf.PI);

			// Set yaw so that 0 points East per ENU conventions
			yaw += Mathf.PI / 2;

			if (yaw > 2 * Mathf.PI)
				yaw -= 2 * Mathf.PI;

			// 2. Convert to bytes and store in an array

			byte[] fixBytes = new byte[17]; // first byte is dtype, plus 4 bytes per float, (lat, lon, alt, heading)

			byte[] latBytes = BitConverter.GetBytes(egoLat);
			byte[] lonBytes = BitConverter.GetBytes(egoLon);
			byte[] altBytes = BitConverter.GetBytes(egoAltM);
			byte[] yawBytes = BitConverter.GetBytes(yaw);

			latBytes.CopyTo(fixBytes, 1);
			lonBytes.CopyTo(fixBytes, 5);
			altBytes.CopyTo(fixBytes, 9);
			yawBytes.CopyTo(fixBytes, 13);

			fixBytes[0] = (byte)KISS.MessageType.GNSS_FIX;

			string bytes_as_string = "";

			foreach (byte b in fixBytes)
			{
				bytes_as_string += $"{b}_";
			}

			// Debug.Log($"{bytes_as_string}");

			websocketBridge.SendBytes(fixBytes);
		}
	}

}  // namespace ROS2