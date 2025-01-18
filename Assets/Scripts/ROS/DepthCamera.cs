using UnityEngine;
// using std_msgs.msg;
using Unity.Collections;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine.Assertions;

namespace ROS2
{
    public class DepthCamera : MonoBehaviour
    {
        // Start is called before the first frame update
        // private ROS2UnityComponent rosUnityComponent;
        // private ROS2Node rosNode;
        // private IPublisher<sensor_msgs.msg.Image> imagePublisher;
        // private IPublisher<sensor_msgs.msg.CameraInfo> cameraInfoPublisher;

        public string nodeName;
        public string cameraName;
        public float hFovDegrees = 101;
        public float vFovDegrees = 68;
        // public float degreesPerPixel = 10;
        public float widthPixels = 672;
        public float heightPixels = 376;


        public new Camera camera;

        WebsocketBridge websocketBridge;

        Rect rect;

        Rigidbody rigidbody;

        float maxRange = 10f; // meters

        public bool showDebug = true;

        public float maxNoiseMultiplier = 1.1f;
        float minNoiseMultiplier;

        // Publisher imagePub;

        void Start()
        {
            websocketBridge = GetComponent<WebsocketBridge>();
            rigidbody = GetComponent<Rigidbody>();

            Assert.IsTrue(maxNoiseMultiplier > 1);
            Assert.IsTrue(maxNoiseMultiplier < 2);

            minNoiseMultiplier = 2 - maxNoiseMultiplier;
        }

        void Update()
        {

            List<Vector3> points = new();

            for (float i = 0; i < widthPixels; i++)
            {
                for (float j = 0; j < heightPixels; j++)
                {
                    float x = i - (widthPixels / 2); // Range is now (-336 px, +336 px)
                    x /= widthPixels / 2; // Range is now (-1, 1)
                    x *= hFovDegrees / 2;
                    float y = j - (heightPixels / 2);
                    y /= heightPixels / 2; // Range is now (-1, 1)
                    y *= vFovDegrees / 2; // Convert to degrees

                    float ray_x = Mathf.Tan(Mathf.Deg2Rad * x);
                    float ray_y = Mathf.Tan(Mathf.Deg2Rad * y);

                    Vector3 ray_direction = new Vector3(ray_x, ray_y, 1f);

                    ray_direction = transform.TransformVector(ray_direction);

                    RaycastHit hit;
                    // Vector3 rayOrigin = rigidbody.position;

                    if (Physics.Raycast(transform.position, ray_direction, out hit, maxRange))
                    {
                        float wheelOffset = hit.distance - maxRange;

                        points.Add(hit.point);

                        if (showDebug)
                        {
                            // Debug.Log($"Ground is {hit.distance} meters below self. Wheel pushed up {wheelOffset}");
                            Debug.DrawLine(transform.position, hit.point, Color.magenta);
                        }
                    }
                }
            }

            // Debug.Log($"Hit {points.Count} points");

            byte[] pointcloudBytes = new byte[points.Count * 6]; // 6 bytes per point

            int byte_idx = 0;

            foreach (Vector3 point in points)
            {

                // CONVERSION FROM UNITY TO ROS COORDINATE SYSTEM:
                // Unity z -> ROS x
                // Unity -x -> ROS y
                // UNITY y -> ROS z

                Vector3 localPoint = transform.InverseTransformPoint(point);


                int x_int = (int)(localPoint.z * 100 * UnityEngine.Random.Range(minNoiseMultiplier, maxNoiseMultiplier)); // Convert from meters to centimeters, then to int

                x_int = Math.Min(x_int, (int)Math.Pow(2, 16)); // Ensure that point will fit in 2 bytes

                byte[] x_bytes = BitConverter.GetBytes((short)x_int);

                foreach (byte b in x_bytes)
                {
                    // bytes_as_string += $"{b}_";
                    pointcloudBytes[byte_idx] = b;
                    // Debug.Log(b);
                    byte_idx++;
                }

                // Y
                int y_int = (int)(localPoint.x * -1 * 100 * UnityEngine.Random.Range(minNoiseMultiplier, maxNoiseMultiplier)); // Convert from meters to centimeters, then to int

                y_int = Math.Min(y_int, (int)Math.Pow(2, 16)); // Ensure that point will fit in 2 bytes

                byte[] y_bytes = BitConverter.GetBytes((short)y_int);

                foreach (byte b in y_bytes)
                {
                    // bytes_as_string += $"{b}_";
                    pointcloudBytes[byte_idx] = b;
                    // Debug.Log(b);
                    byte_idx++;
                }

                // Debug.Log($"{point.y} -> {y_int} -> {bytes.ToHexString()}");

                // Z
                int z_int = (int)(localPoint.y * 100 * UnityEngine.Random.Range(minNoiseMultiplier, maxNoiseMultiplier)); // Convert from meters to centimeters, then to int

                z_int = Math.Min(z_int, (int)Math.Pow(2, 16)); // Ensure that point will fit in 2 bytes

                byte[] z_bytes = BitConverter.GetBytes((short)z_int);

                foreach (byte b in z_bytes)
                {
                    // bytes_as_string += $"{b}_";
                    pointcloudBytes[byte_idx] = b;
                    // Debug.Log(b);
                    byte_idx++;
                }

                // Debug.Log($"{x_int}, {y_int} {z_int} -> {x_bytes.ToHexString()}_{y_bytes.ToHexString()}_{z_bytes.ToHexString()}");
            }

            // Debug.Log($"Sending {points.Count * 6} ({byte_idx + 1}) bytes");
            // Debug.Log(x_int);
            // string bytes_as_string = "";

            // foreach (byte b in pointcloudBytes)
            // {
            //     bytes_as_string += $"{b}_";
            //     // pointcloudBytes[byte_idx] = b;
            //     // byte_idx++;
            // }

            // Debug.Log($"{b} ");
            // bytes_as_string += $"{bytes[1]}_";
            // pointcloudBytes[byte_idx] = bytes[1];

            // Debug.Log($"{bytes_as_string}");
            byte[] kiss_msg = new byte[] { (byte)KISS.MessageType.POINTCLOUD }.Concat(pointcloudBytes).ToArray();
            websocketBridge.SendBytes(kiss_msg);
        }

    }

}  // namespace ROS2