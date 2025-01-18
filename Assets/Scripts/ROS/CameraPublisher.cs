using UnityEngine;
// using std_msgs.msg;
using Unity.Collections;
using System.Linq;

namespace ROS2
{
    public class CameraPublisher : MonoBehaviour
    {
        // Start is called before the first frame update
        // private ROS2UnityComponent rosUnityComponent;
        // private ROS2Node rosNode;
        // private IPublisher<sensor_msgs.msg.Image> imagePublisher;
        // private IPublisher<sensor_msgs.msg.CameraInfo> cameraInfoPublisher;

        public string nodeName;
        public string cameraName;
        public int cameraPixelWidth = 800;
        public int cameraPixelHeight = 600;
        private RenderTexture renderTexture;
        Texture2D screenShot;
        byte[] rawCameraData;


        public new Camera camera;

        WebsocketBridge websocketBridge;

        Rect rect;

        // Publisher imagePub;

        void Start()
        {
            // rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();
            // camera = Camera.main;

            websocketBridge = GetComponent<WebsocketBridge>();

            // // Fix flipped camera (ROS has different vertical axis than Unity)
            Matrix4x4 scale = Matrix4x4.Scale(new Vector3(1, -1, 1));
            camera.projectionMatrix *= scale;

            renderTexture = new RenderTexture(cameraPixelWidth, cameraPixelHeight, 24);

            // 4 bytes per pixel (RGBA)
            rawCameraData = new byte[cameraPixelWidth * cameraPixelHeight * 4];

            screenShot = new Texture2D(cameraPixelWidth, cameraPixelHeight, TextureFormat.RGBA32, false);
            camera.targetTexture = renderTexture;

            // imagePub = websocketBridge.CreatePublisher("/camera_front/image_color", "sensor_msgs/Image");

            // ImageMessage msg = new()
            // {
            //     height = (uint)cameraPixelHeight,
            //     width = (uint)cameraPixelWidth
            // };
            // imagePub.publish(msg);


        }

        /// <summary>
        /// SLOW conversion from NativeArray to byte[].
        /// Likely slow due to GPU memory access, but that's just a theory.
        /// </summary>
        /// <param name="from">NativeArray with pixel data</param>
        void PopulateBytesFromNativeArray(NativeArray<byte> from)
        {
            // var prev = Time.time;
            for (int i = 0; i < from.Length; i++)
            {
                rawCameraData[i] = from[i];
            }
            // var elapsed = Time.time - prev;
            // Debug.Log("Took "+elapsed);
        }

        void Update()
        {
            // if (websocketBridge.directIsReady)
            // {

            //     websocketBridge.sendDirectMessage(rawCameraData);
            //     print($"Published image with width {cameraPixelWidth}");

            //     // image_msg.Data = rawCameraData;
            //     // image_msg.Encoding = "rgba8";
            //     // image_msg.Height = (uint)cameraPixelHeight;
            //     // image_msg.Width = (uint)cameraPixelWidth;
            //     // imagePublisher.Publish(image_msg);
            // }

            // if (websocketBridge.isOpen)
            //     websocketBridge.SendBytes(rawCameraData);


            // Memory leak is between HERE
            // vvvvvvvvvvvvvvvvvvvvvvvvvvv
            GL.invertCulling = true;
            camera.Render();
            GL.invertCulling = false;
            rect = new Rect(0, 0, cameraPixelWidth, cameraPixelHeight);
            RenderTexture.active = renderTexture;
            screenShot.ReadPixels(rect, 0, 0);
            RenderTexture.active = null;
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
            // and HERE

            // Graphics.CopyTexture(renderTexture, screenShot);

            // byte[] imageData = screenShot.EncodeToPNG();
            // var pixels = screenShot.GetPixelData<byte>(0);
            var pixels = screenShot.EncodeToJPG();

            byte[] kiss_msg = new byte[] { (byte)KISS.MessageType.IMAGE }.Concat(pixels).ToArray();
            websocketBridge.SendBytes(kiss_msg);
            // Debug.Log(pixels.Length);
            // PopulateBytesFromNativeArray(pixels);
            // byte[] imageData = screenShot.GetRawTextureData<byte>().ToArray();
            // pixels = null;

        }

    }

}  // namespace ROS2