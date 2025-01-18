using System;
using UnityEngine;

namespace ROS2
{
    [Serializable]
    public abstract class RosMessage
    {

    }

    public class TimeMessage
    {
        int sec;
        int nsec;
    }

    public class Header
    {
        int seq;
        Time time;
        string frame_id;
    }

    // https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
    [Serializable]
    public class ImageMessage : RosMessage
    {
        public Header header;
        public uint height, width;
        public string encoding;
        public uint is_bigendian;
        public uint step;
        public byte[] data;
    }
    [Serializable]
    public class TwistMsg : RosMessage
    {
        public Vector3 linear;
        public Vector3 angular;
    }
}
