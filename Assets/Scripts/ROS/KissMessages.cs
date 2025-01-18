using System;
using UnityEngine;

namespace KISS
{
    public enum MessageType : byte
    {
        IMAGE = 0x00,
        SUBSCRIBE = 0x01,
        TELEOP = 0x02,
        POINTCLOUD = 0x03,
        GNSS_FIX = 0x04
    }
}