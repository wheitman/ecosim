using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using NativeWebSocket;
using ROS2;

public class WebsocketBridge : MonoBehaviour
{
    WebSocket websocket;
    int port = 8765;

    public float throttle = 0f;
    public float turn = 0f;

    // Start is called before the first frame update
    void Start()
    {
        websocket = new WebSocket($"ws://localhost:{port}");

        websocket.OnOpen += () =>
        {
            Debug.Log("Connection open!");
            // Subscribe to teleop commands
            Subscribe(KISS.MessageType.TELEOP);
        };

        websocket.OnError += (e) =>
        {
            Debug.Log("Error! " + e);
        };

        websocket.OnClose += (e) =>
        {
            Debug.Log("Connection closed!");
        };

        websocket.OnMessage += (bytes) =>
        {
            // Debug.Log("OnMessage!");
            if (bytes[0] == (byte)KISS.MessageType.TELEOP)
            {
                throttle = ((float)bytes[1]) / 128 - 1;
                throttle *= 10; // Byte is interally scaled by ten. Re-scale.
                turn = ((float)bytes[2]) / 128 - 1;
                turn *= -1;
            }

            // getting the message as a string
            // var message = System.Text.Encoding.UTF8.GetString(bytes);
            // Debug.Log("OnMessage! " + message);
        };

        InvokeRepeating("TryConnection", 0.0f, 1.0f);

        // waiting for messages
        TryConnection();
        // Debug.Log($"Trying to connect to port {port}");
        // await websocket.Connect();

    }

    void Update()
    {
#if !UNITY_WEBGL || UNITY_EDITOR
        websocket.DispatchMessageQueue();
#endif
    }

    async void TryConnection()
    {
        if (websocket.State == WebSocketState.Closed)
        {
            Debug.Log($"Trying to connect to port {port}");
            await websocket.Connect();


        }
    }

    async void SendWebSocketMessage()
    {
        if (websocket.State == WebSocketState.Open)
        {
            // Sending bytes
            await websocket.Send(new byte[] { 10, 20, 30 });

            // Sending plain text
            await websocket.SendText("plain text message");
        }
    }

    public void Subscribe(KISS.MessageType type)
    {
        if (websocket.State != WebSocketState.Open)
        {
            Debug.LogWarning("Could not request subscription. Connection not open.");
            return;
        }
        SendBytes(new byte[] { (byte)KISS.MessageType.SUBSCRIBE, (byte)type });
    }

    public async void SendBytes(byte[] msg)
    {
        if (websocket.State == WebSocketState.Open)
        {
            // Sending bytes
            await websocket.Send(msg);

            // Sending plain text
            // await websocket.SendText("plain text message");
        }
        else
        {
            Debug.LogWarning("Could not send bytes. Connection not open.");
        }
    }

    private async void OnApplicationQuit()
    {
        await websocket.Close();
    }

}