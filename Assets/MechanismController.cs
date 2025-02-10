using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using Empty =  RosMessageTypes.Std.EmptyMsg;
using Unity.Robotics.ROSTCPConnector;

public class MechanismController : MonoBehaviour
{
    // Start is called before the first frame update

    float plantingStartTime;
    public float plantingDuration = 210.0f;
    bool isPlanting = false;
    GameObject warthog_model;
    Animator planting_animator;
    public RuntimeAnimatorController animationController;

    // A prefab to spawn when planting
    public GameObject plantPrefab;
    public GameObject cameraTarget;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<Empty>("/behavior/do_plant", DoPlantCb);

        warthog_model = GameObject.Find("warthog");
    }

    void DoPlantCb(Empty msg)
    {
        Debug.Log("Planting");
        isPlanting = true;

        // Rotate the camera target about the y-axis 180 degrees
        cameraTarget.transform.Rotate(0, 180, 0);

        // Create an Animator on the warthog model
        planting_animator = warthog_model.AddComponent<Animator>();
        planting_animator.runtimeAnimatorController = Instantiate(animationController);


        // Get current time in seconds
        plantingStartTime = Time.time;
    }

    // Update is called once per frame
    void Update()
    {
        if (isPlanting && Time.time - plantingStartTime > plantingDuration)
        {
            // Spawn a GameObject 0.5 meters in front of the robot
            GameObject plant = Instantiate(plantPrefab, transform.position + transform.forward * 1.0f, transform.rotation);
            
            // Scale down by 0.1
            plant.transform.localScale = new Vector3(0.07f, 0.07f, 0.07f);
            // Move it down 0.5 meters
            plant.transform.position = new Vector3(plant.transform.position.x, plant.transform.position.y - 0.5f, plant.transform.position.z);

            isPlanting = false;
            // Rotate the camera target about the y-axis 180 degrees
            cameraTarget.transform.Rotate(0, 180, 0);

            // Remove the planting animator
            Destroy(planting_animator);
        }
    }
}
