using UnityEngine;

public class SimpleMotionController : MonoBehaviour
{

    private ArticulationBody articulationBody;

    [SerializeField]
    private float forceLimit;
    [SerializeField]
    private float torqueLimit;
    [SerializeField]
    private float forwardKp;
    [SerializeField]
    private float yawKp;
    void Start()
    {
        articulationBody = GetComponent<ArticulationBody>();
    }

    void Update()
    {
        // Get user input from keyboard
        float targetForwardSpeed = Input.GetAxis("Vertical");
        float targetYawRate = Input.GetAxis("Horizontal");

        float currentForwardSpeed = articulationBody.linearVelocity.x * -1;
        float currentYawRate = articulationBody.angularVelocity.y;

        float forwardError = targetForwardSpeed - currentForwardSpeed;
        float yawError = targetYawRate - currentYawRate;

        // Calculate the throttle and steering values
        float forwardForce = forwardError * forwardKp;
        float turnTorque = yawError * yawKp;


        // Push the car forward
        forwardForce = Mathf.Clamp(forwardForce, -forceLimit, forceLimit);
        articulationBody.AddForce(transform.forward * forwardForce);

        // Turn the car
        if (turnTorque != 0)
        {
            articulationBody.AddTorque(transform.up * turnTorque * torqueLimit);
        }

        Debug.Log($"Forward error: {forwardError}, Yaw error: {yawError}");
    }
}
