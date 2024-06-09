using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform Target;
    private Transform FollowTarget;
    private Camera MainCam;
    private float MaxVelocity;

    public float MinFOV = 50f;
    public float MaxFOV = 80f;
    public float SmoothingValue = 0.5f;

    private void Start()
    {
        MainCam = transform.Find("Main Camera").GetComponent<Camera>();
        MaxVelocity = Target.GetComponent<CarController>().GearMaxSpeed[Target.GetComponent<CarController>().GearMaxSpeed.Length - 1];
        FollowTarget = Target.Find("CameraFollow");
    }

    private void FixedUpdate()
    {
        transform.LookAt(Target.position);
        transform.position = Vector3.Lerp(transform.position, FollowTarget.position, SmoothingValue);
        float velocity = Target.GetComponent<Rigidbody>().velocity.magnitude;
        MainCam.fieldOfView = Mathf.Lerp(MinFOV, MaxFOV, (velocity / MaxVelocity));
    }

}
