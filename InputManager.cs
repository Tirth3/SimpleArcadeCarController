using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InputManager : MonoBehaviour
{
  
    [HideInInspector] public int ReverseGear = 1;
    [HideInInspector] public float AccelerationInput;
    [HideInInspector] public float DeAccelerationInput;
    [HideInInspector] public float SteerInput;
    [HideInInspector] public bool Handbrake;
    [HideInInspector] public bool Boost;

    [HideInInspector] public bool GearUp;
    [HideInInspector] public bool GearDown;

    private float ReverseGearCounter = 0;
    private float ReverseGearTimer = 10f;


    // Update is called once per frame
    void Update()
    {
        AccelerationInput = Input.GetKey(KeyCode.W) ? 1 : 0;
        DeAccelerationInput = Input.GetKey(KeyCode.S) ? 1 : 0;
        Handbrake = Input.GetKey(KeyCode.Space) ? true : false;
        SteerInput = Input.GetAxis("Horizontal");
        Boost = Input.GetKey(KeyCode.LeftShift);

        GearUp = Input.GetKeyDown(KeyCode.UpArrow);
        GearDown = Input.GetKeyDown(KeyCode.DownArrow);
        if (Input.GetKey(KeyCode.R) && ReverseGearCounter >= ReverseGearTimer)
        {
            ReverseGearCounter = 0;
            ReverseGear *= -1;
        }
        ReverseGearCounter += 0.1f;
    }
}
