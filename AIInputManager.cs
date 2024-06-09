using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AIInputManager : MonoBehaviour
{
    public bool Run = false;
    public float AccelerationInput;
    public float DeAccelerationInput;
    public float SteerInput;

    public float SteerTime = 100f;
    private float SteerTimeCounter = 0;

    public float AccTime = 100f;
    private float AccTimeCounter = 0;

    // Update is called once per frame
    void Update()
    {
        if (Run)
        {
            if (AccTimeCounter > AccTime)
            {
                AccTimeCounter = 0;
                DeAccelerationInput = (Random.Range(0.0f, 1.1f) > 0.99f) ? 1 : 0;
                if(DeAccelerationInput == 0)
                    AccelerationInput = Random.Range(0, 2);
            }
            if (SteerTimeCounter >= SteerTime)
            {
                SteerTimeCounter = 0;
                SteerInput = Random.Range(-1.0f, 1.1f);
            }
            SteerTimeCounter++;
            AccTimeCounter++;
        }
        else
        {
            AccTimeCounter = 0;
            AccelerationInput = 0;
            DeAccelerationInput = 0;
            SteerInput = 0;
        }

    }
}
