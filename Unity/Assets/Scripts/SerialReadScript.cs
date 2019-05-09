using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System.Threading;
using System;

public class SerialReadScript : MonoBehaviour
{

    public SerialPort serialPort;
    public string serialPortName = "COM3";
    public GameObject OBJECT;
    public float threshold = 0.02f;
    public float positionTHR = 0.00005f;
    
    public Vector3 rotationDelta;
    public Vector3 positionDelta;

    public float positionFactor = 0.01f;

    public Quaternion poseFinal;
    public Vector3 positionFinal;

    public float deltaTime = 0;
    public Vector3 speed;

    Vector3Int speedHistoryCounter;
    public int speedHistoryThreshold;

    public Vector3 gravitationalOffset;
    float gravitationalForceZ;
    Vector3[] rot;

    Color renderColor;

    Color colorRed;
    Color colorDefault;

    // Start is called before the first frame update
    void Start()
    {
        serialPort = new SerialPort(serialPortName, 115200);

        if (serialPort.IsOpen == false)
        {
            serialPort.Open();
        }

        poseFinal = OBJECT.transform.rotation;
        speed = new Vector3();
        speedHistoryCounter = new Vector3Int();
        positionFinal = OBJECT.transform.localPosition;
        rot = new Vector3[3] { new Vector3(), new Vector3(), new Vector3() };
        Thread readThread = new Thread(new ThreadStart(Read));
        colorRed = new Color(255, 0, 0);
        colorDefault = this.GetComponent<Renderer>().material.color;
        renderColor = colorDefault;
        readThread.Start();
    }

    // Update is called once per frame
    void Update()
    {
        // TEST: Fix X and Y angles to 0 if Z axis has maximum gravitational force -> tablet is on the table
        if (gravitationalForceZ > 11 && speedHistoryCounter.z > 350)
        {
            poseFinal = transform.rotation = Quaternion.Euler(0, 0, poseFinal.eulerAngles.z);
        }

        OBJECT.transform.rotation = poseFinal;
        OBJECT.transform.localPosition = positionFinal;
        this.GetComponent<Renderer>().material.color = renderColor;
        rot[0] = OBJECT.transform.up;
        rot[1] = OBJECT.transform.right;
        rot[2] = OBJECT.transform.forward;
    }

    void Read()
    {
        while (true)
        {
            string reading;
            if ((reading = serialPort.ReadLine()) != null)
            {
                Parse(reading);
            }
        }

    }

    float GetAngle(string reading, int place=2)
    {
        try
        { 
            string[] split = reading.Split(' ');
            return float.Parse(split[place].Replace('.', ','));
        }
        catch
        {
            return 0.0f;
        }
    }

    Vector3[] ParseLine(string line)
    {
        try
        {
            string[] split = line.Replace('.',',')
                                 .Split(' ');
            
            Vector3 vec = new Vector3
            {
                x = float.Parse(split[0]),
                y = float.Parse(split[1]),
                z = float.Parse(split[2])
            };

            Vector3 pos = new Vector3
            {
                x = float.Parse(split[3]),
                y = float.Parse(split[4]),
                z = float.Parse(split[5])
            };

            Vector3 dt = new Vector3
            {
                x = int.Parse(split[6]) / 1000.0f,
                y = float.Parse(split[7])
            };

            return new Vector3[] { vec, pos, dt };
        }
        catch
        {
            return new Vector3[] { new Vector3(), new Vector3(), new Vector3() };
        }
    }

    void Parse(string reading)
    {
        /*
        if (reading.StartsWith("ROTATION X:"))
        {
            float angle = GetAngle(reading);
            rotation.x = -angle;
        }
        else if (reading.StartsWith("ROTATION Y:"))
        {
            float angle = GetAngle(reading);
            rotation.z = -angle;
        }
        else if (reading.StartsWith("ROTATION Z:"))
        {
            float angle = GetAngle(reading);
            rotation.y = -angle;
        }
        */

        /*if (reading.StartsWith("ROTATION X DELTA:"))
        {
            float angle = GetAngle(reading, 3);
            rotationDelta.x = -angle;
        }
        else if (reading.StartsWith("ROTATION Y DELTA:"))
        {
            float angle = GetAngle(reading, 3);
            rotationDelta.z = -angle;
        }
        else if (reading.StartsWith("ROTATION Z DELTA:"))
        {
            float angle = GetAngle(reading, 3);
            rotationDelta.y = -angle;

            // Z is last so we can calculate rotation...
            CaluclateRotation();
        }*/

        Vector3[] tempHolder = ParseLine(reading);
        rotationDelta = tempHolder[0];
        positionDelta = tempHolder[1];
        deltaTime = tempHolder[2].x;
        gravitationalForceZ = tempHolder[2].y;


        CalculateRotation();
        CalculatePosition();
        
    }
    
    void CalculateRotation()
    {

        if (Mathf.Abs(rotationDelta.x) < threshold) rotationDelta.x = 0;
        if (Mathf.Abs(rotationDelta.y) < threshold) rotationDelta.y = 0;
        if (Mathf.Abs(rotationDelta.z) < threshold) rotationDelta.z = 0;

        Quaternion poseInitial = poseFinal; // quaternion describing original orientation
        float factor = 3.14f / 180; // degree to radian factor
        Quaternion deltaQuaternion = new Quaternion(
            -0.5f * rotationDelta.x * factor, 
            -0.5f * rotationDelta.y * factor, 
            0.5f * rotationDelta.z * factor, 
            1
        );
        poseFinal = poseInitial * deltaQuaternion;
    }

    void CalculatePosition()
    {
        /*if (Mathf.Abs(positionDelta.x) < positionTHR) positionDelta.x = 0;
        if (Mathf.Abs(positionDelta.y) < positionTHR) positionDelta.y = 0;
        if (Mathf.Abs(positionDelta.z) < positionTHR) positionDelta.z = 0;*/

        //positionFinal = positionFinal + positionDelta * positionFactor;
        //if (Mathf.Abs(poseFinal.eulerAngles.x)*3.14/2 < 2 && Mathf.Abs(poseFinal.eulerAngles.y)*3.14/2 < 2)

        CheckGravitationalForce();
        UpdateSpeedCounter();
        UpdateSpeed();

        // change of position - delta
        positionFinal.x += speed.x * deltaTime;
        positionFinal.y += speed.y * deltaTime;
        positionFinal.z += speed.z * deltaTime;

        positionFinal = positionFinal * positionFactor;
        //positionFinal.z = 0;
    }

    void CheckGravitationalForce()
    {
        float G = 11.0f; //9.84f;
        gravitationalOffset.z = gravitationalForceZ;

        // Get global rotation on X and Y axis only
        float rotX = 90 - Vector3.Angle(Vector3.up, rot[2]);
        float rotY = 90 - Vector3.Angle(Vector3.forward, rot[1]);

        // Calculate approximate G force on X any Y axis (proportionally)
        G -= gravitationalOffset.z;
        gravitationalOffset.x = (90 - Math.Abs(90 - rotX)) / 90.0f * G;
        gravitationalOffset.y = (90 - Math.Abs(90 - rotY)) / 90.0f * G;

        gravitationalOffset.z = 0;

        // Not working so well
        //positionDelta -= gravitationalOffset;

        // Stop moving if too much rotation
        if (Math.Abs(rotX) > 5 || Math.Abs(rotY) > 5)
        {
            positionDelta.x = 0;
            positionDelta.y = 0;
            positionDelta.z = 0;
            renderColor = colorRed;
        }
        else
        {
            renderColor = colorDefault;
        }

    }

    void UpdateSpeed()
    {
        if (speedHistoryCounter.x >= speedHistoryThreshold)
        {
            speed.x = 0;
        }
        else
        {
            speed.x += positionDelta.x * deltaTime;
        }
        if (speedHistoryCounter.y >= speedHistoryThreshold)
        {
            speed.y = 0;
        }
        else
        {
            speed.y += positionDelta.y * deltaTime;
        }
        if (speedHistoryCounter.z >= speedHistoryThreshold)
        {
            speed.z = 0;
        }
        else
        {
            speed.z += positionDelta.z * deltaTime;
        }
    }

    void UpdateSpeedCounter()
    {
        // Values are thresholded in C, so we just check for zero
        if (positionDelta.x == 0)
        {
            speedHistoryCounter.x++;
        }
        else
        {
            speedHistoryCounter.x = 0;
        }
        if (positionDelta.y == 0)
        {
            speedHistoryCounter.y++;
        }
        else
        {
            speedHistoryCounter.y = 0;
        }
        if (positionDelta.z == 0)
        {
            speedHistoryCounter.z++;
        }
        else
        {
            speedHistoryCounter.z = 0;
        }
    }
}
