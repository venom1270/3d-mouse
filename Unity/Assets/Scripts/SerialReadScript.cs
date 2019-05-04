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

    public Quaternion poseFinal;
    public Vector3 positionFinal;

    // Start is called before the first frame update
    void Start()
    {
        serialPort = new SerialPort(serialPortName, 115200);

        if (serialPort.IsOpen == false)
        {
            serialPort.Open();
        }

        poseFinal = OBJECT.transform.rotation;
        positionFinal = OBJECT.transform.localPosition;
        Thread readThread = new Thread(new ThreadStart(Read));
        readThread.Start();
    }

    // Update is called once per frame
    void Update()
    {
        OBJECT.transform.rotation = poseFinal;
        OBJECT.transform.localPosition = positionFinal;
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

            return new Vector3[] { vec, pos };
        }
        catch
        {
            return new Vector3[] { new Vector3(), new Vector3() };
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
        if (Mathf.Abs(positionDelta.x) < positionTHR) positionDelta.x = 0;
        if (Mathf.Abs(positionDelta.y) < positionTHR) positionDelta.y = 0;
        if (Mathf.Abs(positionDelta.z) < positionTHR) positionDelta.z = 0;
        
        positionFinal = positionFinal + positionDelta;
    }
}
