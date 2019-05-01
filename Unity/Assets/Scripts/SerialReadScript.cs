using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System.Threading;

public class SerialReadScript : MonoBehaviour
{

    public SerialPort serialPort;
    public string serialPortName = "COM3";
    public GameObject gameObject;
    public float threshold = 0.02f;

    public Vector3 rotationDelta;
    public Quaternion poseFinal;

    

    // Start is called before the first frame update
    void Start()
    {
        serialPort = new SerialPort(serialPortName, 115200);

        if (serialPort.IsOpen == false)
        {
            serialPort.Open();
        }

        poseFinal = gameObject.transform.rotation;
        Thread readThread = new Thread(new ThreadStart(Read));
        readThread.Start();
    }

    // Update is called once per frame
    void Update()
    {
        gameObject.transform.rotation = poseFinal;
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

    Vector3 ParseLine(string line)
    {
        try
        {
            string[] split = line.Split(' ');
            Vector3 vec = new Vector3();
            vec.x = float.Parse(split[0].Replace('.', ','));
            vec.y = float.Parse(split[1].Replace('.', ','));
            vec.z = float.Parse(split[2].Replace('.', ','));
            return vec;
        }
        catch
        {
            return new Vector3();
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

        rotationDelta = ParseLine(reading);
        CaluclateRotation();


    }

    void CaluclateRotation()
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
}
