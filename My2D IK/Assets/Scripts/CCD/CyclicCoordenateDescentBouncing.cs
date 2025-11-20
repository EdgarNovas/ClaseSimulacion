using System;
using UnityEngine;

public class CyclicCoordenateDescendBouncing : MonoBehaviour
{
    public Transform[] joints;
    public Transform endEffector;
    public Transform target;

    public float tolerance = 0.01f;
    public float maxIterations = 1e5f;
    private int countIterations = 0;
    private int index = 2;
    //Rotation Variables
    float angleRotation;
    Vector3 axisRotation;

    //Conexions between Joints

    Vector3[] link;

    //bounce application
    private int bounceVariable;
    private int newIndex;

    int num = 0;

    int numLinks = 0;

    float distance;
    void Start()
    {
        GetLinks();

        distance = Vector3.Magnitude(link[2]);

        bounceVariable = 0;

        newIndex = index - bounceVariable;
       
    }

    // Update is called once per frame
    void Update()
    {

        for(int i = newIndex; i <= index; i++)
        {
            if(countIterations < maxIterations && distance > tolerance)
            {
                //CCD method
                Vector3 currentJoint = joints[i].position;
                Vector3[] referenceVectors;
                referenceVectors = GetVectors(currentJoint);
                angleRotation = getAngleRotation(referenceVectors);
                axisRotation = getAxisRotation(referenceVectors);


                //Forward Kinematics 
                Quaternion rotation = Quaternion.AngleAxis(angleRotation*180 / Mathf.PI,axisRotation);

                UpdatePosition(i, rotation);

                countIterations++;
            }

           


        }

        bounceVariable++;
        if (bounceVariable > 2)
        {
            bounceVariable = 0;
        }
        newIndex = index - bounceVariable;

    }

    private void UpdatePosition(int index, Quaternion q)
    {
        if(index < 2)
        {
            for(int i=index; i < 2; i++)
            {
                joints[i+1].position = joints[i].position + q * link[i];
            }
        }
        endEffector.position = joints[2].position + q * link[2];

        GetLinks();
    }

    private float getAngleRotation(Vector3[] referenceVectors)
    {
        float angle;
        angle = Mathf.Acos(Mathf.Clamp(Vector3.Dot(referenceVectors[0], referenceVectors[1]), -1.0f, 1.0f));
        return angle;

    }

    private Vector3 getAxisRotation(Vector3[] referenceVectors)
    {
        Vector3 axis = Vector3.Cross(referenceVectors[0], referenceVectors[1]);
        axis.Normalize();
        return axis;
    }

    private void GetLinks()
    {
        link = new Vector3[joints.Length];
        for (int i = 0; i < joints.Length-1; i++)
        {
            link[i] = joints[i+1].position - joints[i].position;
        }

        link[2] = endEffector.position - joints[2].position;
    }


    Vector3[] GetVectors(Vector3 joint)
    {
        Vector3[] referenceVectors = new Vector3[2];
        referenceVectors[0] = Vector3.Normalize(endEffector.position - joint);
        referenceVectors[1] = Vector3.Normalize(target.position - joint);

        return referenceVectors;
    }
}
