using System;
using System.Collections.Generic;
using UnityEngine;

public class Loop : MonoBehaviour
{

    public List<Transform> Joints;
    public Transform target;
    Vector3[] Links;
    private int numberOfJoints;
    private Vector3 endEffector;
    private int endEffectorIndex;
    private int initialPositionIndex; // base of my structure


    //tolerance and max iterations
    public float tolerance = 1.0f;
    public float maxIterations = 1e5f;
    private int countIterations = 0;

    //method's parameter
    private float lambda;
    private Vector3 initialPosition;


    void Start()
    {
        numberOfJoints = Joints.Count;
        GetLinks();
        DefineInitialPosition();
        DefineEndEffector();
    }



    // Update is called once per frame
    void Update()
    {
        if(countIterations < maxIterations && 
            Vector3.Distance(endEffector,target.position) > tolerance)
        {
            FordWard();
            BackWard();
            countIterations++;

            endEffector = Joints[endEffectorIndex].position;
        }
    }

    private void GetLinks()
    {
        Links = new Vector3[numberOfJoints - 1];

        for(int i = 0; i < numberOfJoints ; i++)
        {
            Links[i] = Joints[i+1].position - Joints[i].position;
        }

        Links[numberOfJoints - 1] = Joints[numberOfJoints-2].position - Joints[0].position;

    }

    void DefineEndEffector()
    {
        endEffectorIndex = 0;
        for(int i = 1; i < numberOfJoints; i++)
        {
            if (Vector3.Distance(Joints[i].position,target.position) < Vector3.Distance(Joints[endEffectorIndex].position,target.position))
            {
                endEffectorIndex = i;
            }
        }

        endEffector = Joints[endEffectorIndex].position;
    }

    void DefineInitialPosition()
    {
        initialPositionIndex = 0;
        for(int i = 1; i <numberOfJoints;i++)
        {
            if (Vector3.Distance(Joints[i].position, target.position) > Vector3.Distance(Joints[initialPositionIndex].position, target.position))
            {
                initialPositionIndex = i;
            }
        }

        initialPosition = Joints[initialPositionIndex].position;
    }

    private void FordWard()
    {
        Joints[endEffectorIndex].position = target.position;

        int i = (endEffectorIndex - 1 + numberOfJoints) % numberOfJoints;

        int jointsProcessed = 0;

        while(jointsProcessed < numberOfJoints)
        {
            int nextIndex = (i + 1) % numberOfJoints;
            float distance = Vector3.Magnitude(Links[i-1]);
            float denominator = Vector3.Distance(Joints[i].position, Joints[nextIndex].position);
            lambda = distance / denominator;
            Joints[i].position = Joints[i].position + (1 - lambda) * (Joints[nextIndex].position - Joints[i].position);
            
            
            jointsProcessed++;
        }


        i = (endEffectorIndex + i) % numberOfJoints;
        jointsProcessed = 0;

        while (jointsProcessed < numberOfJoints)
        {
            int nextIndex = (i - 1 + numberOfJoints) % numberOfJoints;
            float distance = Vector3.Magnitude(Links[i-1]);
            float denominator = Vector3.Distance(Joints[i].position, Joints[nextIndex].position);
            lambda = distance / denominator;
            Joints[i].position = Joints[i].position + (1 - lambda) * (Joints[nextIndex].position - Joints[i].position);

            jointsProcessed++;
        }
    }

    private void BackWard()
    {
        Joints[initialPositionIndex].position = initialPosition;

        int i = (initialPositionIndex + 1) % numberOfJoints;
        int jointsProcessed = 0;

        while (jointsProcessed < numberOfJoints)
        {
            int prevIndex = (i - 1 + numberOfJoints) % numberOfJoints;

            float distance = Vector3.Magnitude(Links[prevIndex]);
            float denominator = Vector3.Distance(Joints[prevIndex].position, Joints[i].position);
            lambda = distance / denominator;
            Joints[i].position = Joints[i].position + (1 - lambda) * (Joints[prevIndex].position - Joints[i].position);
        }


        i = (i-1 + numberOfJoints) % numberOfJoints;
        jointsProcessed = 0;

        while (jointsProcessed < numberOfJoints)
        {
            int prevIndex = (i + 1) % numberOfJoints;

            float distance = Vector3.Magnitude(Links[prevIndex]);
            float denominator = Vector3.Distance(Joints[prevIndex].position, Joints[i].position);
            lambda = distance / denominator;
            Joints[i].position = Joints[i].position + (1 - lambda) * (Joints[prevIndex].position - Joints[i].position);
        }

    }




}
