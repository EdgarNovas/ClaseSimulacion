using System.Collections.Generic;
using UnityEngine;

public class Leaf : MonoBehaviour
{
    public List<Transform> Joints;
    public Transform target;
    Vector3[] Links;
    private int numberOfJoints;
    private Vector3 endEffector;


    //tolerance and max iterations
    public float tolerance = 1.0f;
    public float maxIterations = 1e5f;
    private int countIterations = 0;

    //method's parameter
    private float lambda;
    private Vector3 initialPosition;
    private int endEffectorIndex;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        numberOfJoints = Joints.Count;
        GetLinks();
        initialPosition = Joints[0].position;
        DefineEndEffector();
    }



    // Update is called once per frame
    void Update()
    {
        if (countIterations < maxIterations &&
            Vector3.Distance(endEffector, target.position) > tolerance)
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

        for (int i = 0; i < numberOfJoints - 1; i++)
        {
            Links[i] = Joints[i + 1].position - Joints[i].position;
        }

    }

    void DefineEndEffector()
    {
        endEffectorIndex = 0;
        for (int i = 1; i < numberOfJoints; i++)
        {
            if (Vector3.Distance(Joints[i].position, target.position) < Vector3.Distance(Joints[endEffectorIndex].position, target.position))
            {
                endEffectorIndex = i;
            }
        }

        endEffector = Joints[endEffectorIndex].position;
    }

    private void FordWard()
    {
        Joints[endEffectorIndex].position = target.position;

        for (int i = endEffectorIndex - 1; i >= 0; i--)
        {
            float distance = Vector3.Magnitude(Links[i]);
            float denominator = Vector3.Distance(Joints[i].position, Joints[i + 1].position);
            lambda = distance / denominator;
            Joints[i].position = Joints[i].position + (1 - lambda) * (Joints[i + 1].position - Joints[i].position);

        }

        for(int j = endEffectorIndex + 1; j < numberOfJoints; j++)
        {
            float distance = Vector3.Magnitude(Links[j-1]);
            float denominator = Vector3.Distance(Joints[j].position, Joints[j- 1].position);
            lambda = distance / denominator;
            Joints[j].position = Joints[j].position + (1 - lambda) * (Joints[j -1].position - Joints[j].position);
        }

    }

    private void BackWard()
    {
        Joints[0].position = initialPosition;
        for (int i = 1; i < numberOfJoints; i++)
        {
            float distance = Vector3.Magnitude(Links[i - 1]);
            float denominator = Vector3.Distance(Joints[i - 1].position, Joints[i].position);
            lambda = distance / denominator;
            Joints[i].position = Joints[i].position + (1 - lambda) * (Joints[i - 1].position - Joints[i].position);
        }
    }

}
