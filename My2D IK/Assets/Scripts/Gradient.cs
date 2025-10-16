using System;
using UnityEngine;

public class Gradient : MonoBehaviour
{

    public Transform joint0;
    public Transform joint1;
    public Transform joint2;
    public Transform endEffector;
    public Transform target;

    public float apha = 0.1f;
    private float tolerance = 1f;
    private float costFunction;
    private Vector3 theta;

    private Vector3 gradient;

    private float l1;
    private float l2;
    private float l3;

    void Start()
    {
        l1 = Vector3.Distance(joint0.position, joint1.position);
        l2 = Vector3.Distance(joint1.position, joint2.position);
        l3 = Vector3.Distance(joint2.position, endEffector.position);

        costFunction = Vector3.Distance(endEffector.position,target.position) * Vector3.Distance(endEffector.position, target.position);

        theta = Vector3.zero;

    }

    // Update is called once per frame
    void Update()
    {
        if(costFunction > tolerance)
        {
            gradient = CalculateGradient();
            theta += -apha * gradient;
            endEffector.position = UpdateEffectorPosition();
            joint1.position = UpdateJoint1();
            joint2.position = UpdateJoint2();

            

        }

        costFunction = Mathf.Pow( Vector3.Distance(endEffector.position, target.position),2);

    }

    private Vector3 UpdateJoint1()
    {
        Vector3 newPosition = Vector3.zero;
        newPosition.x = joint0.position.x + l1 * MathF.Cos(theta.x);


        newPosition.y = joint0.position.y + l1 * MathF.Sin(theta.x);

        return newPosition;

    }


    private Vector3 UpdateJoint2()
    {
        Vector3 newPosition = Vector3.zero;
        newPosition.x = joint0.position.x + l1 * MathF.Cos(theta.x) +            l2 * MathF.Cos(theta.x + theta.y);


        newPosition.y = joint0.position.y + l1 * MathF.Sin(theta.x) +       l2 * MathF.Sin(theta.x + theta.y);



        return newPosition;
    }

    
    private Vector3 UpdateEffectorPosition()
    {
        Vector3 newPosition = Vector3.zero;
        newPosition.x = joint0.position.x + l1 * MathF.Cos(theta.x) + 
            l2 * MathF.Cos(theta.x + theta.y) + 
            l3 * MathF.Cos(theta.x + theta.y + theta.z);

        newPosition.y = joint0.position.y + l1 * MathF.Sin(theta.x) +
       l2 * MathF.Sin(theta.x + theta.y) +
       l3 * MathF.Sin(theta.x + theta.y + theta.z);


        return newPosition;
    }

    private Vector3 CalculateGradient()
    {
        Vector3 gradientVector = Vector3.zero;

        Vector3 coeff = 2 * (endEffector.position - target.position);

        gradientVector.x = -coeff.x * (l1 * Mathf.Sin(theta.x) + 
            (l2 * Mathf.Sin(theta.x + theta.y)) +
            (l3 * Mathf.Sin(theta.x + theta.y + theta.z)) +
            (coeff.y * (l1 * Mathf.Cos(theta.x) +
            (l2 * Mathf.Cos(theta.x + theta.y)) +
            (l3 * Mathf.Cos(theta.x + theta.y + theta.z)))));


        gradientVector.y = -coeff.x * l2 * Mathf.Sin(theta.x + theta.y) +
            l3 * Mathf.Sin(theta.x + theta.y + theta.z) +
            (coeff.y * l2 * Mathf.Cos(theta.x + theta.y) +
            l3 * Mathf.Cos(theta.x + theta.y + theta.z));


        gradientVector.z = -coeff.x * (l3 * Mathf.Sin(theta.x + theta.y + theta.z))+ coeff.y * (l3 * Mathf.Cos(theta.x + theta.y + theta.z));

        gradientVector.Normalize();

        return gradientVector;
    }

    

}



