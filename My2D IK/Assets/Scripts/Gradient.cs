using System;
using UnityEngine;

public class Gradient : MonoBehaviour
{

    public Transform Joint0;
    public Transform Joint1;
    public Transform Joint2;
    public Transform endEffector;
    public Transform target;

    public float alpha = 0.1f;
    private float tolerance = 1f;
    private float costFunction;
    private Vector3 theta;

    private Vector3 gradient;

    private float l1;
    private float l2;
    private float l3;



    //Parameter Adam's method - Adapt learning rate
    float beta1 = 0.9f;
    float beta2 = 0.999f;
    float epsilon = 1e-8f;
    int t = 1;
    Vector3 m_t = Vector3.zero;
    Vector3 v_t = Vector3.zero;

    //Angle constraints
    public Vector2 joint1Limits = new Vector2(-Mathf.PI * 0.75f, Mathf.PI * 0.75f); //-135 degrees and 135 degrees
    public Vector2 joint2Limits = new Vector2(-Mathf.PI * 0.5f, Mathf.PI * 0.5f); //pi * 2 es 90 graus ya que 2 * pi es una circumferencia
    public Vector2 joint3Limits = new Vector2(-Mathf.PI * 0.5f, Mathf.PI * 0.5f); //pi * 2 es 90 graus ya que 2 * pi es una circumferencia

    void Start()
    {
        l1 = Vector3.Distance(Joint0.position, Joint1.position);
        l2 = Vector3.Distance(Joint1.position, Joint2.position);
        l3 = Vector3.Distance(Joint2.position, endEffector.position);

        costFunction = Vector3.Distance(endEffector.position, target.position) * Vector3.Distance(endEffector.position, target.position);
        theta = Vector3.zero;
       
    }

    // Update is called once per frame
    void Update()
    {

        if (costFunction > tolerance)
        {
            t++;
            gradient = CalculateGradient();

            Vector3 newAlpha = AdaptiveLearningRate(gradient);

            theta += -newAlpha;
            
            theta = ApplyConstraints(theta);

            FordwardKinematics(theta);


        }

        costFunction = Vector3.Distance(endEffector.position, target.position) * Vector3.Distance(endEffector.position, target.position);
    }

    private void FordwardKinematics(Vector3 vector3)
    {
        Vector3 oldJ1 = Joint1.position;
        Vector3 oldJ2 = Joint2.position;

        Joint1.position = GetJoint1Position();
        Joint2.position = GetJoint2Position(oldJ1);
        endEffector.position = GetEndEffectorPosition(theta, oldJ2);
    }

    float Cost(Vector3 theta)
    {

        Vector3 endEffector = GetEndEffectorPosition(theta,Joint2.position);

        return Vector3.Distance(endEffector, target.position) * Vector3.Distance(endEffector, target.position);

    }

    Vector3 CalculateGradient()
    {

        Vector3 gradientVector;

        float step = 0.0001f;

        Vector3 thetaXPlus = new Vector3(theta.x + step, theta.y, theta.z);
        Vector3 thetaYPlus = new Vector3(theta.x, theta.y + step, theta.z);
        Vector3 thetaZPlus = new Vector3(theta.x, theta.y, theta.z + step);

        float DCostDX = (Cost(thetaXPlus) - Cost(theta)) / step;
        float DCostDY = (Cost(thetaYPlus) - Cost(theta)) / step;
        float DCostDZ = (Cost(thetaZPlus) - Cost(theta)) / step;


        gradientVector = new Vector3(DCostDX, DCostDY, DCostDZ);

        return gradientVector;


    }

    Vector3 GetJoint1Position()
    {
        Vector3 newPosition;

        Quaternion rot0 = Quaternion.Euler(0, 0, theta.x * Mathf.Rad2Deg);

        newPosition = Joint0.position + rot0 * (Joint1.position - Joint0.position);


        /*
        newPosition.x = Joint0.position.x + l1 * Mathf.Cos(theta.x);
        newPosition.y = Joint0.position.y + l1 * Mathf.Sin(theta.x);

        newPosition.z = 0;
        */

        return newPosition;
    }

    Vector3 GetJoint2Position(Vector3 oldJ1)
    {
        Vector3 newPosition;


         
        Quaternion rot1= Quaternion.Euler(0, 0, (theta.x + theta.y) * Mathf.Rad2Deg);


        newPosition = Joint1.position + rot1 * (Joint2.position - oldJ1);

        /*

        newPosition = Joint1.position + rot1 * Joint2.position;


        newPosition.x = Joint0.position.x + l1 * Mathf.Cos(theta.x)
                       + l2 * Mathf.Cos(theta.x + theta.y);
        newPosition.y = Joint0.position.y + l1 * Mathf.Sin(theta.x)
                       + l2 * Mathf.Sin(theta.x + theta.y);

        newPosition.z = 0;
        */
        return newPosition;
    }


    Vector3 GetEndEffectorPosition(Vector3 theta, Vector3 oldJ2)
    {
        Vector3 newPosition;

        Quaternion rot3 = Quaternion.Euler(0, 0, (theta.z + theta.y + theta.x) * Mathf.Rad2Deg);
        newPosition = Joint2.position + rot3 * (endEffector.position - oldJ2);


        /*
        newPosition.x = Joint0.position.x + l1 * Mathf.Cos(theta.x)
                       + l2 * Mathf.Cos(theta.x + theta.y)
                       + l3 * Mathf.Cos(theta.x + theta.y + theta.z);
        newPosition.y = Joint0.position.y + l1 * Mathf.Sin(theta.x)
                       + l2 * Mathf.Sin(theta.x + theta.y)
                       + l3 * Mathf.Sin(theta.x + theta.y + theta.z);

        newPosition.z = 0;
        */
        return newPosition;
    }

    Vector3 ApplyConstraints(Vector3 proposedAngles)
    {
        Vector3 constrainedAngles = proposedAngles;

        //Theta1 -  proposedAgles.x
        if (proposedAngles.x < joint1Limits.x)
        {
            constrainedAngles.x = joint1Limits.x;
        }
        else if(proposedAngles.x > joint1Limits.y)
        {
            constrainedAngles.x = joint1Limits.y;
        }


        //Theta1 - proposedAgles.y
        if (proposedAngles.y < joint2Limits.x)
        {
            constrainedAngles.y = joint2Limits.x;
        }
        else if (proposedAngles.y > joint2Limits.y)
        {
            constrainedAngles.y = joint2Limits.y;
        }


        //Theta1 - proposedAgles.z
        if (proposedAngles.z < joint3Limits.x)
        {
            constrainedAngles.z = joint3Limits.x;
        }
        else if (proposedAngles.z > joint3Limits.y)
        {
            constrainedAngles.z = joint3Limits.y;
        }

        return constrainedAngles;


    }

    Vector3 AdaptiveLearningRate(Vector3 gradient)
    {
        m_t = beta1 * m_t + (1 - beta1) * gradient;
        v_t = beta2 * v_t + (1 - beta2) * Vector3.Scale(gradient, gradient);

        Vector3 m_hat = m_t / (1 - beta1);
        Vector3 v_hat = v_t / (1 - beta2);

        Vector3 adaptiveAlpha = new Vector3(alpha * m_hat.x / Mathf.Sqrt(v_hat.x) +
            epsilon, alpha * m_hat.y / Mathf.Sqrt(v_hat.y) + 
            epsilon, alpha * m_hat.z / Mathf.Sqrt(v_hat.z) + epsilon);

        return adaptiveAlpha;
    }

}

//CALCULATEGRADIENT
/*
  Vector3 D = endEffector.position - target.position; // Vector de diferencia (Dx, Dy)

  // dC/d(theta_i) = 2 * (Dx * dEx/d(theta_i) + Dy * dEy/d(theta_i))
  // Nota: El factor '2' de la derivada se absorbe en el 'apha' (Learning Rate)

  // Tetha X (gradiente.x)
  float dEx_dthetaX = -(l1 * Mathf.Sin(theta.x) +
                        l2 * Mathf.Sin(theta.x + theta.y) +
                        l3 * Mathf.Sin(theta.x + theta.y + theta.z));
  float dEy_dthetaX = l1 * Mathf.Cos(theta.x) +
                      l2 * Mathf.Cos(theta.x + theta.y) +
                      l3 * Mathf.Cos(theta.x + theta.y + theta.z);

  gradientVector.x = D.x * dEx_dthetaX + D.y * dEy_dthetaX; // El factor '2' lo dejas a cargo de 'apha'

  // Tetha Y (gradiente.y)
  float dEx_dthetaY = -(l2 * Mathf.Sin(theta.x + theta.y) +
                        l3 * Mathf.Sin(theta.x + theta.y + theta.z));
  float dEy_dthetaY = l2 * Mathf.Cos(theta.x + theta.y) +
                      l3 * Mathf.Cos(theta.x + theta.y + theta.z);

  gradientVector.y = D.x * dEx_dthetaY + D.y * dEy_dthetaY;

  // Tetha Z (gradiente.z)
  float dEx_dthetaZ = -l3 * Mathf.Sin(theta.x + theta.y + theta.z);
  float dEy_dthetaZ = l3 * Mathf.Cos(theta.x + theta.y + theta.z);

  gradientVector.z = D.x * dEx_dthetaZ + D.y * dEy_dthetaZ;

  // Aunque habías normalizado, es mejor no normalizar inicialmente para mantener la magnitud de la pendiente.
  // Si mantienes la normalización, es la "Transpuesta Jacobiana".
  // gradientVector.Normalize(); 

  */
