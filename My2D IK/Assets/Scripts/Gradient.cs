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
            endEffector.position = GetEndEffectorPosition(theta);


            Joint1.position = GetJoint1Position();
            Joint2.position = GetJoint2Position();
        }

        costFunction = Vector3.Distance(endEffector.position, target.position) * Vector3.Distance(endEffector.position, target.position);
    }


    float Cost(Vector3 theta)
    {

        Vector3 endEffector = GetEndEffectorPosition(theta);

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


    Vector3 GetEndEffectorPosition(Vector3 theta)
    {
        Vector3 newPosition;

        newPosition.x = Joint0.position.x + l1 * Mathf.Cos(theta.x)
                       + l2 * Mathf.Cos(theta.x + theta.y)
                       + l3 * Mathf.Cos(theta.x + theta.y + theta.z);
        newPosition.y = Joint0.position.y + l1 * Mathf.Sin(theta.x)
                       + l2 * Mathf.Sin(theta.x + theta.y)
                       + l3 * Mathf.Sin(theta.x + theta.y + theta.z);

        newPosition.z = 0;

        return newPosition;
    }

    Vector3 GetJoint2Position()
    {
        Vector3 newPosition;

        newPosition.x = Joint0.position.x + l1 * Mathf.Cos(theta.x)
                       + l2 * Mathf.Cos(theta.x + theta.y);
        newPosition.y = Joint0.position.y + l1 * Mathf.Sin(theta.x)
                       + l2 * Mathf.Sin(theta.x + theta.y);

        newPosition.z = 0;

        return newPosition;
    }

    Vector3 GetJoint1Position()
    {
        Vector3 newPosition;

        newPosition.x = Joint0.position.x + l1 * Mathf.Cos(theta.x);
        newPosition.y = Joint0.position.y + l1 * Mathf.Sin(theta.x);

        newPosition.z = 0;

        return newPosition;
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
