using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MyRobotController : MonoBehaviour
{
    public Transform Joint0;
    public Transform Joint1;
    public Transform Joint2;
    public Transform endFactor;
    public Transform Stud_target;


    public float alpha = 0.1f;
    private float tolerance = 1f;
    private float costFunction;
    private Vector3 gradient;
    private Vector3 theta;


    private float l1;
    private float l2;
    private float l3;

    // Start is called before the first frame update
    void Start()
    {
        l1 = Vector3.Distance(Joint0.position, Joint1.position);
        l2 = Vector3.Distance(Joint1.position, Joint2.position);
        l3 = Vector3.Distance(Joint2.position, endFactor.position);

        costFunction = Vector3.Distance(endFactor.position, Stud_target.position) * Vector3.Distance(endFactor.position, Stud_target.position);
        theta = Vector3.zero;
    }

    // Update is called once per frame
    void Update()
    {

        if (costFunction > tolerance)
        {

            gradient = CalculateGradient();
            theta += -alpha * gradient;
            endFactor.position = GetEndFactorPosition();


            Joint1.position = GetJoint1Position();
            Joint2.position = GetJoint2Position();
        }

        costFunction = Vector3.Distance(endFactor.position, Stud_target.position) * Vector3.Distance(endFactor.position, Stud_target.position);



    }


    Vector3 CalculateGradient()
    {

        Vector3 gradientVector;

        Vector3 coeff = 2 * (endFactor.position - Stud_target.position);

        gradientVector.x = -coeff.x * (l1 * Mathf.Sin(theta.x)
            + l2 * Mathf.Sin(theta.x + theta.y)
            + l3 * Mathf.Sin(theta.x + theta.y + theta.z))
            + coeff.y * (l1 * Mathf.Cos(theta.x) +
            l2 * Mathf.Cos(theta.x + theta.y) +
            l3 * Mathf.Cos(theta.x + theta.y + theta.z));

        gradientVector.y = -coeff.x * (l2 * Mathf.Sin(theta.x + theta.y)
            + l3 * Mathf.Sin(theta.x + theta.y + theta.z))
            + coeff.y * (l2 * Mathf.Cos(theta.x + theta.y) +
            l3 * Mathf.Cos(theta.x + theta.y + theta.z));

        gradientVector.z = -coeff.x * (l3 * Mathf.Sin(theta.x + theta.y + theta.z))
                         + coeff.y * (l3 * Mathf.Cos(theta.x + theta.y + theta.z));


        gradientVector.Normalize();

        return gradientVector;


    }


    Vector3 GetEndFactorPosition()
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
}
