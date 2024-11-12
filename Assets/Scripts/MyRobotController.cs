using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;

enum RobotState
{
    P_target,
    P_WorkBench,
    P_InitialPosition
};

public class MyRobotController : MonoBehaviour
{
    public Transform Joint0;
    public Transform Joint1;
    public Transform Joint2;
    public Transform endFactor;
    public Transform Stud_target;
    public Transform Workbench_destination;

    public Transform InitialPosition;

    public bool pickTarget = false;

    private bool hasPickedUp = false;
    private RobotState robotState = RobotState.P_target;

    public float alpha = 0.1f;
    private float tolerance = 1f;
    private float costFunction;
    private Vector3 gradient;
    private Vector3 theta;


    private float l1;
    private float l2;
    private float l3;

    private float maxDistance;
    private float distance;

    // Start is called before the first frame update
    void Start()
    {
        l1 = Vector3.Distance(Joint0.position, Joint1.position);
        l2 = Vector3.Distance(Joint1.position, Joint2.position);
        l3 = Vector3.Distance(Joint2.position, endFactor.position);

        InitialPosition.position = endFactor.position;

        maxDistance = l1 + l2 + l3;

        costFunction = Vector3.Distance(endFactor.position, Stud_target.position) * Vector3.Distance(endFactor.position, Stud_target.position);

        theta = Vector3.zero;

        distance = Vector3.Distance(Joint0.position, Stud_target.position);
    }

    // Update is called once per frame
    void Update()
    {
        //distance = Vector3.Distance(Joint0.position, Stud_target.position);

        //if (distance <= maxDistance)
        //{
            
        if(pickTarget)
        {
            PickStudAnim();
        }
            
        //}

        
    }

    void PickStudAnim()
    {
        switch(robotState)
        {
            case RobotState.P_target:

                MoveRobot(Stud_target);

                break;

            case RobotState.P_WorkBench:

                MoveTarget(Stud_target);
                MoveRobot(Workbench_destination);

                break;

            case RobotState.P_InitialPosition:

                MoveRobot(InitialPosition);

                break;

            default: 
                break; 
        }

    }

    void MoveRobot(Transform target)
    {
        costFunction = Vector3.Distance(endFactor.position, target.position) * Vector3.Distance(endFactor.position, target.position);

        //Comprobamos que tocamos el target
        if (costFunction > tolerance)
        {

            gradient = CalculateGradient(target);
            theta += -alpha * gradient;
            endFactor.position = GetEndFactorPosition();


            Joint1.position = GetJoint1Position();
            Joint2.position = GetJoint2Position();

        }
        else
        {
            switch(robotState)
            { 
                case RobotState.P_target:

                    robotState = RobotState.P_WorkBench;

                    break;

                case RobotState.P_WorkBench:

                    robotState = RobotState.P_InitialPosition;

                    break;

                case RobotState.P_InitialPosition:
                    
                    

                    break;

            default:
            break;
            }
        }


        
    }

    void MoveTarget(Transform target)
    {
        target.position = endFactor.position;
    }

    Vector3 CalculateGradient(Transform target)
    {

        Vector3 gradientVector;

        Vector3 coeff = 2 * (endFactor.position - target.position);

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
