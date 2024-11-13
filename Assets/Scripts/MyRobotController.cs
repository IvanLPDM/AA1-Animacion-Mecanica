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

    public LineRenderer lineRenderer1;
    public LineRenderer lineRenderer2;
    public LineRenderer lineRenderer3;

    public float collidersSize;

    private BoxCollider2D colliderBetweenJoint0And1;
    private BoxCollider2D colliderBetweenJoint1And2;
    private BoxCollider2D colliderBetweenJoint2And3;

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

        InitializeLineRenderer(lineRenderer1);
        InitializeLineRenderer(lineRenderer2);
        InitializeLineRenderer(lineRenderer3);

        colliderBetweenJoint0And1 = CreateColliderBetween(Joint0, Joint1);
        colliderBetweenJoint1And2 = CreateColliderBetween(Joint1, Joint2);
        colliderBetweenJoint2And3 = CreateColliderBetween(Joint2, endFactor);
    }

    void InitializeLineRenderer(LineRenderer lineRenderer)
    {
        // Set up the LineRenderer properties like width, color, etc.
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        lineRenderer.positionCount = 2; // Each bone only needs 2 points (start and end)
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));  // Basic material for 2D
        lineRenderer.startColor = Color.white;
        lineRenderer.endColor = Color.grey;


    }

    BoxCollider2D CreateColliderBetween(Transform jointA, Transform jointB)
    {
        GameObject colliderObject = new GameObject("ColliderBetween" + jointA.name + "And" + jointB.name);
        colliderObject.transform.parent = transform;  // Hacer que el objeto collider sea hijo del controlador

        BoxCollider2D boxCollider = colliderObject.AddComponent<BoxCollider2D>();
        boxCollider.isTrigger = true; 

        return boxCollider;
    }

    // Update is called once per frame
    void Update()
    {
        //distance = Vector3.Distance(Joint0.position, Stud_target.position);

        //if (distance <= maxDistance)
        //{
        UpdateCollider(colliderBetweenJoint0And1, Joint0.position, Joint1.position);
        UpdateCollider(colliderBetweenJoint1And2, Joint1.position, Joint2.position);
        UpdateCollider(colliderBetweenJoint2And3, Joint2.position, endFactor.position);


        if (pickTarget)
        {
            PickStudAnim();
        }

        UpdateVisualLinks();

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

            Vector3 newEndFactorPos = GetEndFactorPosition();
            Vector3 newJoint1Pos = GetJoint1Position();
            Vector3 newJoint2Pos = GetJoint2Position();

            // Verifica si se cruza con el segmento previo
            if (!LinesIntersect(Joint0.position, newJoint1Pos, newJoint1Pos, newJoint2Pos))
            {
                endFactor.position = newEndFactorPos;
                Joint1.position = newJoint1Pos;
                Joint2.position = newJoint2Pos;
            }

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

    bool LinesIntersect(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2)
    {
        return false;
    }

    void MoveTarget(Transform target)
    {
        target.position = endFactor.position;
    }

    void UpdateVisualLinks()
    {
        lineRenderer1.SetPosition(0, Joint0.position);
        lineRenderer1.SetPosition(1, Joint1.position); 

        lineRenderer2.SetPosition(0, Joint1.position);
        lineRenderer2.SetPosition(1, Joint2.position);

        lineRenderer3.SetPosition(0, Joint2.position); 
        lineRenderer3.SetPosition(1, endFactor.position);
    }

    void UpdateCollider(BoxCollider2D boxCollider, Vector3 startPos, Vector3 endPos)
    {
        //colocamos el boxCollider en el medio de la recta
        Vector3 midPoint = (startPos + endPos) / 2;
        boxCollider.transform.position = midPoint;

        float distance = Vector3.Distance(startPos, endPos);
        boxCollider.size = new Vector2(distance, collidersSize);

        //Rotar Collider
        Vector3 direction = endPos - startPos;
        float angle = Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg;
        boxCollider.transform.rotation = Quaternion.Euler(0, 0, angle);
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
