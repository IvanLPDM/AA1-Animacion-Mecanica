using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wall : MonoBehaviour
{
    public MyRobotController robotController; // Referencia al controlador principal

    private void OnTriggerEnter2D(Collider2D other)
    {
        if (other.CompareTag("Robot"))
        {
            //Direccion opuesta
            Vector3 collisionDirection = (transform.position - other.transform.position).normalized;

            // Notifica al controlador de la colisión
            robotController.OnSegmentCollision(this, other, collisionDirection);
        }
    }


    private void OnTriggerExit2D(Collider2D other)
    {
        if (other.CompareTag("Robot"))
        {
            robotController.NoCollision();
        }
    }
}
