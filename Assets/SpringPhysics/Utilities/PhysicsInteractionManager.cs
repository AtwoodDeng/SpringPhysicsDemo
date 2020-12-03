using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsInteractionManager : MonoBehaviour
{
    public GameObject bodyPrefab;
    public Transform bodyRoot;

    public float connectDistance = 3f;
    public void Update()
    {
        if ( Input.GetMouseButtonDown(0) )
        {
            CreateBody( Camera.main.ScreenToWorldPoint(new Vector3( Input.mousePosition.x , Input.mousePosition.y , 10f ) ) );
        }

    }

    public void CreateBody( Vector3 worldPos )
    {
        var body = Instantiate(bodyPrefab) as GameObject;
        body.transform.position = worldPos;
        body.transform.parent = bodyRoot;

        var bodyCom = body.GetComponent<BodyCenter>();
        bodyCom.neighbours = new List<BodyCenter>();
        // find neighbour
        foreach( var other in PhysicsManager.Instance.bodyList )
        {
            if ( Vector3.Distance( other.transform.position , worldPos) < connectDistance )
            {
                bodyCom.neighbours.Add(other);
            }
        }
    }

}
