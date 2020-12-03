using Sirenix.OdinInspector;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BodyCenter : MonoBehaviour
{
    public List<BodyCenter> neighbours = new List<BodyCenter>();

    public float mass = 1.0f;
    [ReadOnly]
    public float invMass = 1.0f;
    public int Index;
    public Vector3 velocity;

    void Start()
    {
        invMass = mass / 1.0f;
        PhysicsManager.Instance.Register(this);

    }
    private void OnEnable()
    {
    }

    private void OnDisable()
    {
        if ( PhysicsManager.Instance)
            PhysicsManager.Instance.Unregister(this);
    }
}
