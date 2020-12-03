using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConnectionVisialization : MonoBehaviour
{
    public Transform root;
    public GameObject barPrefab;

    public List<GameObject> bars = new List<GameObject>();

    private void Awake()
    {
        bars = new List<GameObject>();

        for( int i = 0;  i  < 100; ++ i )
        {
            var bar = Instantiate(barPrefab) as GameObject;
            bar.transform.parent = root;

            bars.Add(bar);

            bar.SetActive(false);
        }
    }

    public void Update()
    {
        int i;
        for( i = 0; i < PhysicsManager.Instance.connectionList.Count; ++ i )
        {
            var con = PhysicsManager.Instance.connectionList[i];
            var from = con.body0.transform.position;
            var to = con.body1.transform.position;
            var fromTo = to - from;
            var radius = 0.1f;

            bars[i].transform.position = (from + to) * 0.5f;
            bars[i].transform.up = fromTo;
            bars[i].transform.localScale = new Vector3(radius, fromTo.magnitude * 0.5f, radius);
            bars[i].SetActive(true);
        }

        for (; i < bars.Count; ++i )
        {
            bars[i].SetActive(false);
        }
    }
}
