using System.Collections;
using System.Collections.Generic;
using JetBrains.Annotations;
using UnityEngine;

public class Point : MonoBehaviour
{
    // Start is called before the first frame update
    public Vector3 OGpos;
    public float runTime;
    void Start()
    {
        OGpos = transform.position;
        runTime = 0 ;
    }

    // Update is called once per frame
    void Update()
    {
        runTime += Time.deltaTime;
        if (runTime >= 15){
            float RandomX = Random.Range(-15f, 15f);
                transform.position = OGpos+ new Vector3(0f,0f,RandomX);
                runTime = 0;
        }
        
    }
}
