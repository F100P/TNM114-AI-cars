using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GetAllCheckpoints : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject[] gameObjects;
    void Start()
    {
       
        gameObjects = GameObject.FindGameObjectsWithTag("checkpoint");

        if (gameObjects.Length == 0)
        {
            Debug.Log("No GameObjects are tagged with 'Checkpint'");
        }
        foreach (GameObject go in gameObjects){
            Debug.Log(go.name);
        }
        

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
