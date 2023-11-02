using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.Mathematics;
using System.IO.Compression;
using System;
using System.Numerics;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;
using Random = UnityEngine.Random;
using UnityEditor.UIElements;
using System.Linq;



public class SelfDrive : Agent
{
    public float tiltAngleX;
    public float tiltAngleZ;
    public float tiltAngleY;
    public List<AxleInfo> axleInfos; 
    public float maxMotorTorque;
    public float maxSteeringAngle;
    public float lastVert;
    public float lastHor;

    public List<GameObject> checkpointList;

    public bool isInCollider;
    Rigidbody InitalBody;
    public float runTime;

    public int numberOfLaps;

public float gateTimeLimit;
private bool gateTimerStarted;
    public float NxtChpDot;

    public bool BeforeFirstGoal;


    public override void Initialize()
    {
        gateTimerStarted = false;
        if (!Academy.Instance.IsCommunicatorOn)
        {
            // MaxStep = 0;
        }
    }
    public void Start() {
        InitalBody = GetComponent<Rigidbody>();
        
        // foreach(GameObject cp in checkpointList){
        //     cp.name = "Checkpoint"+index;
        //     index++;
        // }
    }
    private void Update() {
        runTime += Time.deltaTime;
        if(gateTimerStarted){
            gateTimeLimit += Time.deltaTime;
            if(gateTimeLimit >= 5f){
                AddReward(-2f);
                EndEpisode();
            }
        }
        //try to prevent chilling in the base
        if (BeforeFirstGoal){
            AddReward(-0.01f * Time.deltaTime);
        }
        // AddReward(Time.deltaTime*-0.01f);
        // if(runTime > 30f){
        //     EndEpisode();
        // }
        if (isInCollider){AddReward(Time.deltaTime*-0.05f);
        
        }
    }
    public override void OnEpisodeBegin()
    {
        BeforeFirstGoal = true;
        gateTimerStarted = false;
        gateTimeLimit = 0;
        runTime = 0f;
        numberOfLaps = 0;
        this.InitalBody.angularVelocity = Vector3.zero;
        this.InitalBody.velocity = Vector3.zero;
        float RandomZ = Random.Range(-4f, 24f);
        float RandomX = Random.Range(-5f, 5f);
        float RandomY = Random.Range(-5f, 5f);
        transform.position =new Vector3(RandomX,0.15f,RandomZ);
        transform.rotation = Quaternion.identity;
        float RandomRotation = Random.Range(45, 135.0f);
        transform.Rotate(new Vector3(0f, RandomRotation, 0f), Space.Self);

        // gather all checkpoints on new episode
        checkpointList = new List<GameObject>(GameObject.FindGameObjectsWithTag("checkpoint"));
        int index = 0;

        for (int i = checkpointList.Count-1; i >= 0; i--)
        {   
            // Debug.Log(i);
            GameObject itm = checkpointList[i];
            itm.name = "Checkpoint"+index;
            index++;
        }
        checkpointList.Reverse();
        // checkpointList = sortedList;
    }
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0) {
            return;
        }
     
        Transform visualWheel = collider.transform.GetChild(0);
     
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
     
        // collider.transform.position = position;
        collider.transform.rotation = rotation;
    }
    
    public override void OnActionReceived(ActionBuffers actions)
    {
        float time = Time.deltaTime;
        float speed = InitalBody.velocity.magnitude;
        
        
        // didnt really work on first attempt 
        int GasOrBreak;
          int turnLeftRightOrNo;
        // AddReward(time*0.1f*(speed-20f)); //jag tycker att han borde åka i 20 iaf 
        // // Debug.Log("current speed = "+time*0.1f*(speed-20f));
        switch (actions.DiscreteActions[0]){
            case 0: GasOrBreak = -1;
                // Debug.Log("gas -1 taken");
                
                break;
            case 1: GasOrBreak = 0;
                // Debug.Log("gas 0 taken");
                break;
            case 2: GasOrBreak = 1;
                // Debug.Log("gas 1 taken");
                break;
            default: GasOrBreak = 0;
                // Debug.Log("gas default taken");
                break;
        }
         switch (actions.DiscreteActions[1]){
            case 0: turnLeftRightOrNo = -1;
                // Debug.Log("Steering -1 taken");
                break;
            case 1: turnLeftRightOrNo = 0;
                // Debug.Log("Steering 0 taken");
                break;
            case 2: turnLeftRightOrNo = 1;
                // Debug.Log("Steering 1 taken");
                break;
            default: turnLeftRightOrNo = 0;
                // Debug.Log("Steering default taken");
                break;
        }
        
        // Debug.Log("testing gas: " +GasOrBreak + " turn: " + turnLeftRightOrNo);
        lastHor = turnLeftRightOrNo;
        lastVert = GasOrBreak;
         {
        float motor = maxMotorTorque * GasOrBreak;
        float steering = maxSteeringAngle * turnLeftRightOrNo;


        // Handles driving 
        foreach (AxleInfo axleInfo in axleInfos) {
            if (axleInfo.steering) {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }
    }
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousAction = actionsOut.ContinuousActions;
        ActionSegment<int> discreetActions = actionsOut.DiscreteActions;
        // Debug.Log("vertical val = " + Input.GetAxisRaw("Vertical"));
        //  Debug.Log("horizontal val = " + Input.GetAxisRaw("Horizontal"));
        discreetActions[0] = ((int)Input.GetAxisRaw("Vertical"))+1;
        discreetActions[1] = ((int)Input.GetAxisRaw("Horizontal"))+1;
        
    }
    public override void CollectObservations(VectorSensor sensor)
    {    float dist = Vector3.Distance(transform.position, new Vector3(6.02413f,0.54f,-30.83176f));
        float speed = InitalBody.velocity.magnitude;

        
        
        Vector3 WorldPos = transform.position;
        //good to know the slopeangle when taking the corners 
        Quaternion rotation = transform.rotation;
        Vector3 eulerRotation = rotation.eulerAngles;
        // sensor.AddObservation(WorldPos);
        // sensor.AddObservation(speed);
        int listSize = checkpointList.Count -1;
        GameObject nextCheckpoint = checkpointList[0];

        //REWARD TO MOVE TO TARGET 
        Vector3 dirToNextCheckpoint = (nextCheckpoint.transform.position - transform.position).normalized;
        float velocityAlignment = Vector3.Dot(dirToNextCheckpoint,InitalBody.angularVelocity);
        AddReward(-0.01f*velocityAlignment);

        float directionDot = Vector3.Dot(transform.forward,nextCheckpoint.transform.forward);
        // Debug.Log(directionDot);
        NxtChpDot = directionDot;
        sensor.AddObservation(directionDot);
        // sensor.AddObservation(eulerRotation);
  
        
    }

   
    private void OnTriggerEnter(Collider other) {
        int listSize = checkpointList.Count -1;
        GameObject nextCheckpoint = checkpointList[0];
        if(other.GetType() == typeof(CapsuleCollider)){
            if(other.TryGetComponent<Point>(out Point point) && isInCollider == false){
                Debug.Log("NU HAR VI FAN MYCKET POÄNGFAN! LETS GO");
                isInCollider = true;
                AddReward(1f);
        }
        }
        if(other.GetType() == typeof(BoxCollider)){
            
            if(other.TryGetComponent<Checkpoint>(out Checkpoint checkpoint) && isInCollider == false){
                GameObject foundObj = checkpointList.Find(obj => obj.name == checkpoint.name);
                
                if (foundObj && foundObj.name == nextCheckpoint.name) {
                    BeforeFirstGoal = false;
                // Debug.Log("positive reward 1 ");
                AddReward(1f);
                gateTimerStarted = true;
                gateTimeLimit = 0;
                
                bool removed = checkpointList.Remove(foundObj); 
                if (removed) {
                   
                }
                if(checkpointList.Count == 0){
                    checkpointList = new List<GameObject>(GameObject.FindGameObjectsWithTag("checkpoint"));
                    int index = 0;
                    for (int i = checkpointList.Count-1; i >= 0; i--)
                        {   
                        // Debug.Log(i);
                        GameObject itm = checkpointList[i];
                        itm.name = "Checkpoint"+index;
                        index++;
                 }
        checkpointList.Reverse();
                    numberOfLaps++;

                    AddReward(10f);
                    if(numberOfLaps >= 3){
                    AddReward(10f);
                    EndEpisode();}
                }
                    
                } else {
                    // Debug.Log("negative reward -1 ");
                    AddReward(-1f);
                     
                }
            

            
                
                
            }
            // if(other.TryGetComponent<Checkpoint>(out Checkpoint checkpoints)){
            //     SetReward(5f);
            //     Debug.Log("GJ! reward: " +GetCumulativeReward());
                
            //     EndEpisode();
            
            if(other.TryGetComponent<Wall>(out Wall wall)){
                AddReward(-1f);
                EndEpisode();
                // Debug.Log("Failed! total reward: " +GetCumulativeReward());
                
                
            }
            isInCollider = true;
            }
    }
    void OnTriggerExit(Collider other)
    {
        if(other.GetType() == typeof(BoxCollider)){
            isInCollider = false;
            
           
        }
        if(other.GetType() == typeof(CapsuleCollider)){
            isInCollider = false;
        }
           
        
        
        
    }
}
