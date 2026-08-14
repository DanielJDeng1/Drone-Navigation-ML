using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class DroneAgent : Agent
{
    [Header("Drone Settings")]
    [SerializeField] private float moveForce = 15f;
    [SerializeField] private float turnTorque = 10f;
    private Rigidbody rb;

    [Header("Target Reference")]
    [SerializeField] private Transform targetTransform;

    [Header("Arena Bounds")]
    [SerializeField] private Vector3 arenaSize = new Vector3(20f, 10f, 20f);
    [SerializeField] private Vector3 arenaCenter = Vector3.zero;

    [Header("Obstacles")]
    [SerializeField] private int minObstacles = 3;
    [SerializeField] private int maxObstacles = 8;
    [SerializeField] private Vector3 minScale = new Vector3(1f, 1f, 1f);
    [SerializeField] private Vector3 maxScale = new Vector3(3f, 5f, 3f);

    private List<GameObject> spawnedObstacles = new List<GameObject>();
    private float initialDistanceToTarget;
    private float previousDistanceToTarget;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
    }

    public override void OnEpisodeBegin()
    {
        // zero out momentum from previous attempt
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        ClearObstacles();
        GenerateProceduralObstacles();

        // spawn agent on left flank with random position offset to avoid trajectory overfitting
        transform.localPosition = arenaCenter + new Vector3(
            Random.Range(-arenaSize.x * 0.4f, -arenaSize.x * 0.2f),
            Random.Range(-arenaSize.y * 0.3f, arenaSize.y * 0.3f),
            Random.Range(-arenaSize.z * 0.4f, arenaSize.z * 0.4f)
        );
        transform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

        // spawn goal zone on opposing right flank
        targetTransform.localPosition = arenaCenter + new Vector3(
            Random.Range(arenaSize.x * 0.2f, arenaSize.x * 0.4f),
            Random.Range(-arenaSize.y * 0.3f, arenaSize.y * 0.3f),
            Random.Range(-arenaSize.z * 0.4f, arenaSize.z * 0.4f)
        );

        initialDistanceToTarget = Vector3.Distance(transform.localPosition, targetTransform.localPosition);
        previousDistanceToTarget = initialDistanceToTarget;
    }

    private void GenerateProceduralObstacles()
    {
        int count = Random.Range(minObstacles, maxObstacles + 1);

        for (int i = 0; i < count; i++)
        {
            GameObject obs = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obs.tag = "Obstacle";
            obs.transform.SetParent(this.transform.parent != null ? this.transform.parent : null);

            // restrict obstacle placement to center corridors between start and end
            obs.transform.localPosition = arenaCenter + new Vector3(
                Random.Range(-arenaSize.x * 0.2f, arenaSize.x * 0.2f),
                Random.Range(-arenaSize.y * 0.4f, arenaSize.y * 0.4f),
                Random.Range(-arenaSize.z * 0.4f, arenaSize.z * 0.4f)
            );

            obs.transform.localScale = new Vector3(
                Random.Range(minScale.x, maxScale.x),
                Random.Range(minScale.y, maxScale.y),
                Random.Range(minScale.z, maxScale.z)
            );

            obs.transform.localRotation = Quaternion.Euler(
                Random.Range(0f, 360f),
                Random.Range(0f, 360f),
                Random.Range(0f, 360f)
            );

            spawnedObstacles.Add(obs);
        }
    }

    private void ClearObstacles()
    {
        foreach (GameObject obs in spawnedObstacles)
        {
            if (obs != null) Destroy(obs);
        }
        spawnedObstacles.Clear();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 relativeTarget = targetTransform.localPosition - transform.localPosition;

        // 12 observation vectors
        // target relative offset
        // target heading vector (normalized)
        // linear velocity
        // angular velocity
        sensor.AddObservation(relativeTarget);
        sensor.AddObservation(relativeTarget.normalized);
        sensor.AddObservation(rb.linearVelocity);
        sensor.AddObservation(rb.angularVelocity);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[0];
        float moveY = actions.ContinuousActions[1];
        float moveZ = actions.ContinuousActions[2];
        float turnY = actions.ContinuousActions[3];

        rb.AddRelativeForce(new Vector3(moveX, moveY, moveZ) * moveForce, ForceMode.Force);
        rb.AddTorque(Vector3.up * turnY * turnTorque, ForceMode.Force);

        float currentDistance = Vector3.Distance(transform.localPosition, targetTransform.localPosition);

        // continuous progress reward based on distance closed since last tick
        float progress = (previousDistanceToTarget - currentDistance) / initialDistanceToTarget;
        AddReward(progress * 1.0f);
        previousDistanceToTarget = currentDistance;

        // terminal proximity bias to discourage hovering outside the goal trigger
        if (currentDistance < 3.0f)
        {
            AddReward(0.005f);
        }

        AddReward(-0.0002f);

        // hard reset on boundary breach
        Vector3 posFromCenter = transform.localPosition - arenaCenter;
        if (Mathf.Abs(posFromCenter.x) > arenaSize.x * 0.5f ||
            Mathf.Abs(posFromCenter.y) > arenaSize.y * 0.5f ||
            Mathf.Abs(posFromCenter.z) > arenaSize.z * 0.5f)
        {
            SetReward(-1.0f);
            EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Target"))
        {
            SetReward(5.0f);
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall") || collision.gameObject.CompareTag("Obstacle"))
        {
            SetReward(-1.0f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // manual override controls for debugging input mapping in editor
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxisRaw("Horizontal");
        continuousActions[1] = Input.GetKey(KeyCode.Space) ? 1f : (Input.GetKey(KeyCode.LeftShift) ? -1f : 0f);
        continuousActions[2] = Input.GetAxisRaw("Vertical");
        continuousActions[3] = Input.GetKey(KeyCode.E) ? 1f : (Input.GetKey(KeyCode.Q) ? -1f : 0f);
    }
}