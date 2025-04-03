using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace Kart.MLA
{
    public class KartAgent : Agent, IDrive
    {
        public Transform[] wayPoints;
        public KartController kart;
        int currentWaypointIndex = 0;
        Transform initTransform;

        #region Rewards

        public float HitPenalty = -4f;
        public float PassCheckPointReward = 1f;
        public float SpeedReward = 1f;
        public float SlowPenalty = -4f;
        

        #endregion
        public Vector2 Move { get; private set; }
        public bool IsBraking { get; private set; }
        
        public void Enable() {
            // noop
        }

        protected override void Awake()
        {
            this.GetComponent<KartController>().SetInput(this);
            kart = GetComponent<KartController>();
            initTransform = transform;
        }

        private void Update()
        {
            Vector3 toNextPoint = wayPoints[currentWaypointIndex].position - transform.position;
            var distanceToNextPoint = toNextPoint.magnitude;
            if (distanceToNextPoint < 20.0f) {
                currentWaypointIndex = (currentWaypointIndex + 1) % wayPoints.Length;
                AddReward(PassCheckPointReward);
            }

            if (kart.Velocity.magnitude < 5)
            {
                AddReward(SlowPenalty);
            }
        }

        public override void OnEpisodeBegin()
        {
            base.OnEpisodeBegin();
            Debug.Log("Episode");
            if (transform != initTransform && kart != null)
            {
                kart.ResetKart(initTransform);
            }
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            sensor.AddObservation(kart.Velocity);
            sensor.AddObservation(currentWaypointIndex);
            Vector3 toNextPoint = wayPoints[currentWaypointIndex].position - transform.position;
            sensor.AddObservation(toNextPoint);
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            base.OnActionReceived(actions);
            
            float moveX = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
            float moveY = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
            IsBraking = actions.ContinuousActions[2] < 0f;

            Move = new Vector2(moveX, moveY);
            
            AddReward(kart.Velocity.magnitude * SpeedReward);
        }
        
        
        private void OnCollisionEnter(Collision other)
        {
            if (other.gameObject.layer == LayerMask.NameToLayer("Wall"))
            {
                AddReward(HitPenalty);
            }
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var continuousActionsOut = actionsOut.ContinuousActions;
            continuousActionsOut[0] = Input.GetAxis("Horizontal");
            continuousActionsOut[1] = Input.GetAxis("Vertical");
            continuousActionsOut[2] = 1;
        }
    }
}