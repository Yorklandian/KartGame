using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Utilities;

namespace Kart
{
    /**
     * 四轮车车轴
     */
    [System.Serializable]
    public class AxleInfo
    {
        public WheelCollider leftWheel;
        public WheelCollider rightWheel;
        public bool motor; //是否是动力轴
        public bool steering; //是否是转向轴
        public WheelFrictionCurve originalForwardFriction; //原始前向摩擦力
        public WheelFrictionCurve originalSidewayFriction; //原始侧向摩擦力

    }
    
    public class KartController : MonoBehaviour
    {
        [Header("Axle Information")] 
        [SerializeField] private AxleInfo[] axleInfos;

        [Header("Motor Attributes")]
        [SerializeField] private float maxMotorTorque = 3000f;
        [SerializeField] private float maxSpeed;
        
        [Header("Steer Attributes")]
        [SerializeField] private float maxSteeringAngle = 30f;
        [SerializeField] private AnimationCurve turnCurve;
        [SerializeField] private float turnStrength = 1500f;

        [Header("Braking and Drifting")]
        [SerializeField] private float driftSpeedMultiplier = 1.5f;
        [SerializeField] private float brakeTorque = 10000f;

        [Header("Physics")]
        [SerializeField] private Transform centerOfMass;
        [SerializeField] private float downForce = 100f;
        [SerializeField] private float gravity = Physics.gravity.y;
        [SerializeField] private float lateralGScale = 10f; //侧向G力

        [Header("Banking")]
        [SerializeField] private float maxBankingAngle = 5f;
        [SerializeField] private float bankSpeed = 2f;
        
        [Header("Refs")]
        [SerializeField] private InputReader playerInput;
        [SerializeField] Circuit circuit;
        [SerializeField] AIDriverData driverData;

        private IDrive input;
        private Rigidbody rb;
        private Vector3 kartVelocity;
        private float brakeVelocity;
        private float driftVelocity;

        private float originalY;
        private float adjustedY;
        private float yDiff;
        
        private Vector3 syncPosition;
        
        private RaycastHit hit;

        private const float thresholdSpeed = 10f;
        private const float centerOfMassOffset = -0.5f;
        private Vector3 originalCenterOfMass;
        

        
        public bool IsGrounded = true;
        public Vector3 Velocity => kartVelocity;
        public float MaxSpeed => maxSpeed;

        private void Awake()
        {
            if (playerInput is IDrive driveInput) {
                input = driveInput;
            }
        }

        private void Start()
        {
            rb = GetComponent<Rigidbody>();
            input.Enable();


            rb.centerOfMass = centerOfMass.localPosition;
            originalCenterOfMass = centerOfMass.localPosition;
            
            foreach (AxleInfo axleInfo in axleInfos)
            {
                axleInfo.originalForwardFriction = axleInfo.leftWheel.forwardFriction;
                axleInfo.originalSidewayFriction = axleInfo.leftWheel.sidewaysFriction;
            }
        }

        private void FixedUpdate()
        {
            Move();
        }
        
        public void SetInput(IDrive input) {
            this.input = input;
        }



        #region Basic Movement

        private void Move()
        {
            float verticalInput = AdjustInput(input.Move.y);
            float horizontalInput = AdjustInput(input.Move.x);
            
            float motor = maxMotorTorque * verticalInput;
            float steer = maxSteeringAngle * horizontalInput;
            UpdateAxles(motor, steer);
            UpdateBanking(horizontalInput);
            
            kartVelocity = transform.InverseTransformDirection(rb.velocity);
            
            if (IsGrounded) {
                HandleGroundedMovement(verticalInput, horizontalInput);
            } else {
                HandleAirborneMovement(verticalInput, horizontalInput);
            }
        }
        
        private void UpdateBanking(float horizontalInput)
        {
            // Bank the Kart in the opposite direction of the turn
            float targetBankAngle = horizontalInput * -maxBankingAngle;
            Vector3 currentEuler = transform.localEulerAngles;
            currentEuler.z = Mathf.LerpAngle(currentEuler.z, targetBankAngle, Time.deltaTime * bankSpeed);
            transform.localEulerAngles = currentEuler;
        }

        private void HandleAirborneMovement(float verticalInput, float horizontalInput)
        {
            //Add gravity
            rb.velocity = Vector3.Lerp(rb.velocity, rb.velocity + Vector3.down * gravity, Time.deltaTime * gravity);
        }

        private void HandleGroundedMovement(float verticalInput, float horizontalInput)
        {
            // Turn logic
            if (Mathf.Abs(verticalInput) > 0.1f || Mathf.Abs(kartVelocity.z) > 1) {
                float turnMultiplier = Mathf.Clamp01(turnCurve.Evaluate(kartVelocity.magnitude / maxSpeed));
                rb.AddTorque(Vector3.up * (horizontalInput * Mathf.Sign(kartVelocity.z) * turnStrength * 100f * turnMultiplier));
            }
            
            // Acceleration Logic
            if (!input.IsBraking) {
                float targetSpeed = verticalInput * maxSpeed;
                Vector3 forwardWithoutY = transform.forward.With(y: 0).normalized;
                rb.velocity = Vector3.Lerp(rb.velocity, forwardWithoutY * targetSpeed, Time.deltaTime);
            }
            
            // Downforce - always push the Kart down, using lateral Gs to scale the force if the Kart is moving sideways fast
            float speedFactor = Mathf.Clamp01(rb.velocity.magnitude / maxSpeed);
            float lateralG = Mathf.Abs(Vector3.Dot(rb.velocity, transform.right));
            float downForceFactor = Mathf.Max(speedFactor, lateralG / lateralGScale);
            rb.AddForce(-transform.up * (downForce * rb.mass * downForceFactor));
            
            // Shift Center of Mass
            float speed = rb.velocity.magnitude;
            Vector3 centerOfMassAdjustment = (speed > thresholdSpeed) 
                ? new Vector3(0f, 0f, Mathf.Abs(verticalInput) > 0.1f ? Mathf.Sign(verticalInput) * centerOfMassOffset : 0f)
                : Vector3.zero;
            rb.centerOfMass = originalCenterOfMass + centerOfMassAdjustment;
        }

        private void UpdateAxles(float motor, float steering)
        {
            foreach (AxleInfo axleInfo in axleInfos)
            {
                HandleSteering(axleInfo, steering);
                HandleMotor(axleInfo, motor);
                HandleBrakesAndDrift(axleInfo);
                UpdateWheelVisuals(axleInfo.leftWheel);
                UpdateWheelVisuals(axleInfo.rightWheel);
            }
        }

        private void HandleSteering(AxleInfo axleInfo, float steering)
        {
            if (axleInfo.steering)
            {
                float steeringMultiplier = input.IsBraking ? driftSpeedMultiplier : 1f;
                axleInfo.leftWheel.steerAngle = steering * steeringMultiplier;
                axleInfo.rightWheel.steerAngle = steering * steeringMultiplier;
            }
        }

        private void HandleMotor(AxleInfo axleInfo, float motor)
        {
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
        }

        private void HandleBrakesAndDrift(AxleInfo axleInfo)
        {
            if (axleInfo.motor)
            {
                if (input.IsBraking)
                {
                    rb.constraints = RigidbodyConstraints.FreezeRotationX;
                    
                    float newZ = Mathf.SmoothDamp(rb.velocity.z, 0, ref brakeVelocity, 1f);
                    rb.velocity = rb.velocity.With(z: newZ);

                    axleInfo.leftWheel.brakeTorque = brakeTorque;
                    axleInfo.rightWheel.brakeTorque = brakeTorque;
                    ApplyDriftFriction(axleInfo.leftWheel);
                    ApplyDriftFriction(axleInfo.rightWheel);
                }
                else
                {
                    rb.constraints = RigidbodyConstraints.None;
                    axleInfo.leftWheel.brakeTorque = 0f;
                    axleInfo.rightWheel.brakeTorque = 0f;
                    ResetDriftFriction(axleInfo.leftWheel);
                    ResetDriftFriction(axleInfo.rightWheel);
                }
            }
        }

        private void UpdateWheelVisuals(WheelCollider collider)
        {
            if(collider.transform.childCount == 0)
                return;
            
            Transform visualWheel = collider.transform.GetChild(0);
            Vector3 position;
            Quaternion rotation;
            collider.GetWorldPose(out position, out rotation);
            visualWheel.transform.position = position;
            visualWheel.transform.rotation = rotation;
        }
        
        void ResetDriftFriction(WheelCollider wheel) {
            AxleInfo axleInfo = axleInfos.FirstOrDefault(axle => axle.leftWheel == wheel || axle.rightWheel == wheel);
            if (axleInfo == null) return;
            
            wheel.forwardFriction = axleInfo.originalForwardFriction;
            wheel.sidewaysFriction = axleInfo.originalSidewayFriction;
        }

        void ApplyDriftFriction(WheelCollider wheel) {
            if (wheel.GetGroundHit(out var hit)) {
                wheel.forwardFriction = UpdateFriction(wheel.forwardFriction);
                wheel.sidewaysFriction = UpdateFriction(wheel.sidewaysFriction);
                IsGrounded = true;
            }
        }
        
        WheelFrictionCurve UpdateFriction(WheelFrictionCurve friction) {
            friction.stiffness = input.IsBraking ? Mathf.SmoothDamp(friction.stiffness, .5f, ref driftVelocity, Time.deltaTime * 2f) : 1f;
            return friction;
        }

 #endregion

        
        
        
        
        private float AdjustInput(float input) { 
            return input switch {
                >= .7f => 1f,
                <= -.7f => -1f,
                _ => input
            }; 
        }

        public void ResetKart(Transform transform)
        {
            this.transform.position = transform.position;
            this.transform.rotation = transform.rotation;
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            foreach (WheelCollider wheel in GetComponentsInChildren<WheelCollider>())
            {
                wheel.steerAngle = 0f;;
                wheel.motorTorque = 0f;
            }
            
        }
        
    }
}
