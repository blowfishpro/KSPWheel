using UnityEngine;

namespace KSPWheel
{
    /// <summary>
    /// This class is a wrapper around the KSPWheelCollider class to allow for easier use while debugging in the Unity Editor.<para/>
    /// It will merely instantiate a KSPWheelCollider object and update its internal variables with the ones entered into the Editor Inspector panel.<para/>
    /// Also includes a few display-only variables for debugging in the editor<para/>
    /// Not intended to be useful aside from the intial debugging period, or as a basic example that can be used in the editor; this class has no use in KSP
    /// </summary>
    
    public class KSPWheelComponent : MonoBehaviour
    {

        #region REGION - Unity Editor Inspector Assignable Fields
        // These variables are set onto the KSPWheelCollider object when Start is called,
        // and updated during script OnValidate() to update any changed values from the editor inspector panel

        /// <summary>
        /// The rigidbody that this wheel will apply forces to and sample velocity from
        /// </summary>
        public Rigidbody rigidBody;

        public Transform steeringTransform;

        public Transform suspensionTransform;

        public Transform wheelTransform;

        /// <summary>
        /// The radius of the wheel to simulate; this is the -actual- size to simulate, not a pre-scaled value
        /// </summary>
        public float wheelRadius = 0.5f;

        /// <summary>
        /// The mass of the -wheel- in... kg? tons? NFC
        /// </summary>
        public float wheelMass = 1f;//used to simulate wheel rotational inertia for brakes and friction purposes

        /// <summary>
        /// The length of the suspension travel
        /// </summary>
        public float suspensionLength = 0.5f;

        /// <summary>
        /// The 'target' parameter for suspension; 0 = fully uncompressed, 1 = fully compressed
        /// </summary>
        public float target = 0;

        /// <summary>
        /// The maximum force the suspension will exhert, in newtons
        /// </summary>
        public float spring = 1000;

        /// <summary>
        /// The damping ratio for the suspension spring force
        /// </summary>
        public float damper = 1500;

        /// <summary>
        /// The maximum torque the motor can exhert against the wheel
        /// </summary>
        public float maxMotorTorque = 0;

        /// <summary>
        /// Max RPM limit for wheel; this aids in making sure slips aren't infinite.
        /// Normally the RPM would be limited by the motor redline / max RPM and the current gearing
        /// but simplifying to a singular max-wheel-rpm value for this simple wheel component
        ///   -- yes, even electric motors have a max RPM regardless of power input
        /// </summary>
        public float rpmLimit = 600f;

        /// <summary>
        /// The maximum torque the brakes can exhert against the wheel while attempting to bring its angular velocity to zero
        /// </summary>
        public float maxBrakeTorque = 0;

        /// <summary>
        /// The maximum deflection for the steering of this wheel, in degrees
        /// </summary>
        public float maxSteerAngle = 0;

        /// <summary>
        /// Throttle/motor torque lerp speed
        /// </summary>
        public float throttleResponse = 2;

        /// <summary>
        /// Steering angle lerp speed
        /// </summary>
        public float steeringResponse = 2;

        /// <summary>
        /// Brake torque lerp speed
        /// </summary>
        public float brakeResponse = 2;

        /// <summary>
        /// The forward friction constant (rolling friction)
        /// </summary>
        public float forwardFrictionCoefficient = 1f;

        /// <summary>
        /// The sideways friction constant
        /// </summary>
        public float sideFrictionCoefficient = 1f;

        /// <summary>
        /// Global surface friction coefficient applied to both forward and sideways friction
        /// </summary>
        public float surfaceFrictionCoefficient = 1f;

        public KSPWheelFrictionType frictionModel = KSPWheelFrictionType.STANDARD;

        public KSPWheelSweepType sweepType = KSPWheelSweepType.RAY;
        
        public bool debug = false;

        public float susResp = 1f;

        #endregion ENDREGION - Unity Editor Inspector Assignable Fields

        // these variables are updated every fixed-tick after the wheel has been updated
        // used merely to display some info while in the editor for debugging purposes

        #region REGION - Unity Editor Display-Only Variables

        public Vector3 localVelocity;
        public Vector3 localAcceleration;
        public float rpm;
        public float sLong;
        public float sLat;
        public float fSpring;
        public float fDamp;
        public float fLong;
        public float fLat;
        public float comp;

        public bool suspLock = false;

        #endregion ENDREGION - Unity Editor Display Variables

        private SLWheelCollider wheelCollider;
        public void Start()
        {
            wheelCollider = new SLWheelCollider(gameObject, rigidBody);

            //OnValidate();//manually call to set all current parameters into wheel collider object
        }

        public void FixedUpdate()
        {

            Vector3 targetPos = suspLock ? transform.position - transform.up * suspensionLength : transform.position;

            Vector3 p = Vector3.Lerp(pos, targetPos, Time.fixedDeltaTime);


            wheelCollider.motorTorque = currentMotorTorque;
            wheelCollider.steeringAngle = currentSteer;
            wheelCollider.brakeTorque = currentBrakeTorque;
            wheelCollider.updateWheel();
            if (steeringTransform != null)
            {
                steeringTransform.localRotation = Quaternion.AngleAxis(currentSteer, steeringTransform.up);
            }
            if (suspensionTransform != null)
            {
                suspensionTransform.position = gameObject.transform.position - (suspensionLength - wheelCollider.compressionDistance) * gameObject.transform.up;
            }
            if (wheelTransform != null)
            {
                wheelTransform.Rotate(wheelTransform.right, wheelCollider.perFrameRotation, Space.World);
            }
            Vector3 prevVel = localVelocity;
            localVelocity = wheelCollider.wheelLocalVelocity;
            localAcceleration = (prevVel - localVelocity) / Time.fixedDeltaTime;
            fSpring = wheelCollider.springForce;
            fDamp = wheelCollider.dampForce;
            rpm = wheelCollider.rpm;
            sLong = wheelCollider.longitudinalSlip;
            sLat = wheelCollider.lateralSlip;
            fLong = wheelCollider.longitudinalForce;
            fLat = wheelCollider.lateralForce;
            comp = wheelCollider.compressionDistance;
            if (debug)
            {
                MonoBehaviour.print("s/d: " + fSpring + " : " + fDamp);
            }
        }

        public void OnValidate()
        {
            if (wheelCollider != null)
            {
                wheelCollider.radius = wheelRadius;
                wheelCollider.mass = wheelMass;
                wheelCollider.length = suspensionLength;
                wheelCollider.target = target;
                wheelCollider.spring = spring;
                wheelCollider.damper = damper;
                wheelCollider.motorTorque = maxMotorTorque;
                wheelCollider.brakeTorque = maxBrakeTorque;
                wheelCollider.forwardFrictionCoefficient = forwardFrictionCoefficient;
                wheelCollider.sideFrictionCoefficient = sideFrictionCoefficient;
                wheelCollider.surfaceFrictionCoefficient = surfaceFrictionCoefficient;
                wheelCollider.sweepType = sweepType;
                wheelCollider.frictionModel = frictionModel;
                wheelCollider.susResponse = susResp;

                SphereCollider sc = bumpStopCollider.GetComponent<SphereCollider>();
                bumpStopCollider.layer = 26;
                sc.radius = wheelRadius;
                bumpStopCollider.transform.parent = gameObject.transform;
                bumpStopCollider.transform.localPosition = Vector3.zero;
            }
        }



    }
}
