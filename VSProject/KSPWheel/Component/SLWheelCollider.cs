using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using UnityEngine;

namespace KSPWheel
{
    [AddComponentMenu("Physics/SLWheelCollider")]
    public class SLWheelCollider : MonoBehaviour
    {

        #region REGION - Public Accessible values

        /// <summary>
        /// The game object this script should be attached to / affect, set from constructor
        /// </summary>
        public GameObject wheel;

        // TODO really should be read-only, being grabbed from wheel game object when collider is initialized
        // but silly KSP doesn't have RB's on the PART during MODULE initialization; needs to be delayed until first fixed update at least?
        /// <summary>
        /// The rigidbody that this wheel will apply forces to and sample velocity from, set from constructor
        /// </summary>
        public Rigidbody rigidBody;

        #endregion ENDREGION - Public Accessible values

        #region REGION - Private variables

        //externally set values
        public float wheelMass = 1f;
        public float wheelRadius = 0.52f;
        public float suspensionDistance = 0.7f;
        public float suspensionSpring = 35f;
        public float suspensionDamper = 2f;
        public float fwdFrictionCoef = 1f;
        public float sideFrictionCoef = 1f;
        public float surfaceFrictionCoef = 1f;
        public float steerAngle = 0f;
        public float motorTorque = 0f;
        public float brakeTorque = 0f;
        public float momentOfInertia = 1.0f * 0.5f * 0.5f * 0.5f; //moment of inertia of wheel; used for mass in acceleration calculations regarding wheel angular velocity.  MOI of a solid cylinder = ((m*r*r)/2)
        public int raycastMask = ~(1 << 26); //default cast to all layers except 26; 1<<26 sets 26 to the layer; ~inverts all bits in the mask (26 = KSP WheelColliderIgnore layer)
        public KSPWheelFrictionType frictionModel = KSPWheelFrictionType.STANDARD;
        public KSPWheelSweepType sweepType = KSPWheelSweepType.SPHERE;

        //joint-based suspension variables
        //stuff shamelessly pasted from my joint class
        //TODO: I don't know what I'm doing here. Should some of these be GET/SET methods? 
        public ConfigurableJoint susJoint;
        public GameObject contact;
        public Rigidbody contactRb;
        public Vector3 lastHitPoint;
        public float lateralGrip = 2000;
        public RaycastHit hit;
        public JointDrive suspensionSetting;

        //internal friction model values
        public float currentSuspensionCompression = 0f;

        public bool isGrounded = false;

        private Vector3 contactForward, contactRight;
        private Vector3 wheelUp;
        private Vector3 wheelForward;
        private Vector3 wheelRight;
        public Vector3 localVelocity;
        public Vector3 localForce;
        private float wWheel = 0f;//angular velocity of wheel; rotations in radians per second
        private float vWheel;
        private float vWheelDelta;
        private float sLong;
        private float sLat;
        private Vector3 hitPoint;//world-space position of contact patch
        private Vector3 hitNormal;
        private Collider hitCollider;
        private float inertiaInverse;//cached inertia inverse used to eliminate division operations from per-tick update code
        private float radiusInverse;//cached radius inverse used to eliminate division operations from per-tick update code
        private float massInverse;//cached mass inverse used to eliminate division operations from per-tick update code

        private Action<Vector3> onImpactCallback;//simple blind callback for when the wheel changes from !grounded to grounded, the input variable is the wheel-local impact velocity

        private KSPWheelFrictionCurve fwdFrictionCurve;//current forward friction curve
        private KSPWheelFrictionCurve sideFrictionCurve;//current sideways friction curve

        #endregion ENDREGION - Private variables

        #region REGION - Public accessible methods, Constructor, API get/set methods

        public float rpm
        {
            // wWheel / (pi*2) * 60f
            // all values converted to combined constants
            get { return wWheel * 9.549296585f; }
        }

        public float perFrameRotation
        {
            // returns rpm * 0.16666_ * 360f * secondsPerFrame
            // degrees per frame = (rpm / 60) * 360 * secondsPerFrame
            get { return rpm * 6 * Time.deltaTime; }
        }

        /// <summary>
        /// Initialize a wheel-collider object for the given GameObject (the wheel collider), and the given rigidbody (the RB that the wheel-collider will apply forces to)<para/>
        /// -Both- must be valid references (i.e. cannot be null)
        /// </summary>
        public bool init()
        {
            wheel = this.gameObject;
            if (wheel == null)
            {
                throw new NullReferenceException("Wheel game object for WheelCollider may not be null!");
                //a throw includes an implicit return statement
            }
            //rigidBody = this.gameObject.GetComponentInParent<Rigidbody>();
            if (rigidBody == null)
            {
                throw new NullReferenceException("Rigidbody for wheel collider may not be null!");
            }
            //default friction curves; may be set to custom curves through the get/set methods below
            sideFrictionCurve = new KSPWheelFrictionCurve(0.06f, 1.2f, 0.08f, 1.0f, 0.65f);
            fwdFrictionCurve = new KSPWheelFrictionCurve(0.06f, 1.2f, 0.08f, 1.0f, 0.65f);
            return true;
        }

        /// <summary>
        /// Seat the reference to the wheel-impact callback.  This method will be called when the wheel first contacts the surface, passing in the wheel-local impact velocity (impact force is unknown)
        /// </summary>
        /// <param name="callback"></param>
        public void setImpactCallback(Action<Vector3> callback)
        {
            onImpactCallback = callback;
        }

        public void Start()
        {
            if (!init())
            {
                throw new NullReferenceException("Init failed Could not find correct components");
            }
            SetupJoint();
            StartCoroutine(WaitForFixed()); // Start FixedUpdate when we're ready. Prevents FixedUpdate running before. Disable wheel by stopping
        }

        /// <summary>
        /// A FixedUpdate coroutine that calls suspension and grip code. If stopped, no raycasting is done and the wheels stop working.
        /// </summary>
        /// <returns></returns>
        IEnumerator WaitForFixed()
        {
            yield return new WaitForFixedUpdate();
            while (true)
            {
                bool prevGrounded = UpdateSuspension();
                UpdateWheel();
                
                if (isGrounded && !prevGrounded && onImpactCallback != null)//if was not previously grounded, call-back with impact data; we really only know the impact velocity
                {
                    onImpactCallback.Invoke(localVelocity);
                }

                yield return new WaitForFixedUpdate();
            }
        }

        /// <summary>
        /// Called from Start(). Sets up suspension joint and raycast
        /// </summary>
        /// <returns></returns>
        private void SetupJoint()
        {
            raycastMask = 1 << 26;
            raycastMask = ~raycastMask;

            if (wheel.transform.localScale != Vector3.one)
            {
                print("The GameObject this script is aplied to should be set to scale 1,1,1!!!!");
            }
            Vector3 ITP = rigidBody.transform.InverseTransformPoint(wheel.transform.position); //correct
            print("ITP " + ITP);

            // Needs to be an empty GO eventually - Sphere for convenience for now. Has to have Rigidybody for joint to attach to.
            contact = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            var visual = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            Destroy(visual.GetComponent("SphereCollider"));
            visual.transform.parent = contact.transform;
            visual.transform.localPosition = Vector3.zero;
            Destroy(contact.GetComponent("SphereCollider"));
            Destroy(contact.GetComponent("MeshRenderer"));
            contact.layer = 26;
            contact.transform.position = wheel.transform.position + -wheel.transform.up * (suspensionDistance + wheelRadius); //working
            contact.transform.rotation = wheel.transform.rotation;
            contactRb = contact.AddComponent<Rigidbody>();
            contactRb.mass = 0;

            susJoint = rigidBody.gameObject.AddComponent<ConfigurableJoint>();
            Vector3 worldSpaceContactPosition = wheel.transform.position + wheel.transform.up * (suspensionDistance + wheelRadius);
            print("WSCP " + worldSpaceContactPosition);
            Vector3 relative = rigidBody.transform.InverseTransformDirection(rigidBody.transform.position - worldSpaceContactPosition);
            print("relative " + relative);
            susJoint.anchor = relative;
            susJoint.axis = new Vector3(1, 0, 0);
            susJoint.autoConfigureConnectedAnchor = false;
            susJoint.secondaryAxis = new Vector3(0, 1, 0);

            susJoint.xMotion = ConfigurableJointMotion.Limited; //lateral grip
            susJoint.yMotion = ConfigurableJointMotion.Limited; //suspension
            susJoint.zMotion = ConfigurableJointMotion.Free;

            susJoint.angularXMotion = ConfigurableJointMotion.Free;
            susJoint.angularYMotion = ConfigurableJointMotion.Free; //steering
            susJoint.angularZMotion = ConfigurableJointMotion.Free;
            susJoint.targetPosition = new Vector3(0, 0, 0); //essentially where to suspend to

            var SJLS = new SoftJointLimitSpring();
            SJLS.spring = 0;
            SJLS.damper = 0;
            susJoint.linearLimitSpring = SJLS;

            var suspensionLimit = new SoftJointLimit();
            suspensionLimit.bounciness = 0;
            suspensionLimit.limit = suspensionDistance; //this sets the hard limit, or what are often called bump stops.
            suspensionLimit.contactDistance = suspensionDistance;
            susJoint.linearLimit = suspensionLimit;

            var XD = new JointDrive();
            XD.positionSpring = 000;
            XD.positionDamper = 00;
            XD.maximumForce = 1000;
            susJoint.xDrive = XD;

            //THIS CONTROLS SUSPENSION SETTINGS
            suspensionSetting = new JointDrive();
            suspensionSetting.positionSpring = suspensionSpring;
            suspensionSetting.positionDamper = suspensionDamper;
            suspensionSetting.maximumForce = 10000000;
            susJoint.yDrive = suspensionSetting;

            susJoint.connectedBody = contactRb; //Attached the joint to the contact object

            susJoint.connectedAnchor = Vector3.zero;
        }

        /// <summary>
        /// Updates suspension values, fires the raycast and manipulates suspension contact point for the spring to act against.
        /// </summary>
        /// <returns>bool isGrounded</returns>
        private bool UpdateSuspension()
        {
            suspensionSetting.positionSpring = suspensionSpring;
            suspensionSetting.positionDamper = suspensionDamper;
            suspensionSetting.maximumForce = 10000000;
            susJoint.yDrive = suspensionSetting;

            susJoint.targetRotation = Quaternion.Euler(0, steerAngle, 0);

            
            if (checkSuspensionContact())
            {
                contactRb.isKinematic = true;
                contactRb.position = wheel.transform.position + (-wheel.transform.up * hit.distance);
                contactRb.rotation = wheel.transform.rotation;
                //print(hit.collider.gameObject.name);
                lastHitPoint = hit.point;
            }
            else
            {
                contactRb.position = wheel.transform.position + (-wheel.transform.up * (suspensionDistance + wheelRadius)); //move the object to the end of suspension travel
                contactRb.rotation = wheel.transform.rotation;
                contactRb.isKinematic = false;
            }
            return isGrounded;
        }

        /// <summary>
        /// Now called from WaitForFixed() <para/>
        /// 
        /// 
        /// </summary>
        private void UpdateWheel()
        {
            wheelForward = Quaternion.AngleAxis(steerAngle, wheel.transform.up) * wheel.transform.forward;
            wheelUp = wheel.transform.up;
            wheelRight = -Vector3.Cross(wheelForward, wheelUp);
            if (isGrounded)
            {
                //most of this was cached/updated through encapsulated setter methods... but can't do that with public fields for the Unity editor
                momentOfInertia = wheelMass * wheelRadius * wheelRadius * 0.5f;
                radiusInverse = 1.0f / wheelRadius;
                inertiaInverse = 1.0f / momentOfInertia;
                massInverse = 1.0f / wheelMass;

                localForce.y = suspensionSpring * currentSuspensionCompression;

                contactRight = Vector3.Cross(hitNormal, wheelForward);
                contactForward = -Vector3.Cross(hitNormal, contactRight);

                contactForward = wheelForward - hitNormal * Vector3.Dot(wheelForward, hitNormal);
                contactRight = Vector3.Cross(hitNormal, contactForward);

                Vector3 worldVelocityAtHit = rigidBody.GetPointVelocity(hitPoint);
                float mag = worldVelocityAtHit.magnitude;
                localVelocity.z = Vector3.Dot(worldVelocityAtHit.normalized, contactForward) * mag;
                localVelocity.x = Vector3.Dot(worldVelocityAtHit.normalized, contactRight) * mag;
                localVelocity.y = Vector3.Dot(worldVelocityAtHit.normalized, hitNormal) * mag;

                calcFriction();

                float d1 = Vector3.Dot(wheel.transform.forward, hitNormal);
                float d2 = Vector3.Dot(wheel.transform.right, hitNormal);

                Vector3 calculatedForces = localForce.z * contactForward;
                calculatedForces += localForce.x * contactRight;

                calculatedForces += d1 * localForce.y * wheel.transform.forward;
                calculatedForces += d2 * localForce.y * wheel.transform.right;

                rigidBody.AddForceAtPosition(calculatedForces, hitPoint, ForceMode.Force);
                if (hitCollider.attachedRigidbody != null && !hitCollider.attachedRigidbody.isKinematic)
                {
                    hitCollider.attachedRigidbody.AddForceAtPosition(-calculatedForces, hitPoint, ForceMode.Force);
                }

            }
            else
            {
                integrateUngroundedTorques();
                isGrounded = false;
                localForce = Vector3.zero;
                hitNormal = Vector3.zero;
                hitPoint = Vector3.zero;
                hitCollider = null;
                localVelocity = Vector3.zero;
            }

        }

        #endregion ENDREGION - Public accessible methods, API get/set methods

        #region REGION - Private/internal update methods

        /// <summary>
        /// Integrate drive and brake torques into wheel velocity for when -not- grounded.
        /// This allows for wheels to change velocity from user input while the vehicle is not in contact with the surface.
        /// Not-yet-implemented are torques on the rigidbody due to wheel accelerations.
        /// </summary>
        private void integrateUngroundedTorques()
        {
            //velocity change due to motor; if brakes are engaged they can cancel this out the same tick
            //acceleration is in radians/second; only operating on fixedDeltaTime seconds, so only update for that length of time
            wWheel += motorTorque * inertiaInverse * Time.fixedDeltaTime;
            // maximum torque exerted by brakes onto wheel this frame
            float wBrake = brakeTorque * inertiaInverse * Time.fixedDeltaTime;
            // clamp the max brake angular change to the current angular velocity
            wBrake = Mathf.Min(Mathf.Abs(wWheel), wBrake);
            // sign it opposite of current wheel spin direction
            // and finally, integrate it into wheel angular velocity
            wWheel += wBrake * -Mathf.Sign(wWheel);
        }

        /// <summary>
        /// Uses either ray- or sphere-cast to check for suspension contact with the ground, calculates current suspension compression, and caches the world-velocity at the contact point
        /// </summary>
        /// <returns></returns>
        private bool checkSuspensionContact()
        {
            switch (sweepType)
            {
                case KSPWheelSweepType.RAY:
                    return suspensionSweepRaycast();
                case KSPWheelSweepType.SPHERE:
                    return suspensionSweepSpherecast();
                case KSPWheelSweepType.CAPSULE:
                    return suspensionSweepCapsuleCast();
                default:
                    return suspensionSweepRaycast();
            }
        }

        /// <summary>
        /// Check suspension contact using a ray-cast; return true/false for if contact was detected
        /// </summary>
        /// <returns></returns>
        private bool suspensionSweepRaycast()
        {
            //if (Physics.Raycast(wheel.transform.position, -wheel.transform.up, out hit, suspensionLength + wheelRadius, raycastMask))
            if (Physics.Raycast(wheel.transform.position, -wheel.transform.up, out hit, suspensionDistance + wheelRadius, raycastMask))
            {
                currentSuspensionCompression = suspensionDistance + wheelRadius - hit.distance;
                hitNormal = hit.normal;
                hitCollider = hit.collider;
                susJoint.connectedAnchor = Vector3.zero;
                hitPoint = hit.point;
                isGrounded = true;
                return true;
            }
            isGrounded = false;
            return false;
        }

        /// <summary>
        /// Check suspension contact using a sphere-cast; return true/false for if contact was detected.
        /// </summary>
        /// <returns></returns>
        private bool suspensionSweepSpherecast()
        {
            //need to start cast above max-compression point, to allow for catching the case of @ bump-stop
            float rayOffset = wheelRadius;
            if (Physics.SphereCast(wheel.transform.position + wheel.transform.up * wheelRadius, wheelRadius, -wheel.transform.up, out hit, suspensionDistance + wheelRadius, raycastMask))
            {
                currentSuspensionCompression = suspensionDistance + rayOffset - hit.distance;
                hitNormal = hit.normal;
                hitCollider = hit.collider;
                hitPoint = hit.point;
                isGrounded = true;
                Debug.DrawLine(hit.point, hit.point - hit.normal, Color.red);

                return true;
            }
            isGrounded = false;
            return false;
        }

        //TODO config specified 'wheel width' (perhaps grab from wheel-mesh local-bounds)
        //TODO config specified number of capsules
        /// <summary>
        /// less efficient and less optimal solution for skinny wheels, but avoids the edge cases caused by sphere colliders<para/>
        /// uses 2 capsule-casts in a V shape downward for the wheel instead of a sphere; 
        /// for some collisions the wheel may push into the surface slightly, up to about 1/3 radius.  
        /// Could be expanded to use more capsules at the cost of performance, but at increased collision fidelity, by simulating more 'edges' of a n-gon circle.  
        /// Sadly, unity lacks a collider-sweep function, or this could be a bit more efficient.
        /// </summary>
        /// <returns></returns>
        private bool suspensionSweepCapsuleCast()
        {
            //create two capsule casts in a v-shape
            //take whichever collides first
            float wheelWidth = 0.3f;
            float capRadius = wheelWidth * 0.5f;

            RaycastHit hit;
            RaycastHit hit1;
            RaycastHit hit2;
            bool hit1b;
            bool hit2b;
            Vector3 startPos = wheel.transform.position;
            float rayOffset = wheelRadius;
            float rayLength = suspensionDistance + rayOffset;
            float capLen = wheelRadius - capRadius;
            Vector3 worldOffset = wheel.transform.up * rayOffset;//offset it above the wheel by a small amount, in case of hitting bump-stop
            Vector3 capEnd1 = wheel.transform.position + wheel.transform.forward * capLen;
            Vector3 capEnd2 = wheel.transform.position - wheel.transform.forward * capLen;
            Vector3 capBottom = wheel.transform.position - wheel.transform.up * capLen;
            hit1b = Physics.CapsuleCast(capEnd1 + worldOffset, capBottom + worldOffset, capRadius, -wheel.transform.up, out hit1, rayLength, raycastMask);
            hit2b = Physics.CapsuleCast(capEnd2 + worldOffset, capBottom + worldOffset, capRadius, -wheel.transform.up, out hit2, rayLength, raycastMask);
            if (hit1b || hit2b)
            {
                if (hit1b && hit2b) { hit = hit1.distance < hit2.distance ? hit1 : hit2; }
                else if (hit1b) { hit = hit1; }
                else if (hit2b) { hit = hit2; }
                else
                {
                    hit = hit1;
                }
                currentSuspensionCompression = suspensionDistance + rayOffset - hit.distance;
                hitNormal = hit.normal;
                hitCollider = hit.collider;
                hitPoint = hit.point;
                isGrounded = true;
                return true;
            }
            isGrounded = false;
            return false;
        }

        #region REGION - Friction model shared functions

        private void calcFriction()
        {
            switch (frictionModel)
            {
                case KSPWheelFrictionType.STANDARD:
                    calcFrictionStandard();
                    break;
                case KSPWheelFrictionType.PACEJKA:
                    calcFrictionPacejka();
                    break;
                case KSPWheelFrictionType.PHSYX:
                    calcFrictionPhysx();
                    break;
                default:
                    calcFrictionStandard();
                    break;
            }
        }

        /// <summary>
        /// Returns a slip ratio between 0 and 1, 0 being no slip, 1 being lots of slip
        /// </summary>
        /// <param name="vLong"></param>
        /// <param name="vWheel"></param>
        /// <returns></returns>
        private float calcLongSlip(float vLong, float vWheel)
        {
            float sLong = 0;
            if (vLong == 0 && vWheel == 0) { return 0f; }//no slip present
            float a = Mathf.Max(vLong, vWheel);
            float b = Mathf.Min(vLong, vWheel);
            sLong = (a - b) / Mathf.Abs(a);
            sLong = Mathf.Clamp(sLong, 0, 1);
            return sLong;
        }

        /// <summary>
        /// Returns a slip ratio between 0 and 1, 0 being no slip, 1 being lots of slip
        /// </summary>
        /// <param name="vLong"></param>
        /// <param name="vLat"></param>
        /// <returns></returns>
        private float calcLatSlip(float vLong, float vLat)
        {
            float sLat = 0;
            if (vLat == 0)//vLat = 0, so there can be no sideways slip
            {
                return 0f;
            }
            else if (vLong == 0)//vLat!=0, but vLong==0, so all slip is sideways
            {
                return 1f;
            }
            sLat = Mathf.Abs(Mathf.Atan(vLat / vLong));//radians
            sLat = sLat * Mathf.Rad2Deg;//degrees
            sLat = sLat / 90f;//percentage (0 - 1)
            return sLat;
        }

        #endregion ENDREGION - Friction calculations methods based on alternate

        #region REGION - Standard Friction Model
        // based on : http://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html

        public void calcFrictionStandard()
        {
            //initial motor/brake torque integration, brakes integrated further after friction applied
            //motor torque applied directly
            wWheel += motorTorque * inertiaInverse * Time.fixedDeltaTime;//acceleration is in radians/second; only operating on 1 * fixedDeltaTime seconds, so only update for that length of time
                                                                         // maximum torque exerted by brakes onto wheel this frame
            float wBrakeMax = brakeTorque * inertiaInverse * Time.fixedDeltaTime;
            // clamp the max brake angular change to the current angular velocity
            float wBrake = Mathf.Min(Mathf.Abs(wWheel), wBrakeMax);
            // sign it opposite of current wheel spin direction
            // and finally, integrate it into wheel angular velocity
            wWheel += wBrake * -Mathf.Sign(wWheel);
            // this is the remaining brake angular acceleration/torque that can be used to counteract wheel acceleration caused by traction friction
            float wBrakeDelta = wBrakeMax - wBrake;

            vWheel = wWheel * wheelRadius;
            sLong = calcLongSlip(localVelocity.z, vWheel);
            sLat = calcLatSlip(localVelocity.z, localVelocity.x);
            vWheelDelta = vWheel - localVelocity.z;

            float fLongMax = fwdFrictionCurve.evaluate(sLong) * localForce.y * fwdFrictionCoef * surfaceFrictionCoef;
            float fLatMax = sideFrictionCurve.evaluate(sLat) * localForce.y * sideFrictionCoef * surfaceFrictionCoef;

            // TODO - this should actually be limited by the amount of force necessary to arrest the velocity of this wheel in this frame
            // so limit max should be (abs(vLat) * sprungMass) / Time.fixedDeltaTime  (in newtons)
            localForce.x = fLatMax;
            // using current down-force as a 'sprung-mass' to attempt to limit overshoot when bringing the velocity to zero; the 2x multiplier is just because it helped with response but didn't induce oscillations; higher multipliers can
            if (localForce.x > Mathf.Abs(localVelocity.x) * localForce.y * 2f) { localForce.x = Mathf.Abs(localVelocity.x) * localForce.y * 2f; }
            // if (fLat > sprungMass * Mathf.Abs(vLat) / Time.fixedDeltaTime) { fLat = sprungMass * Mathf.Abs(vLat) * Time.fixedDeltaTime; }
            localForce.x *= -Mathf.Sign(localVelocity.x);// sign it opposite to the current vLat

            //angular velocity delta between wheel and surface in radians per second; radius inverse used to avoid div operations
            float wDelta = vWheelDelta * radiusInverse;
            //amount of torque needed to bring wheel to surface speed over one second
            float tDelta = wDelta * momentOfInertia;
            //newtons of force needed to bring wheel to surface speed over one second; radius inverse used to avoid div operations
            // float fDelta = tDelta * radiusInverse; // unused
            //absolute value of the torque needed to bring the wheel to road speed instantaneously/this frame
            float tTractMax = Mathf.Abs(tDelta) / Time.fixedDeltaTime;
            //newtons needed to bring wheel to ground velocity this frame; radius inverse used to avoid div operations
            float fTractMax = tTractMax * radiusInverse;
            //final maximum force value is the smallest of the two force values;
            // if fTractMax is used the wheel will be brought to surface velocity,
            // otherwise fLongMax is used and the wheel is still slipping but maximum traction force will be exerted
            fTractMax = Mathf.Min(fTractMax, fLongMax);
            // convert the clamped traction value into a torque value and apply to the wheel
            float tractionTorque = fTractMax * wheelRadius * -Mathf.Sign(vWheelDelta);
            // and set the longitudinal force to the force calculated for the wheel/surface torque
            localForce.z = fTractMax * Mathf.Sign(vWheelDelta);
            //use wheel inertia to determine final wheel acceleration from torques; inertia inverse used to avoid div operations; convert to delta-time, as accel is normally radians/s
            float angularAcceleration = tractionTorque * inertiaInverse * Time.fixedDeltaTime;
            //apply acceleration to wheel angular velocity
            wWheel += angularAcceleration;
            //second integration pass of brakes, to allow for locked-wheels after friction calculation
            if (Mathf.Abs(wWheel) < wBrakeDelta)
            {
                wWheel = 0;
                wBrakeDelta -= Mathf.Abs(wWheel);
                float fMax = Mathf.Max(0, Mathf.Abs(fLongMax) - Mathf.Abs(localForce.z));//remaining 'max' traction left
                float fMax2 = Mathf.Max(0, localForce.y * Mathf.Abs(localVelocity.z) * 2 - Mathf.Abs(localForce.z));
                float fBrakeMax = Mathf.Min(fMax, fMax2);
                localForce.z += fBrakeMax * -Mathf.Sign(localVelocity.z);
            }
            else
            {
                wWheel += -Mathf.Sign(wWheel) * wBrakeDelta;//traction from this will be applied next frame from wheel slip, but we're integrating here basically for rendering purposes
            }

            // cap friction / combined friction
            // in this simplified model, longitudinal force wins
            float cap = Mathf.Max(fLatMax, fLongMax);
            float latLimit = cap - Mathf.Abs(localForce.z);
            if (Mathf.Abs(localForce.x) > latLimit) { localForce.x = latLimit * Mathf.Sign(localForce.x); }
        }

        #endregion ENDREGION - Standard Friction Model

        #region REGION - Alternate Friction Model - Pacejka
        // based on http://www.racer.nl/reference/pacejka.htm
        // and also http://www.mathworks.com/help/physmod/sdl/ref/tireroadinteractionmagicformula.html?requestedDomain=es.mathworks.com
        // and http://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
        // and http://www.edy.es/dev/2011/12/facts-and-myths-on-the-pacejka-curves/
        // and http://www-cdr.stanford.edu/dynamic/bywire/tires.pdf

        public void calcFrictionPacejka()
        {
            // TODO
            // really this should just be an adjustment to the curve parameters
            // as all that the pacejka formulas do is define the curves used by slip ratio to calculate maximum force output

            vWheel = wWheel * wheelRadius;
            sLong = calcLongSlip(localVelocity.z, vWheel);
            sLat = calcLatSlip(localVelocity.z, localVelocity.x);
            vWheelDelta = vWheel - localVelocity.z;

            // 'simple' magic-formula
            float B = 10f;//stiffness
                          // float C = 1.9f;
            float Clat = 1.3f;
            float Clong = 1.65f;
            float D = 1;
            float E = 0.97f;
            // F = Fz * D * sin(C * atan(B*slip - E * (B*slip - atan(B*slip))))
            float Fz = localForce.y;
            float slipLat = sLat * 100f;
            float slipLong = sLong * 100f;
            float fLatMax = localForce.x = Fz * D * Mathf.Sin(Clat * Mathf.Atan(B * slipLat - E * (B * slipLat - Mathf.Atan(B * slipLat))));
            float fLongMax = localForce.z = Fz * D * Mathf.Sin(Clong * Mathf.Atan(B * slipLong - E * (B * slipLong - Mathf.Atan(B * slipLong))));

            if (localForce.x > Mathf.Abs(localVelocity.x) * localForce.y * 2f) { localForce.x = Mathf.Abs(localVelocity.x) * localForce.y * 2f; }
            localForce.x *= -Mathf.Sign(localVelocity.x);// sign it opposite to the current vLat

            //angular velocity delta between wheel and surface in radians per second; radius inverse used to avoid div operations
            float wDelta = vWheelDelta * radiusInverse;
            //amount of torque needed to bring wheel to surface speed over one second
            float tDelta = wDelta * momentOfInertia;
            //newtons of force needed to bring wheel to surface speed over one update tick
            float fDelta = tDelta * radiusInverse / Time.fixedDeltaTime;
            localForce.z = Mathf.Min(Mathf.Abs(fDelta), localForce.z) * Mathf.Sign(fDelta);
            float tTract = -localForce.z * wheelRadius;
            wWheel += tTract * Time.fixedDeltaTime * inertiaInverse;
            wWheel += motorTorque * Time.fixedDeltaTime * inertiaInverse;

            float cap = Mathf.Max(fLatMax, fLongMax);
            float latLimit = cap - Mathf.Abs(localForce.z);
            if (Mathf.Abs(localForce.x) > latLimit) { localForce.x = latLimit * Mathf.Sign(localForce.x); }
        }

        #endregion ENDREGION - Alternate friction model

        #region REGION - Alternate Friction Model - PhysX

        // TODO
        // based on http://www.eggert.highpeakpress.com/ME485/Docs/CarSimEd.pdf
        public void calcFrictionPhysx()
        {
            calcFrictionStandard();
        }

        #endregion ENDREGION - Alternate Friction Model 2

        public void drawDebug()
        {
            if (!isGrounded) { return; }

            Vector3 rayStart, rayEnd;
            Vector3 vOffset = rigidBody.velocity * Time.fixedDeltaTime;

            //draw the force-vector line
            rayStart = hitPoint;
            //because localForce isn't really a vector... its more 3 separate force-axis combinations...
            rayEnd = hitNormal * localForce.y;
            rayEnd += contactRight * localForce.x;
            rayEnd += contactForward * localForce.z;
            rayEnd += rayStart;

            //rayEnd = rayStart + wheel.transform.TransformVector(localForce.normalized) * 2f;
            Debug.DrawLine(rayStart + vOffset, rayEnd + vOffset, Color.magenta);

            rayStart += wheel.transform.up * 0.1f;
            rayEnd = rayStart + contactForward * 10f;
            Debug.DrawLine(rayStart + vOffset, rayEnd + vOffset, Color.blue);

            rayEnd = rayStart + contactRight * 10f;
            Debug.DrawLine(rayStart + vOffset, rayEnd + vOffset, Color.red);
        }

        public void LateUpdate()
        {
            drawDebug();
        }

        /// <summary>
        /// Display a visual representation of the wheel in the editor. Unity has no inbuilt gizmo for 
        /// circles, so a sphere is used. Unlike the original WC, I've represented the wheel at top and bottom 
        /// of suspension travel
        /// </summary>
        void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(gameObject.transform.position, wheelRadius);
            Vector3 pos2 = gameObject.transform.position + -gameObject.transform.up * suspensionDistance;
            // if ( != null) { pos2 += gameObject.transform.up * wheelCollider.compressionDistance; }
            Gizmos.DrawWireSphere(pos2, wheelRadius);
            Gizmos.DrawRay(gameObject.transform.position - gameObject.transform.up * wheelRadius, -gameObject.transform.up * suspensionDistance);
        }

        #endregion ENDREGION - Private/internal update methods

    }

    public enum KSPWheelSweepType
    {
        RAY,
        SPHERE,
        CAPSULE
    }

    public enum KSPWheelFrictionType
    {
        STANDARD,
        PACEJKA,
        PHSYX
    }

}

