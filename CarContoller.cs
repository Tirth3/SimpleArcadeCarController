using UnityEngine;
using UnityEngine.UI;

public class CarController : MonoBehaviour
{
    private Transform[] WheelMeshes = new Transform[4];
    private Transform[] RayCastPoints = new Transform[4];
    private Transform AccPoint;
    private Transform CenterOfMass;
    private InputManager IM;
    private Rigidbody RB;
    private Vector3[] SpherePositions = new Vector3[4];
    private bool FixedUpdateFlag = true;
    private AudioSource AudSource;
    private Transform Needle;

    [Header("Motor Settings")]
    public float MotorTorque = 10f;
    public AnimationCurve TorqueCurve;

    public float MaxSpeed;
    public float Velocity;
    public float RPM;
    public ForceMode AccForceMode;

    [Header("Suspension Settings")]
    public float RestLength = 5f;
    public float MaxRayCastDistance = 10f;
    public float WheelRadius = 3f;
    public float SpringStiffness = 2f;
    public float DampingStiffness = 1f;

    [Header("Steering Settings")]
    public float SteerForce = 2f;
    [Range(0 , 1)] public float GripCoefficient = 1f;
    public ForceMode SteerForceMode;

    [Header("Gear Settings")]
    public float[] GearMaxSpeed = new float[4];
    public int CurrentGear = 0;
    public float MaxRPM = 9500f;

    [Header("Other settings")]
    public float Gravity = 10f;
    public LayerMask Driveable;
    public int CastType = 1;
    
    private bool IsGrounded;

    private void Start()
    {
        IM = transform.GetComponent<InputManager>();
        CenterOfMass = transform.Find("CenterOfMass");
        RB = transform.GetComponent<Rigidbody>();
        RB.centerOfMass = CenterOfMass.localPosition;
        AudSource = transform.GetComponent<AudioSource>();

        Transform Meshes = transform.Find("WheelMeshes");
        Transform Points = transform.Find("RayCastPoints");

        for(int i = 1; i < 5; i++)
        {
            WheelMeshes[i - 1] = Meshes.Find("SP" + i);
            RayCastPoints[i - 1] = Points.Find("SP" + i);
        }

        AccPoint = transform.Find("AccPoint");
        Needle = GameObject.Find("Needle").GetComponent<Transform>();
    }

    private void Update()
    {
        if (FixedUpdateFlag)
        {
            if (CastType == 1)
                SuspensionSphereCast();
            if (CastType == 2)
                SuspensionRayCast();

            FixedUpdateFlag = false;     
        }
        UpdateGears();
        UpdateAudio();
        UpdateCanvas();
        UpdateWheelsMeshes();
    }

    private void FixedUpdate()
    {
        FixedUpdateFlag = true;
        Move();
        Steer();
        RB.AddForce(-transform.up * Gravity , ForceMode.Force);
    }
    void Move()
    {
        if (IM.AccelerationInput > 0 && IsGrounded)
        {
            RB.AddForceAtPosition(IM.ReverseGear * transform.forward * MotorTorque * TorqueCurve.Evaluate(Velocity / MaxSpeed), AccPoint.position, AccForceMode);
        }
        if (IM.DeAccelerationInput > 0 && IsGrounded && Velocity > 0.3f)
        {
            RB.AddForceAtPosition(-transform.forward * MotorTorque * IM.ReverseGear, AccPoint.position, AccForceMode);
        }
        if(IM.Handbrake)
            RB.velocity = Vector3.zero;

        if(IsGrounded)
            RB.velocity = Vector3.ClampMagnitude(RB.velocity, MaxSpeed);    
        if (RB.velocity.magnitude < 0.1f)
            RB.velocity = Vector3.zero;
        Velocity = Vector3.Dot(RB.velocity , transform.forward);
        RPM = Mathf.Abs(Velocity / MaxSpeed) / (2 * Mathf.PI * WheelRadius) * 30000;
    }
    void SuspensionSphereCast()
    {
        for(int i = 0; i < WheelMeshes.Length; i++)
        {
            RaycastHit hit;
            bool IsHit = Physics.SphereCast(RayCastPoints[i].position, WheelRadius, -RayCastPoints[i].up, out hit, MaxRayCastDistance, Driveable);
            if (IsHit)
            {
                SpherePositions[i] = hit.point + RayCastPoints[i].up * WheelRadius;

                float SpringCompression = RestLength - hit.distance;
                float SpringForce = SpringCompression * SpringStiffness;

                float SpringVelocity = Vector3.Dot(RB.GetPointVelocity(RayCastPoints[i].position), RayCastPoints[i].up);
                float DampingForce = SpringVelocity * DampingStiffness;
                float TotalForce = SpringForce - DampingForce;

                RB.AddForceAtPosition(RayCastPoints[i].up * TotalForce, RayCastPoints[i].position, ForceMode.Force); ;

                Debug.DrawLine(RayCastPoints[i].position, hit.point + RayCastPoints[i].up * WheelRadius, Color.red);
                IsGrounded = true;
            }
            else
            {
                SpherePositions[i] = -RayCastPoints[i].up * MaxRayCastDistance + RayCastPoints[i].position;
                Debug.DrawLine(RayCastPoints[i].position, -RayCastPoints[i].up * MaxRayCastDistance + RayCastPoints[i].position , Color.green);
                IsGrounded = false;
            }
        }
    }
    void SuspensionRayCast()
    {
        for (int i = 0; i < WheelMeshes.Length; i++)
        {
            RaycastHit hit;
            bool IsHit = Physics.Raycast(RayCastPoints[i].position, -RayCastPoints[i].up, out hit, MaxRayCastDistance + WheelRadius, Driveable);
            if (IsHit)
            {
                SpherePositions[i] = hit.point + RayCastPoints[i].up * WheelRadius;

                float SpringCompression = RestLength - (hit.distance - WheelRadius);
                float SpringForce = SpringCompression * SpringStiffness;

                float SpringVelocity = Vector3.Dot(RB.GetPointVelocity(RayCastPoints[i].position), RayCastPoints[i].up);
                float DampingForce = SpringVelocity * DampingStiffness;
                float TotalForce = SpringForce - DampingForce;

                RB.AddForceAtPosition(RayCastPoints[i].up * TotalForce, RayCastPoints[i].position, ForceMode.Force); ;

                Debug.DrawLine(RayCastPoints[i].position, hit.point + RayCastPoints[i].up * WheelRadius, Color.red);
                IsGrounded = true;
            }
            else
            {
                SpherePositions[i] = -RayCastPoints[i].up * MaxRayCastDistance + RayCastPoints[i].position;
                Debug.DrawLine(RayCastPoints[i].position, -RayCastPoints[i].up * MaxRayCastDistance + RayCastPoints[i].position, Color.green);
                IsGrounded = false;
            }
        }
    }
    void Steer()
    {
        if (Mathf.Abs(Velocity) > 0.3f)
        {
            Vector3 SteerDir = transform.right;
            Vector3 WorldVelocity = RB.velocity;

            float SteeringVel = Vector3.Dot(SteerDir, WorldVelocity);
            float DesiredVelChange = -SteeringVel * GripCoefficient;
            float DesiredAccChange = DesiredVelChange / Time.fixedDeltaTime;

            float SteerMultiplier = (IsGrounded == true) ? 1 : 0.3f;

            RB.AddTorque(transform.up * IM.SteerInput * SteerForce * Time.fixedDeltaTime * SteerMultiplier, ForceMode.Acceleration);
            RB.AddForce(SteerDir * DesiredAccChange * SteerMultiplier, SteerForceMode);
        }
    }
    void UpdateWheelsMeshes()
    {
        for(int i = 0; i < WheelMeshes.Length; i++)
        {
            WheelMeshes[i].position = SpherePositions[i];
        }
    }
    void UpdateGears()
    {
        if (IM.GearUp && CurrentGear < GearMaxSpeed.Length - 1)
            CurrentGear++;

        if (IM.GearDown && CurrentGear > 0)
            CurrentGear--;

        MaxSpeed = GearMaxSpeed[CurrentGear];
    }
    void UpdateAudio()
    {
        AudSource.pitch = -(RPM / MaxRPM) - 0.3f;
    }
    void UpdateCanvas()
    {
        float RPMRatio = (RPM / MaxRPM);
        float minrot = -100;
        float maxrot = 100;
        float rotamt = ((minrot - maxrot) * RPMRatio) + maxrot;

        Needle.rotation = Quaternion.Euler(0f, 0f, rotamt);
    }
    private void OnDrawGizmos()
    {
        for(int i = 0; i < SpherePositions.Length; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(SpherePositions[i], WheelRadius);
            
        }
    }
}
