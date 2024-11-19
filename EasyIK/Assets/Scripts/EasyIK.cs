using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR 
using UnityEditor;

#endif 
using System.Linq;


[ExecuteAlways]
public class EasyIK : MonoBehaviour
{

    enum UpdateMode
    {
        Update,
        LateUpdate,
        FixedUpdate,
        ManualUpdate
    }

    [Header("Options")]
    [SerializeField] UpdateMode m_updateMode;
    [SerializeField] bool m_automaticallyFindJointTransforms;
    [SerializeField, Tooltip("WARNING: Enabling this will modify the transforms at edit time")] bool m_updateInEditor;

    [Header("IK properties")]

    public int numberOfJoints = 2;
    public Transform ikTarget;
    public int iterations = 10;
    public float tolerance = 0.05f;
    [SerializeField, Tooltip("WARNING: Disabling this will force rotation of the last bone to match the ik target if UpdateInEditor is enabled")] bool m_useRotationOffsetForLastJointFromIKTarget;
    [SerializeField] private Transform[] jointTransforms;
    private Vector3 startPosition;
    private Vector3[] jointPositions;
    private float[] boneLength;
    private float jointChainLength;
    private float distanceToTarget;
    private Quaternion[] startRotation;
    private Vector3[] jointStartDirection;
    private Quaternion ikTargetStartRot;
    private Quaternion lastJointStartRot;

    [Header("Pole target (3 joint chain)")]
    public Transform poleTarget;

    [Header("Debug")]
    public bool debugJoints = true;
    public bool localRotationAxis = false;

    // Remove this if you need bigger gizmos:
    [Range(0.0f, 1.0f)]
    public float gizmoSize = 0.05f;
    public bool poleDirection = false;
    public bool poleRotationAxis = false;
    bool m_ready;

    void OnValidate()
    {
        Initialize();
    }

    void Initialize()
    {
        m_ready = false;
        jointChainLength = 0;


        if (m_automaticallyFindJointTransforms)
        {
            jointTransforms = new Transform[numberOfJoints];
        }

        jointPositions = new Vector3[numberOfJoints];
        boneLength = new float[numberOfJoints];
        jointStartDirection = new Vector3[numberOfJoints];
        startRotation = new Quaternion[numberOfJoints];
        ikTargetStartRot = ikTarget.rotation;




        var current = transform;
        Transform next;
        int transformsFound = 1; // the transform this script is attached to


        // For each bone calculate and store the lenght of the bone
        for (var i = 0; i < jointTransforms.Length; i += 1)
        {
            if (!m_automaticallyFindJointTransforms)
            {
                current = jointTransforms[i];

            }
            else
            {
                jointTransforms[i] = current;



            }


            // If the bones lenght equals the max lenght, we are on the last joint in the chain
            if (i == jointTransforms.Length - 1)
            {
                lastJointStartRot = current.rotation;
                break;
            }
            // Store length and add the sum of the bone lengths
            else
            {

                if (m_automaticallyFindJointTransforms)
                {
                    // int c = current.childCount;
                    // if (c == 0) break;
                    next = current.GetChild(0);
                }
                else
                {
                    next = jointTransforms[i + 1];
                }

                // if (next == null) break;

                boneLength[i] = Vector3.Distance(current.position, next.position);
                jointChainLength += boneLength[i];

                jointStartDirection[i] = next.position - current.position;
                startRotation[i] = current.rotation;
            }

            if (m_automaticallyFindJointTransforms)
            {
                // Move the iteration to next joint in the chain
                current = current.GetChild(0);
            }


            transformsFound++;
        }

        m_ready = transformsFound == numberOfJoints;
        if (!m_ready)
        {
            Debug.LogWarning("EasyIK Warning: The number of transforms in chain do not match the set number of joints, click to focus on gameobject", gameObject);
        }



    }
    void Awake()
    {
        Initialize();
    }




    void PoleConstraint()
    {
        if (poleTarget != null && numberOfJoints < 4)
        {
            // Get the limb axis direction
            var limbAxis = (jointPositions[2] - jointPositions[0]).normalized;

            // Get the direction from the root joint to the pole target and mid joint
            Vector3 poleDirection = (poleTarget.position - jointPositions[0]).normalized;
            Vector3 boneDirection = (jointPositions[1] - jointPositions[0]).normalized;

            // Ortho-normalize the vectors
            Vector3.OrthoNormalize(ref limbAxis, ref poleDirection);
            Vector3.OrthoNormalize(ref limbAxis, ref boneDirection);

            // Calculate the angle between the boneDirection vector and poleDirection vector
            Quaternion angle = Quaternion.FromToRotation(boneDirection, poleDirection);

            // Rotate the middle bone using the angle
            jointPositions[1] = angle * (jointPositions[1] - jointPositions[0]) + jointPositions[0];
        }
    }

    void Backward()
    {
        // Iterate through every position in the list until we reach the start of the chain
        for (int i = jointPositions.Length - 1; i >= 0; i -= 1)
        {
            // The last bone position should have the same position as the ikTarget
            if (i == jointPositions.Length - 1)
            {
                jointPositions[i] = ikTarget.transform.position;
            }
            else
            {
                jointPositions[i] = jointPositions[i + 1] + (jointPositions[i] - jointPositions[i + 1]).normalized * boneLength[i];
            }
        }
    }

    void Forward()
    {
        // Iterate through every position in the list until we reach the end of the chain
        for (int i = 0; i < jointPositions.Length; i += 1)
        {
            // The first bone position should have the same position as the startPosition
            if (i == 0)
            {
                jointPositions[i] = startPosition;
            }
            else
            {
                jointPositions[i] = jointPositions[i - 1] + (jointPositions[i] - jointPositions[i - 1]).normalized * boneLength[i - 1];
            }
        }
    }

    private void SolveIK()
    {
        // Get the jointPositions from the joints
        for (int i = 0; i < jointTransforms.Length; i += 1)
        {
            jointPositions[i] = jointTransforms[i].position;
        }
        // Distance from the root to the ikTarget
        distanceToTarget = Vector3.Distance(jointPositions[0], ikTarget.position);

        // IF THE TARGET IS NOT REACHABLE
        if (distanceToTarget > jointChainLength)
        {
            // Direction from root to ikTarget
            var direction = ikTarget.position - jointPositions[0];

            // Get the jointPositions
            for (int i = 1; i < jointPositions.Length; i += 1)
            {
                jointPositions[i] = jointPositions[i - 1] + direction.normalized * boneLength[i - 1];
            }
        }
        // IF THE TARGET IS REACHABLE
        else
        {
            // Get the distance from the leaf bone to the ikTarget
            float distToTarget = Vector3.Distance(jointPositions[jointPositions.Length - 1], ikTarget.position);
            float counter = 0;
            // While the distance to target is greater than the tolerance let's iterate until we are close enough
            while (distToTarget > tolerance)
            {
                startPosition = jointPositions[0];
                Backward();
                Forward();
                counter += 1;
                // After x iterations break the loop to avoid an infinite loop
                if (counter > iterations)
                {
                    break;
                }
            }
        }
        // Apply the pole constraint
        PoleConstraint();

        // Apply the jointPositions and rotations to the joints
        for (int i = 0; i < jointPositions.Length - 1; i += 1)
        {
            jointTransforms[i].position = jointPositions[i];
            var targetRotation = Quaternion.FromToRotation(jointStartDirection[i], jointPositions[i + 1] - jointPositions[i]);
            jointTransforms[i].rotation = targetRotation * startRotation[i];


        }


        if (m_useRotationOffsetForLastJointFromIKTarget)
        {
            // Let's constrain the rotation of the last joint to the IK target and maintain the offset.
            Quaternion offset = Quaternion.Inverse(ikTargetStartRot) * lastJointStartRot;
            jointTransforms.Last().rotation = ikTarget.rotation * offset;
        }
        else
        {

            jointTransforms.Last().rotation = ikTarget.rotation;
        }


    }


    void Update()
    {
        if (m_updateMode != UpdateMode.Update) return;
        OnUpdate();
    }
    void LateUpdate()
    {

        if (Application.isEditor && !Application.isPlaying && m_updateInEditor)
        {
            SolveIK();
        }

        if (m_updateMode != UpdateMode.LateUpdate) return;
        OnUpdate();

    }
    void FixedUpdate()
    {
        if (m_updateMode != UpdateMode.FixedUpdate) return;
        OnUpdate();
    }
    public void OnUpdate()
    {
        if (!Application.isPlaying) return;
        SolveIK();
    }
#if UNITY_EDITOR 

    // Visual debugging
    void OnDrawGizmos()
    {
        if (!m_ready) return;
        if (debugJoints == true)
        {
            var current = jointTransforms[0];
            var child = jointTransforms[1];

            for (int i = 0; i < numberOfJoints; i += 1)
            {
                if (i == numberOfJoints - 2)
                {
                    var length = Vector3.Distance(current.position, child.position);
                    DrawWireCapsule(current.position + (child.position - current.position).normalized * length / 2, Quaternion.FromToRotation(Vector3.up, (child.position - current.position).normalized), gizmoSize, length, Color.cyan);
                    break;
                }
                else
                {
                    var length = Vector3.Distance(current.position, child.position);
                    DrawWireCapsule(current.position + (child.position - current.position).normalized * length / 2, Quaternion.FromToRotation(Vector3.up, (child.position - current.position).normalized), gizmoSize, length, Color.cyan);
                    current = jointTransforms[i+1];
                    child = jointTransforms[i+2];
                }
            }
        }

        if (localRotationAxis == true)
        {
            var current = transform;
            for (int i = 0; i < numberOfJoints; i += 1)
            {
                if (i == numberOfJoints - 1)
                {
                    drawHandle(current);
                }
                else
                {
                    drawHandle(current);
                    current = jointTransforms[i+1];
                }
            }
        }

        var start = jointTransforms[0];
        var mid = jointTransforms[1];
        var end = jointTransforms[2];

        if (poleRotationAxis == true && poleTarget != null && numberOfJoints < 4)
        {
            Handles.color = Color.white;
            Handles.DrawLine(start.position, end.position);
        }

        if (poleDirection == true && poleTarget != null && numberOfJoints < 4)
        {
            Handles.color = Color.grey;
            Handles.DrawLine(start.position, poleTarget.position);
            Handles.DrawLine(end.position, poleTarget.position);
        }

    }

    void drawHandle(Transform debugJoint)
    {
        Handles.color = Handles.xAxisColor;
        Handles.ArrowHandleCap(0, debugJoint.position, debugJoint.rotation * Quaternion.LookRotation(Vector3.right), gizmoSize, EventType.Repaint);

        Handles.color = Handles.yAxisColor;
        Handles.ArrowHandleCap(0, debugJoint.position, debugJoint.rotation * Quaternion.LookRotation(Vector3.up), gizmoSize, EventType.Repaint);

        Handles.color = Handles.zAxisColor;
        Handles.ArrowHandleCap(0, debugJoint.position, debugJoint.rotation * Quaternion.LookRotation(Vector3.forward), gizmoSize, EventType.Repaint);
    }

    public static void DrawWireCapsule(Vector3 _pos, Quaternion _rot, float _radius, float _height, Color _color = default(Color))
    {
        Handles.color = _color;
        Matrix4x4 angleMatrix = Matrix4x4.TRS(_pos, _rot, Handles.matrix.lossyScale);
        using (new Handles.DrawingScope(angleMatrix))
        {
            var pointOffset = (_height - (_radius * 2)) / 2;

            Handles.DrawWireArc(Vector3.up * pointOffset, Vector3.left, Vector3.back, -180, _radius);
            Handles.DrawLine(new Vector3(0, pointOffset, -_radius), new Vector3(0, -pointOffset, -_radius));
            Handles.DrawLine(new Vector3(0, pointOffset, _radius), new Vector3(0, -pointOffset, _radius));
            Handles.DrawWireArc(Vector3.down * pointOffset, Vector3.left, Vector3.back, 180, _radius);

            Handles.DrawWireArc(Vector3.up * pointOffset, Vector3.back, Vector3.left, 180, _radius);
            Handles.DrawLine(new Vector3(-_radius, pointOffset, 0), new Vector3(-_radius, -pointOffset, 0));
            Handles.DrawLine(new Vector3(_radius, pointOffset, 0), new Vector3(_radius, -pointOffset, 0));
            Handles.DrawWireArc(Vector3.down * pointOffset, Vector3.back, Vector3.left, -180, _radius);

            Handles.DrawWireDisc(Vector3.up * pointOffset, Vector3.up, _radius);
            Handles.DrawWireDisc(Vector3.down * pointOffset, Vector3.up, _radius);
        }
    }

#endif 
}
