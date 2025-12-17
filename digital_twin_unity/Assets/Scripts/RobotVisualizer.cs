using UnityEngine;
using Unity.Robotics.ROSTCPConnector; // or ROS2ForUnity.ROS2;
using RosMessageTypes.Sensor;
using RosMessageTypes.Tf2;
using System.Collections.Generic;

namespace DigitalTwin
{
    public class RobotVisualizer : MonoBehaviour
    {
        // Reference to the ROS Subscriber script or direct subscription if this script handles it
        // public RosSubscriber rosSubscriber; 

        // Dictionary to store references to robot joints in Unity
        public Dictionary<string, GameObject> robotJoints = new Dictionary<string, GameObject>();
        public GameObject robotRoot; // Assign the root of your robot model in Unity Editor

        void Start()
        {
            // Example: Populate robotJoints dictionary based on your Unity robot hierarchy
            // You would typically find these by name or tag in your scene.
            // For example:
            // robotJoints["left_shoulder_joint"] = GameObject.Find("LeftShoulder");
            // robotJoints["right_shoulder_joint"] = GameObject.Find("RightShoulder");

            // You would also need to integrate with the ROS Subscriber to get the messages.
            // For example, if RosSubscriber is on the same GameObject:
            // rosSubscriber = GetComponent<RosSubscriber>();
            // You might want to pass callbacks from RosSubscriber to this script.
        }

        // Call this method when a JointState message is received
        public void UpdateJointState(JointStateMsg jointState)
        {
            for (int i = 0; i < jointState.name.Length; i++)
            {
                string jointName = jointState.name[i];
                float jointPosition = (float)jointState.position[i];

                if (robotJoints.ContainsKey(jointName))
                {
                    // Apply joint position to the corresponding Unity joint
                    // This often involves rotating the GameObject representing the joint
                    // based on its local axis.
                    // Example (pseudo-code):
                    // Quaternion rotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0); // Assuming rotation around Y axis
                    // robotJoints[jointName].transform.localRotation = rotation;
                    Debug.Log($"Updating joint {jointName} to position {jointPosition}");
                }
            }
        }

        // Call this method when a TFMessage is received
        public void UpdateTFTransforms(TFMessageMsg tfMessage)
        {
            foreach (var transformMsg in tfMessage.transforms)
            {
                string childFrameId = transformMsg.child_frame_id;
                // Assuming child_frame_id maps directly to a GameObject name in Unity
                if (robotRoot != null && robotRoot.transform.Find(childFrameId) != null)
                {
                    Transform targetTransform = robotRoot.transform.Find(childFrameId);
                    
                    // Update position
                    targetTransform.localPosition = new Vector3(
                        (float)transformMsg.transform.translation.x,
                        (float)transformMsg.transform.translation.y,
                        (float)transformMsg.transform.translation.z
                    );

                    // Update rotation
                    targetTransform.localRotation = new Quaternion(
                        (float)transformMsg.transform.rotation.x,
                        (float)transformMsg.transform.rotation.y,
                        (float)transformMsg.transform.rotation.z,
                        (float)transformMsg.transform.rotation.w
                    );
                    Debug.Log($"Updating transform for {childFrameId}");
                }
            }
        }
    }
}
