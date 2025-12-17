using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // For JointState
using RosMessageTypes.Tf2;     // For TFMessage (if using ROS-TCP-Connector)
// or using ROS2ForUnity.ROS2; and other namespaces if using ROS2 For Unity

namespace DigitalTwin
{
    public class RosSubscriber : MonoBehaviour
    {
        // ROS Connection
        private ROSConnection ros;

        // ROS 2 Topic Names
        public string jointStatesTopicName = "/joint_states";
        public string tfTopicName = "/tf";

        void Start()
        {
            // Get ROSConnection (assuming it's already set up in the scene)
            ros = ROSConnection.Get	Instance();

            // Register subscribers
            // You might need to change the message type depending on the ROS2 For Unity implementation
            // For ROS-TCP-Connector:
            ros.Subscribe<JointStateMsg>(jointStatesTopicName, JointStateCallback);
            ros.Subscribe<TFMessageMsg>(tfTopicName, TFCallback);

            // For ROS2 For Unity (native nodes):
            // ROS2.Subscribe<JointStateMsg>(jointStatesTopicName, JointStateCallback);
            // ROS2.Subscribe<TFMessageMsg>(tfTopicName, TFCallback);

            Debug.Log($"Subscribing to {jointStatesTopicName} and {tfTopicName}");
        }

        void JointStateCallback(JointStateMsg msg)
        {
            // Implement logic to process joint state messages
            // Example: Update joint angles of the robot model in Unity
            Debug.Log($"Received JointState: {msg.position.Length} joints");
        }

        void TFCallback(TFMessageMsg msg)
        {
            // Implement logic to process TF messages
            // Example: Update transform of robot links in Unity
            Debug.Log($"Received TFMessage with {msg.transforms.Length} transforms");
        }
    }
}
