# Communication Format

This document explains the communication protocol, including the format for messages exchanged between the client and the server.

---

## Message Structure

Each message sent to the server consists of the following components, serialized and sent in the specified order:

1. **Message Type (1 byte)**  
   A single byte that indicates the type of message being sent. The following message types are supported:
    - `0x01`: `PoseArray`
    - `0x02`: `JointTrajectoryDof6`

   This byte allows the server to correctly determine how to parse the incoming data.

2. **Message Length (4 bytes)**  
   The size of the serialized protobuf message, in bytes. This value is encoded in **big-endian (network byte order)**.  
   The server uses this length to read the exact amount of data for the protobuf message.

3. **Message Content (variable size)**  
   The serialized Protocol Buffers (protobuf) message. The content depends on the message type specified in the first byte.

---

## Protobuf Message Definitions

### 1. PoseArray
The `PoseArray` message contains a list of poses, each represented by a position and orientation.

```protobuf
syntax = "proto3";

message Pose {
    double x = 1;  // X-coordinate of the position
    double y = 2;  // Y-coordinate of the position
    double z = 3;  // Z-coordinate of the position
    double qx = 4; // Quaternion X-component (rotation)
    double qy = 5; // Quaternion Y-component (rotation)
    double qz = 6; // Quaternion Z-component (rotation)
    double qw = 7; // Quaternion W-component (rotation)
}

message PoseArray {
    string topic = 1;           // Topic name (e.g., "pose_array")
    repeated Pose poses = 2;    // List of Pose messages
}
```

### 2. JointTrajectoryDof6
The `JointTrajectoryDof6` message represents a trajectory for a 6-DOF robotic arm, with each step defining the joint positions.

```protobuf
syntax = "proto3";

message TrajectoryStep {
    float j1 = 1; // Joint 1 position
    float j2 = 2; // Joint 2 position
    float j3 = 3; // Joint 3 position
    float j4 = 4; // Joint 4 position
    float j5 = 5; // Joint 5 position
    float j6 = 6; // Joint 6 position
}

message JointTrajectoryDof6 {
    string topic = 1;                  // Topic name (e.g., "trajectory_topic")
    repeated TrajectoryStep steps = 2; // List of trajectory steps
}
```

---

## Communication Protocol

### Message Sending Sequence

1. **Construct the Message Content**  
   Serialize the appropriate protobuf message (`PoseArray` or `JointTrajectoryDof6`) into a binary format.

2. **Calculate the Message Length**  
   Determine the size of the serialized protobuf message in bytes.

3. **Send the Message**  
   Transmit the following in sequence:
    - The **message type** (1 byte)
    - The **message length** (4 bytes, big-endian)
    - The **serialized protobuf message** (variable size)

### Example Workflow

1. **Message Preparation Example**  
   For a `PoseArray` message:
    - Serialize the `PoseArray` using Protocol Buffers.
    - Determine the size of the serialized message in bytes (e.g., 256 bytes).
    - Assume the message type is `0x01`.

   Data to send:  
   `[0x01]` (1 byte) + `[0x00, 0x00, 0x01, 0x00]` (4 bytes, big-endian) + Protobuf binary data (256 bytes).

2. **Sending the Message**  
   Use the `sendMessage` function to encapsulate and transmit these components (refer to the provided code). The server will parse and handle the message accordingly.

---

## Notes

### Error Handling
- If the server receives invalid or corrupted data:
    - It will discard the message.
    - An error may be logged for debugging purposes.

### Extending the Protocol
Additional message types can be added by:
- Creating new `.proto` message definitions.
- Extending the communication protocol to include a unique message type identifier for the new message.
- Updating the server's parsing logic to handle the new message type.

---

Let me know if you need further modifications or additional details!