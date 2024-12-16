# Communication Format

This document describes the communication protocol, including the format of messages exchanged between the client and server.

---

## Message Structure

Each message sent to the server follows the structure described below **in the exact order**:

1. **Magic Sequence (3 bytes)**  
   A unique sequence of bytes (`0xAA 0x55 0x01`) sent at the start of every message to allow the server to verify message integrity and identify valid communication.

2. **Message Type (1 byte)**  
   A single byte that identifies the type of message being sent.  
   Supported message types include:
    - `0x01`: `PoseArray`
    - `0x02`: `JointTrajectoryDof6`

   This type allows the server to correctly process the incoming payload.

3. **Message Length (4 bytes)**  
   A 4-byte integer representing the size (in bytes) of the serialized protobuf message (`Message Content`).  
   - **Encoding:** Big-endian (network byte order)  
   - This enables the server to read the exact amount of data for the `Message Content`.

4. **Message Content (variable size)**  
   The serialized Protocol Buffers (protobuf) message. This data depends on the `Message Type`.

---

## Protobuf Message Definitions

### 1. PoseArray
The `PoseArray` message contains a list of poses, each represented by position and orientation.

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
    repeated Pose poses = 2;    // List of Pose messages (can be empty)
}
```

### 2. JointTrajectoryDof6
The `JointTrajectoryDof6` message represents a robot arm trajectory. Each step consists of the target positions for all 6 joints.

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

To ensure correct operation, messages must be constructed and sent **in the following order**:

1. **Construct the Serialized Payload**  
   Prepare the serialized protobuf message (`PoseArray` or `JointTrajectoryDof6`) in binary format (e.g., using a protobuf library).

2. **Prepare the Magic Sequence**  
   Use the constant `MAGIC_NUMBER` (3 bytes: `0xAA 0x55 0x01`) for message verification.

3. **Send the Message**  
   Transmit the following in sequence:
    - **Magic Sequence (3 bytes):** `0xAA 0x55 0x01`
    - **Message Type (1 byte):** Identify the type of message (e.g., `0x01` for `PoseArray`)
    - **Message Length (4 bytes):** Big-endian representation of the serialized payload size
    - **Message Content (variable size):** Serialized protobuf binary content

---

## Example

### PoseArray Example

Suppose we want to transmit a `PoseArray` with topic `"poses_example"` containing a single pose `(x: 1.0, y: 2.0, z: 3.0, qx: 0.0, qy: 1.0, qz: 0.0, qw: 1.0)`.

1. **Magic Sequence:**  
   `0xAA 0x55 0x01`

2. **Message Type:**  
   `0x01` (indicating this is a `PoseArray` message)

3. **Serialized Protobuf Message:**  
   Using the protobuf library, the serialized content might look like this (in raw binary form):  
   `0x0A 0x0D 0x70 0x6F 0x73 0x65 0x73 0x5F 0x65 0x78 0x61 0x6D 0x70 0x6C 0x65 0x12 0x2A ...`  

   Let's assume the payload's total size is **50 bytes**.

4. **Message Length (4 bytes):**  
   The length of the serialized protobuf message (`50 bytes`) in big-endian:  
   `0x00 0x00 0x00 0x32`

5. **Final Message** (in transmitted order):  
   ```
   0xAA 0x55 0x01  // Magic Sequence (3 bytes)
   0x01            // Message Type (1 byte)
   0x00 0x00 0x00 0x32  // Message Length (4 bytes, big-endian)
   0x0A 0x0D 0x70 ...    // Message Content (variable size, 50 bytes in this example)
   ```

---

## Server Validation

On receiving data, the server will follow these steps:

1. **Verify the Magic Sequence:**  
   Ensure the first 3 bytes match the predefined magic sequence (`0xAA 0x55 0x01`). If not, discard the message.

2. **Read the Message Type (1 byte):**  
   Use the message type to prepare for parsing the payload accordingly.

3. **Read the Message Length (4 bytes):**  
   Parse the next 4 bytes as the length (big-endian). This determines how many bytes to read for the message payload.

4. **Read and Parse the Protobuf Message:**  
   Read the remaining bytes in the payload based on the parsed length and deserialize the protobuf message based on its type.


