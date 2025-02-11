#include "cxx_bridge/MagicNumbers.h" // Include the magic number header
#include "pose_array.pb.h"
#include "joint_trajectory_dof6.pb.h"

#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Exception.h>

#include <nlohmann/json.hpp> // Include nlohmann/json for JSON parsing
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib> // For std::atoi

using json = nlohmann::json;

static const double SCALING_FACTOR = 0.005;

// Function to create a PoseArray message from a JSON file
std::string createPoseArrayMessageFromJson(const std::string& jsonFilePath) {
  pose_array::PoseArray poseArray;
  poseArray.set_topic("/pose_array");
  poseArray.set_frame("world");

  // Open the JSON file
  std::ifstream jsonFile(jsonFilePath);
  if (!jsonFile.is_open()) {
    throw std::runtime_error("Unable to open JSON file: " + jsonFilePath);
  }

  // Parse the JSON file
  json jsonData;
  jsonFile >> jsonData;

  // Iterate through the JSON array and populate the PoseArray Protobuf message
  for (const auto& poseData : jsonData) {
    auto* pose = poseArray.add_poses();
    pose->set_x(poseData["position"]["x"].get<double>() * SCALING_FACTOR);
    pose->set_y(poseData["position"]["y"].get<double>() * SCALING_FACTOR);
    pose->set_z(poseData["position"]["z"].get<double>() * SCALING_FACTOR);
    pose->set_qx(poseData["rotation"]["x"].get<double>());
    pose->set_qy(poseData["rotation"]["y"].get<double>());
    pose->set_qz(poseData["rotation"]["z"].get<double>());
    pose->set_qw(poseData["rotation"]["w"].get<double>());
  }

  // Serialize the Protobuf message to a string
  std::string serializedMessage;
  if (!poseArray.SerializeToString(&serializedMessage)) {
    throw std::runtime_error("Failed to serialize PoseArray Protobuf message.");
  }

  // Debug: Print serialized message size and content
  std::cout << "Serialized PoseArray message size: " << serializedMessage.size() << std::endl;
  std::cout << std::endl;


  return serializedMessage;
}

std::string createJointTrajectoryMessageFullRotation() {
  // Prepare the JointTrajectoryDof6 message
  joint_trajectory_dof6::JointTrajectoryDof6 trajectory_msg;
  trajectory_msg.set_topic("trajectory_topic");

  // Add 360 steps to represent a full rotation of j1 in radians
  const float step_increment = 2 * M_PI / 360.0f; // Each step in radians (equal division of 2π)

  for (int step = 0; step < 360; ++step) {
    auto* trajectory_step = trajectory_msg.add_steps();

    trajectory_step->set_j1(step * step_increment); // Set j1 to the current angle in radians
    trajectory_step->set_j2(M_PI_2);                 // Keep j2 at 90
    trajectory_step->set_j3(0.0f);                 // Keep j3 at 0
    trajectory_step->set_j4(0.0f);                 // Keep j4 at 0
    trajectory_step->set_j5(0.0f);                 // Keep j5 at 0
    trajectory_step->set_j6(0.0f);                 // Keep j6 at 0
  }

  // Serialize the Protobuf message to a string
  std::string serializedMessage;
  if (!trajectory_msg.SerializeToString(&serializedMessage)) {
    throw std::runtime_error("Failed to serialize JointTrajectoryDof6 Protobuf message.");
  }

  // Debug: Print serialized message size
  std::cout << "Serialized JointTrajectoryDof6 message size: " << serializedMessage.size() << std::endl;

  return serializedMessage;
}

// Function to send a message to the server
/**
 * Sends a message to the server.
 *
 * The message is sent in the following structure:
 * 1. Magic Number (3 bytes): Used for validating the message format.
 * 2. Message Type (1 byte): Identifies the type of message being sent (e.g., pose array, trajectory, etc.).
 * 3. Message Size (4 bytes, big-endian): The size of the serialized message payload.
 * 4. Message Payload: The actual serialized content of the message.
 *
 * @param serverAddress The IP address or hostname of the server.
 * @param serverPort The port on which the server is listening.
 * @param messageType A 1-byte identifier indicating the type of message to send.
 * @param message The serialized payload of the message.
 */
void sendMessage(const std::string& serverAddress, int serverPort, char messageType, const std::string& message) {
  try {
    // Step 1: Establish a connection to the server
    Poco::Net::SocketAddress socketAddress(serverAddress, serverPort);
    Poco::Net::StreamSocket socket(socketAddress);

    // Step 2: Send the predefined Magic Number (3 bytes)
    socket.sendBytes(MAGIC_NUMBER.data(), MAGIC_NUMBER.size());

    // Step 3: Send the Message Type (1 byte to specify the message format)
    socket.sendBytes(&messageType, sizeof(messageType));

    // Step 4: Calculate the size of the serialized message payload
    uint32_t messageSize = message.size();

    // Convert the message size to network byte order (big-endian)
    uint32_t networkOrderSize = htonl(messageSize);

    // Step 5: Send the Message Size (4 bytes, big-endian format)
    socket.sendBytes(&networkOrderSize, sizeof(networkOrderSize));

    // Step 6: Send the actual Serialized Message Payload
    socket.sendBytes(message.data(), message.size());

    // Log a success message after the data transmission
    std::cout << "Message sent successfully to " << serverAddress << ":" << serverPort << std::endl;

    // Close the socket after sending the message
    socket.close();
  } catch (const Poco::Exception& ex) {
    // Handle Poco-specific exceptions (e.g., network errors)
    std::cerr << "Poco Exception: " << ex.displayText() << std::endl;
  } catch (const std::exception& ex) {
    // Handle standard C++ exceptions
    std::cerr << "Standard Exception: " << ex.what() << std::endl;
  }
}

int main(int argc, char** argv) {
  try {
    // Verify and get server address, port, and JSON file path from command-line arguments
    if (argc != 4) {
      throw std::invalid_argument("Usage: client <server_address> <server_port> <json_file_path>");
    }

    std::string serverAddress = argv[1];
    int serverPort = std::atoi(argv[2]);
    std::string jsonFilePath = argv[3];

    if (serverPort <= 0) {
      throw std::invalid_argument("Invalid port number. Please enter a positive integer.");
    }

    // Create and send PoseArray message from JSON file
    std::cout << "[INFO] Creating and sending PoseArray message." << std::endl;
    std::string poseArrayMessage = createPoseArrayMessageFromJson(jsonFilePath);
    sendMessage(serverAddress, serverPort, POSE_ARRAY_MESSAGE, poseArrayMessage);

    // Create and send JointTrajectoryDof6 message with full rotation
    std::cout << "[INFO] Creating and sending JointTrajectoryDof6 message." << std::endl;
    std::string jointTrajectoryMessage = createJointTrajectoryMessageFullRotation();
    sendMessage(serverAddress, serverPort, JOINT_TRAJECTORY_MESSAGE, jointTrajectoryMessage);

    std::cout << "[INFO] Messages sent successfully." << std::endl;

  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}