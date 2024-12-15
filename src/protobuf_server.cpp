#include "cxx_bridge/message_types.h" // Include the message type constants
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Exception.h>
#include <iostream>
#include <vector>
#include <cstring>
#include "pose_array.pb.h" // Protobuf PoseArray
#include "joint_trajectory_dof6.pb.h" // Protobuf JointTrajectoryDof6
#include "cxx_bridge/protobuf_server.hpp" // Include the RosSender class
#include <rclcpp/rclcpp.hpp> // ROS 2 library

void dumpRawBytes(Poco::Net::StreamSocket& socket, size_t numBytes) {
  std::vector<char> buffer(numBytes);
  int bytesRead = socket.receiveBytes(buffer.data(), numBytes);

  std::cout << "[DEBUG] Dumping raw bytes received (" << bytesRead << " bytes): ";
  for (int i = 0; i < bytesRead; i++) {
    printf("%02x ", static_cast<unsigned char>(buffer[i]));
  }
  std::cout << std::endl;
}

uint32_t readMessageSize(Poco::Net::StreamSocket& socket) {
  uint32_t size;
  char buffer[4];

  std::cout << "[DEBUG] Attempting to read message size (4 bytes)." << std::endl;

  int bytesRead = socket.receiveBytes(buffer, sizeof(buffer));
  if (bytesRead != 4) {
    std::cerr << "[ERROR] Failed to read 4 bytes for message size. Bytes read: "
              << bytesRead << std::endl;
    throw std::runtime_error("Failed to read message size (incomplete read)");
  }

  // Log the raw size bytes
  std::cout << "[DEBUG] Raw message size bytes received: ";
  for (int i = 0; i < 4; i++) {
    printf("%02x ", static_cast<unsigned char>(buffer[i]));
  }
  std::cout << std::endl;

  // Convert from network byte order to host byte order
  std::memcpy(&size, buffer, sizeof(size));
  size = ntohl(size); // Network to host byte order

  std::cout << "[DEBUG] Converted message size (host byte order): " << size << " bytes." << std::endl;

  return size;
}

// Function to read the full serialized message based on the size
std::string readFullMessage(Poco::Net::StreamSocket &socket, uint32_t messageSize) {
  std::string message;
  message.resize(messageSize); // Pre-allocate buffer with required size

  size_t totalBytesRead = 0;

  // Loop to ensure the entire message is read
  while (totalBytesRead < messageSize) {
    int bytesRead = socket.receiveBytes(&message[totalBytesRead], messageSize - totalBytesRead);
    if (bytesRead <= 0) {
      throw std::runtime_error("Socket closed or error occurred while reading");
    }
    totalBytesRead += bytesRead;
  }

  return message;
}

// Function to handle PoseArray messages and publish via RosSender
void handlePoseArray(const std::string &message, RosSender &rosSender) {
  std::cout << "Received PoseArray message size: " << message.size() << std::endl;

  // Print message content as hex (for debugging)
  std::cout << "Received PoseArray message (hex): ";
  for (unsigned char c: message) {
    printf("%02x ", c);
  }
  std::cout << std::endl;

  pose_array::PoseArray poseArray;
  if (poseArray.ParseFromString(message)) {
    std::cout << "Received PoseArray message: Topic = " << poseArray.topic() << std::endl;

    // Publish the parsed PoseArray message to ROS via RosSender
    rosSender.sendPoseArray(poseArray);
  } else {
    std::cerr << "Failed to parse PoseArray message." << std::endl;
  }
}

// Function to handle JointTrajectoryDof6 messages and publish via RosSender
void handleJointTrajectoryDof6(const std::string &message, RosSender &rosSender) {
  joint_trajectory_dof6::JointTrajectoryDof6 jointTrajectory;

  // Parse the serialized message into a Protobuf object
  if (jointTrajectory.ParseFromString(message)) {
    std::cout << "Received JointTrajectoryDof6 message: Topic = " << jointTrajectory.topic() << std::endl;

    // Publish the parsed JointTrajectory message to ROS via RosSender
    rosSender.sendJointTrajectory(jointTrajectory);
  } else {
    std::cerr << "Failed to parse JointTrajectoryDof6 message." << std::endl;
  }
}

void runServer(int serverPort, RosSender &rosSender) {
  try {
    Poco::Net::ServerSocket serverSocket(serverPort);
    std::cout << "[INFO] Server is listening on port " << serverPort << std::endl;

    while (true) {
      // Wait for a client connection
      Poco::Net::StreamSocket clientSocket = serverSocket.acceptConnection();
      std::cout << "[INFO] Client connected: " << clientSocket.peerAddress().toString() << std::endl;

      try {
        // Step 1: Read the message type (1 byte)
        char messageType;
        int typeBytes = clientSocket.receiveBytes(&messageType, sizeof(messageType));
        if (typeBytes != 1) {
          std::cerr << "[ERROR] Failed to read message type. Bytes read: " << typeBytes << std::endl;
          throw std::runtime_error("Failed to read message type.");
        }
        std::cout << "[DEBUG] Message type received: " << static_cast<int>(messageType) << std::endl;
        std::cout << "[DEBUG] Dumping raw socket data..." << std::endl;

        // Step 2: Read the message size (4 bytes)
        uint32_t messageSize = readMessageSize(clientSocket);
        std::cout << "[DEBUG] Message size received: " << messageSize << " bytes" << std::endl;

        // Validate message size
        if (messageSize == 0) {
          std::cerr << "[ERROR] Received message size is 0. Skipping processing." << std::endl;
          throw std::runtime_error("Invalid message size (0)");
        }

        // Step 3: Read the full serialized Protobuf message
        std::string serializedMessage = readFullMessage(clientSocket, messageSize);

        // Debug log message content
        std::cout << "[DEBUG] Serialized message read successfully: " << serializedMessage.size()
                  << " bytes." << std::endl;
        std::cout << "[DEBUG] Serialized message (hex): ";
        for (unsigned char c: serializedMessage) {
          printf("%02x ", c);
        }
        std::cout << std::endl;

        // Step 4: Route the message to the appropriate handler
        if (messageType == POSE_ARRAY_MESSAGE) {
          std::cout << "[INFO] Handling PoseArray message." << std::endl;
          handlePoseArray(serializedMessage, rosSender);
        } else if (messageType == JOINT_TRAJECTORY_MESSAGE) {
          std::cout << "[INFO] Handling JointTrajectoryDof6 message." << std::endl;
          handleJointTrajectoryDof6(serializedMessage, rosSender);
        } else {
          std::cerr << "[ERROR] Unknown message type received: "
                    << static_cast<int>(messageType) << std::endl;
        }

      } catch (const std::exception &ex) {
        std::cerr << "[ERROR] Exception while handling client message: " << ex.what() << std::endl;
      }

      clientSocket.close();
      std::cout << "[INFO] Client connection closed." << std::endl;
    }
  } catch (const Poco::Exception &ex) {
    std::cerr << "[ERROR] Poco Exception: " << ex.displayText() << std::endl;
  } catch (const std::exception &ex) {
    std::cerr << "[ERROR] Standard Exception: " << ex.what() << std::endl;
  }
}

// Main entry point
int main(int argc, char **argv) {
  try {
    // Verify and get the port number from command-line arguments
    if (argc != 2) {
      throw std::invalid_argument("Usage: protobuf_server <port>");
    }

    int serverPort = std::atoi(argv[1]);
    if (serverPort <= 0) {
      throw std::invalid_argument("Invalid port number. Please enter a positive integer.");
    }

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the RosSender instance
    auto rosSender = std::make_shared<RosSender>();

    // Run the server, passing the RosSender instance
    runServer(serverPort, *rosSender);

    // Shutdown ROS 2 after server ends
    rclcpp::shutdown();
  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}