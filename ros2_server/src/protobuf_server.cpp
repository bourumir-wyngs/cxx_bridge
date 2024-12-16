#include "cxx_bridge/MagicNumbers.h" // Include the magic number header, message types

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

#include <csignal>
#include <atomic> // For thread-safe control flag
#include <memory> // For std::shared_ptr

std::atomic<bool> running(true); // Controls the server loop
std::shared_ptr<Poco::Net::ServerSocket> globalServerSocket; // Shared global pointer to manage server socket

// Signal handler to shut down the server
void signalHandler(int signum) {
  std::cout << "[INFO] Received signal " << signum << ". Shutting down server..." << std::endl;
  running = false; // Set the shutdown flag
  if (globalServerSocket) {
    globalServerSocket->close(); // Close the socket to unblock if it's waiting
    std::cout << "[INFO] Listening socket closed." << std::endl;
  }
}

// Function to read the message size
uint32_t readMessageSize(Poco::Net::StreamSocket &socket) {
  uint32_t size;
  char buffer[4];

  std::cout << "[DEBUG] Attempting to read message size (4 bytes)." << std::endl;

  int bytesRead = socket.receiveBytes(buffer, sizeof(buffer));
  if (bytesRead != 4) {
    std::cerr << "[ERROR] Failed to read 4 bytes for message size. Bytes read: " << bytesRead << std::endl;
    throw std::runtime_error("Failed to read message size (incomplete read)");
  }

  std::memcpy(&size, buffer, sizeof(size));
  size = ntohl(size); // Convert from network byte order to host byte order
  return size;
}

// Function to read the full serialized message based on the size
std::string readFullMessage(Poco::Net::StreamSocket &socket, uint32_t messageSize) {
  std::string message;
  message.resize(messageSize); // Pre-allocate buffer with required size

  size_t totalBytesRead = 0;
  while (totalBytesRead < messageSize) {
    int bytesRead = socket.receiveBytes(&message[totalBytesRead], messageSize - totalBytesRead);
    if (bytesRead <= 0) {
      throw std::runtime_error("Socket closed or error occurred while reading");
    }
    totalBytesRead += bytesRead;
  }

  return message;
}

// Function to handle PoseArray messages
void handlePoseArray(const std::string &message, RosSender &rosSender) {
  pose_array::PoseArray poseArray;
  if (poseArray.ParseFromString(message)) {
    rosSender.sendPoseArray(poseArray);
  } else {
    std::cerr << "[ERROR] Failed to parse PoseArray message." << std::endl;
  }
}

// Function to handle JointTrajectoryDof6 messages
void handleJointTrajectoryDof6(const std::string &message, RosSender &rosSender) {
  joint_trajectory_dof6::JointTrajectoryDof6 jointTrajectory;
  if (jointTrajectory.ParseFromString(message)) {
    rosSender.sendJointTrajectory(jointTrajectory);
  } else {
    std::cerr << "[ERROR] Failed to parse JointTrajectoryDof6 message." << std::endl;
  }
}

// Server loop to process messages
void runServer(int serverPort, RosSender &rosSender) {
  try {
    // Create and store the server socket in the global pointer
    globalServerSocket = std::make_shared<Poco::Net::ServerSocket>(serverPort);
    std::cout << "[INFO] Server is listening on port " << serverPort << std::endl;

    while (running) {
      try {
        if (globalServerSocket->poll(Poco::Timespan(1, 0), Poco::Net::Socket::SELECT_READ)) {
          // Accept incoming connection
          Poco::Net::StreamSocket clientSocket = globalServerSocket->acceptConnection();
          std::cout << "[INFO] Client connected: " << clientSocket.peerAddress().toString() << std::endl;

          // Step 1: Read and verify the magic number
          std::array<uint8_t, MAGIC_NUMBER.size()> receivedMagicNumber;
          int bytesRead = clientSocket.receiveBytes(receivedMagicNumber.data(), receivedMagicNumber.size());

          // Use structured binding in C++17+ to simplify comparisons
          if (bytesRead != MAGIC_NUMBER.size() || receivedMagicNumber != MAGIC_NUMBER) {
            std::cerr << "[ERROR] Invalid or missing magic number. Disconnecting client." << std::endl;
            clientSocket.close();
            continue; // Move to the next client connection
          }

          // Step 2: Read message type (1 byte)
          char messageType;
          bytesRead = clientSocket.receiveBytes(&messageType, sizeof(messageType));
          if (bytesRead != 1) {
            throw std::runtime_error("Failed to read message type");
          }

          // Step 3: Read message size (4 bytes)
          uint32_t messageSize = readMessageSize(clientSocket); // Implemented elsewhere

          // Step 4: Read the full message content (message size determines length)
          std::string serializedMessage = readFullMessage(clientSocket, messageSize); // Implemented elsewhere

          // Step 5: Process the message based on its type
          if (messageType == POSE_ARRAY_MESSAGE) {
            handlePoseArray(serializedMessage, rosSender);
          } else if (messageType == JOINT_TRAJECTORY_MESSAGE) {
            handleJointTrajectoryDof6(serializedMessage, rosSender);
          } else {
            std::cerr << "[ERROR] Unknown message type received." << std::endl;
          }

          // Close the client connection
          clientSocket.close();
        }
      } catch (const Poco::TimeoutException &) {
        // Ignore timeout exceptions from poll()
      } catch (const std::exception &ex) {
        std::cerr << "[ERROR] Exception while processing client: " << ex.what() << std::endl;
        if (!running) {
          break; // Exit the loop if the server is shutting down
        }
      }
    }
  } catch (const std::exception &ex) {
    std::cerr << "[FATAL] Server encountered an unrecoverable error: " << ex.what() << std::endl;
  }

  std::cout << "[INFO] Server shutting down." << std::endl;
}

// Main entry point
int main(int argc, char **argv) {
  try {
    // Default host and port
    constexpr char DEFAULT_HOST[] = "127.0.0.1";
    constexpr int DEFAULT_PORT = 5555;

    // Determine the port
    int serverPort = DEFAULT_PORT;
    if (argc == 2) {
      serverPort = std::stoi(argv[1]);
      if (serverPort <= 0) {
        throw std::runtime_error("Invalid port number.");
      }
    } else if (argc > 2) {
      throw std::runtime_error("Usage: protobuf_server [port]");
    }

    // Signal handling
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto rosSender = std::make_shared<RosSender>();

    std::cout << "Starting server on host " << DEFAULT_HOST << " and port " << serverPort << "..." << std::endl;

    // Run the server
    runServer(serverPort, *rosSender);

    // Shutdown ROS 2
    rclcpp::shutdown();
  } catch (const std::exception &ex) {
    std::cerr << "[ERROR] " << ex.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}