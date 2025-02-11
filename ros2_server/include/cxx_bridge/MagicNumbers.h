#pragma once

#include <array>
#include <cstdint>

/// Define the Magic Number (3-byte sequence) that acts as a unique identifier
/// for verifying the validity of client messages.
/// Format: {0xAA, 0x55, VERSION}.
constexpr std::array<uint8_t, 3> MAGIC_NUMBER = {0xAA, 0x55, 0x05};

// Message type constants
constexpr char POSE_ARRAY_MESSAGE = 0x01;
constexpr char JOINT_TRAJECTORY_MESSAGE = 0x02;

// This can be mesh or point cloud (mesh with zero triangles is considered point cloud)
constexpr char POINT_CLOUD_MESSAGE = 0x03;