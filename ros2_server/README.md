# Protobuf Server

This project is a C++ server application that listens for incoming protobuf messages and processes them using **Poco C++ Libraries** and **ROS 2**.

## Features

- Handles Protocol Buffers messages like `PoseArray` and `JointTrajectoryDof6`.
- Publishes received data using ROS 2.
- Implements graceful shutdown with signal handling (e.g., `Ctrl+C` for SIGINT).
- Lightweight and extensible for additional protobuf message types.

---

## Dependencies

This project relies on the following dependencies:

1. **ROS 2 Humble**  
   Follow the installation instructions for [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html).

   **Note:** Make sure you source the ROS 2 environment in your terminal:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Poco C++ Libraries**  
   Install the Poco libraries for networking and utility support:
   ```bash
   sudo apt-get install libpoco-dev
   ```

3. **Protocol Buffers**  
   Install the protobuf compiler and its C++ development libraries:
   ```bash
   sudo apt-get install protobuf-compiler libprotobuf-dev
   ```

4. **CMake**  
   Install CMake for build configuration:
   ```bash
   sudo apt-get install cmake
   ```

5. **GCC or Clang with C++17+ Support**  
   Ensure your compiler supports at least C++17. Install GCC:
   ```bash
   sudo apt-get install build-essential
   ```

---

## Building the Project

### Clone the Repository
First, clone this repository:
```bash
git clone <repository_url> cxx_bridge
cd cxx_bridge
```

### Build Steps

1. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```

2. Configure the project using `cmake`:
   ```bash
   cmake ..
   ```

3. Build the project:
   ```bash
   cmake --build . --target protobuf_server
   ```

After the build completes, the `protobuf_server` binary will be available in the `build` folder.

---

## Running the Server

To execute the server, run the following command:
```bash
./protobuf_server <port>
```

### Example:
Run the server on port `8080`:
```bash
./protobuf_server 8080
```

---

## Generating Protobuf Files (if needed)

If you modify or add `.proto` files, use the following commands to generate the C++ source files:

1. Generate protobuf definitions:
   ```bash
   protoc --cpp_out=. <file_name>.proto
   ```

2. Add generated `.pb.h` and `.pb.cc` files to the project CMake configuration for compilation.

---

## Troubleshooting

### 1. **ROS 2 Environment Not Found**
Ensure your ROS 2 environment is properly sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

### 2. **Missing Dependencies**
Check the dependency list above and ensure everything is installed correctly.

### 3. **Port Already in Use**
If the port specified for the server is already in use, try starting the server with a different port.

---

## Notes

- This server only supports the protobuf messages `PoseArray` and `JointTrajectoryDof6` for now. If you wish to extend it for more message types, update the protobuf definitions and add appropriate handlers in the code.
- This project assumes compatibility with Linux environments. Steps may vary for macOS or Windows systems.

---

Let me know if you'd like further enhancements or examples added to the file!