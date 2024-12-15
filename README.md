
# cxx_bridge Project

## Build Setup Instructions
### 1. Required Dependencies
- A C++ toolchain (e.g., GCC or Clang).
- CMake (minimum required version 3.12).
- Rust and Cargo (stable version).
- gRPC and Protocol Buffers (ensure both are installed).
- Python (optional, if additional scripts are required).

#### Installing gRPC and Protocol Buffers
To install gRPC and Protocol Buffers, you can follow the official instructions:

```bash
# Clone the gRPC repository
git clone --recurse-submodules -b v1.57.0 https://github.com/grpc/grpc
cd grpc

# Install dependencies
mkdir -p cmake/build
cd cmake/build

# Build and install Protocol Buffers
cmake ../..
make -j
make install

# Build and install gRPC
cd ../..
cmake -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF .
make -j
make install
```

This will ensure that gRPC and Protocol Buffers are available for your `cxx_bridge` project.
- Python (optional, if additional scripts are required).
### 2. Steps to Configure and Run the Build Using CMake
1. Clone the `cxx_bridge` repository:
    ```bash
    git clone https://github.com/your-organization/cxx_bridge.git
    cd cxx_bridge
    ```
2. Install gRPC and Protocol Buffers if not already installed (see "Required Dependencies").

3. Create a build directory and navigate into it:
    ```bash
    mkdir build
    cd build
    ```
4. Configure the project using CMake. Ensure to define paths for gRPC and Protocol Buffers:
    ```bash
    cmake -DProtobuf_PROTOC_EXECUTABLE=/usr/local/bin/protoc \
          -DGRPC_CPP_PLUGIN=/usr/local/bin/grpc_cpp_plugin \
          ..
    ```
5. Build the project:
    ```bash
    cmake --build .
    ```
cd ../..
### 3. Example Commands for Building and Running the Project
To build the project:
```bash
cmake -S . -B build -DProtobuf_PROTOC_EXECUTABLE=/usr/local/bin/protoc \
                        -DGRPC_CPP_PLUGIN=/usr/local/bin/grpc_cpp_plugin
cmake --build build
```

To run the project:
```bash
./build/cxx_bridge
```

### 4. Using gRPC in the `cxx_bridge` Project
Make sure to define `.proto` files for your gRPC services. Use the following command to generate the gRPC code:
```bash
protoc --grpc_out=. --plugin=protoc-gen-grpc=/usr/local/bin/grpc_cpp_plugin service_name.proto
protoc --cpp_out=. service_name.proto
```
Add the generated code into your `cxx_bridge` project, and ensure `CMakeLists.txt` includes the necessary gRPC libraries:
```cmake
find_package(gRPC CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
add_executable(cxx_bridge main.cpp generated_code.cpp)
target_link_libraries(cxx_bridge PRIVATE gRPC::grpc++ protobuf::libprotobuf)
```
    cd cxx_bridge
    ```
2. Install gRPC and Protocol Buffers if not already installed (see "Required Dependencies").

3. Create a build directory and navigate into it:
    ```bash
    mkdir build
    cd build
    ```
4. Configure the project using CMake. Ensure to define paths for gRPC and Protocol Buffers:
    ```bash
    cmake -DProtobuf_PROTOC_EXECUTABLE=/usr/local/bin/protoc \
          -DGRPC_CPP_PLUGIN=/usr/local/bin/grpc_cpp_plugin \
          ..
    ```
5. Build the project:
    ```bash
    cmake --build .
    ```
    ```
### 3. Example Commands for Building and Running the Project
To build the project:
```bash
cmake -S . -B build -DProtobuf_PROTOC_EXECUTABLE=/usr/local/bin/protoc \
                        -DGRPC_CPP_PLUGIN=/usr/local/bin/grpc_cpp_plugin
cmake --build build
```

To run the project (if applicable):
```bash
./build/<executable-name>
```

### 4. Using gRPC in the Project
Make sure to define `.proto` files for your gRPC services. Use the following command to generate the gRPC code:
```bash
protoc --grpc_out=. --plugin=protoc-gen-grpc=/usr/local/bin/grpc_cpp_plugin <your-service>.proto
protoc --cpp_out=. <your-service>.proto
```
Add the generated code into your project, and ensure CMakeLists.txt includes the necessary gRPC libraries:
```cmake
find_package(gRPC CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
add_executable(<your-target> <generated-files>)
target_link_libraries(<your-target> PRIVATE gRPC::grpc++ protobuf::libprotobuf)
```
```

Replace `<repository-url>` and `<executable-name>` with the actual repository URL and executable name respectively.
