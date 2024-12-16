use std::env;
use std::path::PathBuf;

fn main() {
    // Get the `OUT_DIR` where the generated Rust files will be placed
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    prost_build::Config::new()
        .out_dir(out_dir) // Specify the output directory for generated Rust code
        .compile_protos(
            // The .proto files to compile
            &["../proto/joint_trajectory_dof6.proto",
                "../proto/pose_array.proto"
            ],
            // The directories to search for imports
            &["../proto"],
        )
        .expect("Failed to compile Protobuf files");

    // Ensure the build script reruns if these files change
    println!("cargo:rerun-if-changed=../proto/joint_trajectory_dof6.proto");
    println!("cargo:rerun-if-changed=../proto/pose_array.proto");
    println!("cargo:rerun-if-changed=../proto");
}