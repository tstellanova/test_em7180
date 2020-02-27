use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {

    #[cfg(feature = "stm32h7x")]
    let memfile_bytes = include_bytes!("stm32h743_memory.x");

    #[cfg(feature = "stm32f4x")]
    let memfile_bytes = include_bytes!("stm32f401_memory.x");

    #[cfg(feature = "stm32f3x")]
    let memfile_bytes = include_bytes!("stm32f334_memory.x");

        // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(memfile_bytes) //include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // // Only re-run the build script when memory.x is changed,
    // // instead of when any part of the source code changes.
    // println!("cargo:rerun-if-changed=memory.x");
}
