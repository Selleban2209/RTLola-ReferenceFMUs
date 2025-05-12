# Reference FMUs with RTLola Monitoring Integration

This repository is a fork of the original [Modelica Reference FMUs](https://github.com/modelica/Reference-FMUs) with added RTLola runtime monitoring capabilities through FMI interfacing.

## Key Modifications

- **RTLola Integration**: All FMUs now include embedded RTLola monitoring through:
  - Direct FFI calls to RTLola interpreter
  - Depricated commented out code of the a pipe solution calling upon the RTLola-CLI program.
  - Type-safe data mapping between FMI and RTLola
  - Real-time trigger handling

- **Enhanced Interface**:
  - Added monitoring-specific variables to modelDescription.xml of models with the integration applied.(currently only BoucningBall)
  - New API calls for specification switching
  - Integrated FMI call and RTLola call logging 

## Usage Differences from Original

While maintaining all original functionality, these FMUs require:

1. Linking against RTLola shared libraries (`librtlola_ffi`)
2. Additional environment variables for monitoring configuration (e.g rtlola_spec, and rtlola_output).


> ℹ️ For standard FMI usage without monitoring, please see the [original README](README_ORIGINAL.md).


---

### Related Repositories  
This project is part of an ecosystem for RTLola-FMU integration. Key related components:  

1. **[FMU Test Environment](https://github.com/Selleban2209/FMU_test_env)**  
   - A dedicated framework for validating RTLola-enhanced FMUs  
   - Features automated test cases and performance benchmarking  
   - Supports co-simulation scenarios with trigger injection  

2. **[RTLola Parser](https://github.com/Selleban2209/rtloa_parser)**  
   - Lightweight parser for preprocessing RTLola specifications  
   - Handles variable mapping between RTLola and FMI conventions  
   - Validates spec compatibility before FMU integration  

3. **[RTLola Integration Core](https://github.com/Selleban2209/RTLola_integration)**  
   - Shared library with FFI bindings for Rust/C interoperability  
   - Implements the core monitoring state machine  
   - Used as a dependency by this repository  

---

## Building


### Building RTLola-Enabled FMUs

1. **Modify FMU for Monitoring**  
   - Add ValueReference ↔ VariableName mapping functions  
   - Extend `modelDescription.xml` with RTLola-specific variables
   - Modify getters/setters to support monitoring (see `BouncingBall` example)
   

2. **Compile Rust Components**  
   ```bash
   cd rtlola-ffi/
   cargo build --release
   ```
   Produces: `target/release/librtlola_ffi.{so,dylib,dll}`

3. **Configure CMake Integration**  
   Add to `CMakeLists.txt`:
   ```cmake
   # RTLola FFI linking
   find_library(RTLOLA_FFI rtlola_ffi PATHS "${PROJECT_SOURCE_DIR}/lib")
   target_link_libraries(${PROJECT_NAME} PRIVATE ${RTLOLA_FFI})
   ```


### Building RTLola-Enabled FMUs

4. **Build FMU**  
   The build process from this point follows the original Reference-FMUs procedure

   > **Note**: For detailed build options and platform-specific instructions, refer to [README_ORIGINAL.md](README_ORIGINAL.md#build-the-fmus)

5. **Validation**  
   Test using the [FMU Test Environment](https://github.com/Selleban2209/FMU_test_env):
   ```bash
   ./test_runner --monitoring=rtlola BouncingBall.fmu
   ```
  

**Compatibility**:
- Current solution only tested for Ubuntu 20.04 LTS
 
