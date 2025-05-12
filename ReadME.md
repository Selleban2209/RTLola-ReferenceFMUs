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
2. Additional environment variables for monitoring configuration
3. New CSV output columns for monitoring triggers

> ℹ️ For standard FMI usage without monitoring, please see the [original README](README_ORIGINAL.md).

## Building

Follow the original build instructions, with these additions:

```bash
# Required new dependency
git clone https://github.com/rtlola/rtlola-ffi.git
export RTLOLA_FFI_PATH=/path/to/rtlola-ffi
CMake options now include:

WITH_RTLOLA (ON by default)

RTLOLA_SPEC_FILE (default: monitoring.lola)

Examples
bash
# Simulate with monitoring
fmusim --output-variable=rtlola_triggers BouncingBall.fmu
Repository Structure
Added/modified files:

/include/rtlola_interface.h    # Monitoring API
/src/rtlola_bridge.c           # FFI implementation
/specs/                        # Example RTLola specs
License
Like the original, this fork is released under the 2-Clause BSD License, with additional copyright notices for RTLola components.
