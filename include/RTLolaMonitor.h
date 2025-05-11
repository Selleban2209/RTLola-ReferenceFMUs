#ifndef RTLOLA_MONITOR_H
#define RTLOLA_MONITOR_H

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif
#include <stdio.h>
#include <stdbool.h>

// Forward declaration of ModelInstance
#ifndef MODEL_H_INCLUDED
typedef struct ModelInstance ModelInstance;
#else
#include "model.h"
#include "RTlolaNative.h"
#include "fmi2TypesPlatform.h"
#include "fmi3FunctionTypes.h"
#endif

#ifdef _WIN32
    #define POPEN _popen
    #define PCLOSE _pclose
#else
    #define POPEN popen
    #define PCLOSE pclose
#endif

// Define a structure to represent an input
typedef struct {
    const char* name;  // Input name (e.g., "h")
    const char* type_; // Input type (e.g., "Float64")
    unsigned int vr; // Value references for the output variable;
} Input;

// Define a structure to represent an output
typedef struct {
    const char* variable;   // Output variable (e.g., "height_above_ground")
    const char* comparison; // Output comparison (e.g., "h > 0.0")
} Output;

// Define a structure to represent a trigger
typedef struct {
    const char* condition; // The condition to evaluate (e.g., "h > 0.0")
    const char* message;   // The message to display or action to take if the condition is met
} Trigger;

// Define a structure to represent the feedback from RTLola
typedef struct {
    const char* variable; // Variable name (e.g., "h")
    double value;         // Current value of the variable
} Feedback;

// Define a structure to represent the RTLola specification
typedef struct {
    Input* inputs;          // Array of inputs
    int input_count;        // Number of inputs
    Output* outputs;        // Array of outputs
    int output_count;       // Number of outputs
    Trigger* triggers;      // Array of triggers
    int trigger_count;      // Number of triggers
} RTLolaSpec;


typedef struct {
    // Configuration
    const char* spec_path;          // Path to the RTLola specification
    const char * last_output;     // Last output from RTLola
    unsigned int* monitored_vrs;    // Array of monitored variable references
    size_t num_vars;                // Number of monitored variables
    const char** input_names;         // Array of input names for RTLola
    FILE* input_stream;             // Stream to write data to rtlola-cli
    FILE* output_stream;            // Stream to read data from rtlola-cli

    int input_pipe[2];  // Pipe for sending data to RTLola
    int output_pipe[2]; // Pipe for receiving output from RTLola    
    //pid_t child_pid;    // Process ID of the child process
    //pthread_t reader_thread;
    bool is_active;                 // Whether the monitor is active
    bool spec_switch_requested;     // Whether a spec switch has been requested
    bool spec_switch_pending;       // Whether a spec switch is pending
    bool execMode;            // Whether the monitor is in online mode

    // New fields for trigger evaluation
    RTLolaSpec* rtlola_spec;       // Parsed RTLola specification (from JSON)
    Feedback* feedback;            // Array of feedback values for monitored variables
    int feedback_count;            // Number of feedback values

    //Native implementation
    RTLolaMonitorHandle* rtlola_handle; // Handle for the RTLola monitor
} RTLolaMonitor;




/*
#############################
RTLola-cli functions
#############################
*/

// Initialization (called during FMU instantiation)
void RTLolaMonitor_Init(RTLolaMonitor* monitor, 
                        const char* spec_path,
                        const unsigned int* vrs,
                        size_t num_vars, bool execMode);

void RTLolaMonitor_Setup(RTLolaMonitor* monitor, 
                            const char* spec_path,
                            const unsigned int* vrs,
                            size_t num_vars);
// Start monitoring (called during enterInitializationMode)
bool RTLolaMonitor_Start(RTLolaMonitor* monitor);

// Send data (called during doStep)
bool RTLolaMonitor_SendData_v2(RTLolaMonitor* monitor, 
                               double time,
                               fmi2Component instanceRTLola);

bool RTLolaMonitor_SendData_FMI3(RTLolaMonitor* monitor, 
                                double time,
                                fmi3Instance instance);


// Cleanup (called during model termination)
void RTLolaMonitor_Cleanup(RTLolaMonitor* monitor);

// Adjust FMU variables based on triggers
bool adjustFMUVariables(RTLolaMonitor* monitor, const unsigned int* vrs, fmi2Component instance);

// Switch RTLola specification 
//bool RTLolaMonitor_HandleSpecSwitch(    fmi3Instance instance);

//  functions for trigger evaluation
void RTLolaMonitor_ParseSpec(RTLolaMonitor* monitor, const char* json_spec); // Parse JSON specification
void RTLolaMonitor_EvaluateTriggers(RTLolaMonitor* monitor, const char* output); // Evaluate triggers based on feedback
void RTLolaMonitor_TakeActions(RTLolaMonitor* monitor, const char* trigger_message); // Take actions based on triggered conditions


/*
#############################
Natve RTLola functions using Rust FFI 
#############################
*/
bool Native_RTLolaMonitor_Init(RTLolaMonitor* monitor, fmi3Instance instance);

bool Native_RTLolaMonitor_Start(RTLolaMonitor* monitor, fmi3Instance instance);

bool Native_RTLolaMonitor_SendData(RTLolaMonitor* monitor, fmi3Instance instance);
#endif // RTLOLA_MONITOR_H