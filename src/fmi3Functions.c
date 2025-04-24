#ifndef FMI_VERSION
#define FMI_VERSION 3
#endif

#if FMI_VERSION != 3
#error FMI_VERSION must be 3
#endif

#ifdef _WIN32
    #include <windows.h>
    #define POPEN _popen
    #define PCLOSE _pclose
#else
    #include <pthread.h>
    #define POPEN popen
    #define PCLOSE pclose
#endif
#include <sys/types.h>


#include <stdlib.h> 
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include "config.h"
#include "model.h"
#include "cJSON.c"
#include "cosimulation.h"
#include "RTLolaMonitor.h"
#include "RTLolaNative.h"
#include "RTLolaParser.h"



// C-code FMUs have functions names prefixed with MODEL_IDENTIFIER_.
// Define DISABLE_PREFIX to build a binary FMU.
#if !defined(DISABLE_PREFIX) && !defined(FMI3_FUNCTION_PREFIX)
#define pasteA(a,b)          a ## b
#define pasteB(a,b)          pasteA(a,b)
#define FMI3_FUNCTION_PREFIX pasteB(MODEL_IDENTIFIER, _)
#endif
#include "fmi3Functions.h"

#define ASSERT_NOT_NULL(p) \
do { \
    if (!p) { \
        logError(S, "Argument %s must not be NULL.", xstr(p)); \
        S->state = Terminated; \
        return (fmi3Status)Error; \
    } \
} while (0)

#define CALL(f) do { \
    const Status s = f; \
    if (s > status) { \
        status = s; \
    } \
    if (status == Discard) { \
        goto TERMINATE; \
    } else if (status == Error) { \
        S->state = Terminated; \
        goto TERMINATE; \
    } else if (status == Fatal) { \
        S->state = StartAndEnd; \
        goto TERMINATE; \
    } \
} while (false)

#define GET_VARIABLES(T) \
BEGIN_FUNCTION(Get ## T); \
if (nValueReferences == 0) goto TERMINATE; \
else ASSERT_NOT_NULL(valueReferences); \
if (nValues > 0) ASSERT_NOT_NULL(values); \
if (S->isDirtyValues) { \
    CALL(calculateValues(S)); \
    S->isDirtyValues = false; \
} \
size_t index = 0; \
for (size_t i = 0; i < nValueReferences; i++) { \
    CALL(get ## T(S, (ValueReference)valueReferences[i], values, nValues, &index)); \
} \
if (index != nValues) { \
    logError(S, "Expected nValues = %zu but was %zu.", index, nValues); \
    CALL(Error); \
} \
END_FUNCTION()

#define SET_VARIABLES(T) \
BEGIN_FUNCTION(Set ## T); \
if (nValueReferences == 0) goto TERMINATE; \
ASSERT_NOT_NULL(valueReferences); \
size_t index = 0; \
for (size_t i = 0; i < nValueReferences; i++) { \
    CALL(set ## T(S, (ValueReference)valueReferences[i], values, nValues, &index)); \
} \
if (nValueReferences > 0) S->isDirtyValues = true; \
if (index != nValues) { \
    logError(S, "Expected nValues = %zu but was %zu.", index, nValues); \
    CALL(Error); \
} \
END_FUNCTION()

#ifndef max
#define max(a,b) ((a)>(b) ? (a) : (b))
#endif

#ifndef DT_EVENT_DETECT
#define DT_EVENT_DETECT 1e-10
#endif

// ---------------------------------------------------------------------------
// Function calls allowed state masks for both Model-exchange and Co-simulation
// ---------------------------------------------------------------------------
#define MASK_AnyState                     (~0)

/* Inquire version numbers and set debug logging */
#define MASK_fmi3GetVersion               MASK_AnyState
#define MASK_fmi3SetDebugLogging          MASK_AnyState

/* Creation and destruction of FMU instances */
#define MASK_fmi3InstantiateInstantiateModelExchange MASK_AnyState
#define MASK_fmi3InstantiateCoSimulation             MASK_AnyState
#define MASK_fmi3InstantiateScheduledExectuion       MASK_AnyState
#define MASK_fmi3FreeInstance                        MASK_AnyState

/* Enter and exit initialization mode, terminate and reset */
#define MASK_fmi3EnterInitializationMode  Instantiated
#define MASK_fmi3ExitInitializationMode   InitializationMode
#define MASK_fmi3EnterEventMode           (ContinuousTimeMode | StepMode)
#define MASK_fmi3Terminate                (ContinuousTimeMode | StepMode | StepDiscarded | EventMode | ClockActivationMode | ReconfigurationMode)
#define MASK_fmi3Reset                    MASK_AnyState

/* Common Functions */

/* Getting and setting variable values */
#define MASK_fmi3GetFloat32               (Instantiated | InitializationMode | ConfigurationMode | ReconfigurationMode | EventMode | ContinuousTimeMode | StepMode | ClockActivationMode | IntermediateUpdateMode | Terminated)
#define MASK_fmi3GetFloat64               MASK_fmi3GetFloat32
#define MASK_fmi3GetInt8                  MASK_fmi3GetFloat32
#define MASK_fmi3GetUInt8                 MASK_fmi3GetFloat32
#define MASK_fmi3GetInt16                 MASK_fmi3GetFloat32
#define MASK_fmi3GetUInt16                MASK_fmi3GetFloat32
#define MASK_fmi3GetInt32                 MASK_fmi3GetFloat32
#define MASK_fmi3GetUInt32                MASK_fmi3GetFloat32
#define MASK_fmi3GetInt64                 MASK_fmi3GetFloat32
#define MASK_fmi3GetUInt64                MASK_fmi3GetFloat32
#define MASK_fmi3GetBoolean               MASK_fmi3GetFloat32
#define MASK_fmi3GetString                MASK_fmi3GetFloat32
#define MASK_fmi3GetBinary                MASK_fmi3GetFloat32
#define MASK_fmi3GetClock                 MASK_AnyState

#define MASK_fmi3SetFloat32               (Instantiated | InitializationMode | ConfigurationMode | ReconfigurationMode | EventMode | ContinuousTimeMode | StepMode | ClockActivationMode | IntermediateUpdateMode | Terminated)
#define MASK_fmi3SetFloat64               MASK_fmi3SetFloat32
#define MASK_fmi3SetInt8                  (Instantiated | ConfigurationMode | ReconfigurationMode | InitializationMode | EventMode | StepMode | ClockActivationMode | Terminated)
#define MASK_fmi3SetUInt8                 MASK_fmi3SetInt8
#define MASK_fmi3SetInt16                 MASK_fmi3SetInt8
#define MASK_fmi3SetUInt16                MASK_fmi3SetInt8
#define MASK_fmi3SetInt32                 MASK_fmi3SetInt8
#define MASK_fmi3SetUInt32                MASK_fmi3SetInt8
#define MASK_fmi3SetInt64                 MASK_fmi3SetInt8
#define MASK_fmi3SetUInt64                MASK_fmi3SetInt8
#define MASK_fmi3SetBoolean               MASK_fmi3SetInt8
#define MASK_fmi3SetString                MASK_fmi3SetInt8
#define MASK_fmi3SetBinary                MASK_fmi3SetInt8
#define MASK_fmi3SetClock                 MASK_AnyState

/* Getting Variable Dependency Information */
#define MASK_fmi3GetNumberOfVariableDependencies  MASK_AnyState
#define MASK_fmi3GetVariableDependencies          MASK_AnyState

/* Getting and setting the internal FMU state */
#define MASK_fmi3GetFMUState              MASK_AnyState
#define MASK_fmi3SetFMUState              MASK_AnyState
#define MASK_fmi3FreeFMUState             MASK_AnyState
#define MASK_fmi3SerializedFMUStateSize   MASK_AnyState
#define MASK_fmi3SerializeFMUState        MASK_AnyState
#define MASK_fmi3DeserializeFMUState      MASK_AnyState

/* Getting partial derivatives */
#define MASK_fmi3GetDirectionalDerivative (InitializationMode | StepMode | EventMode | ContinuousTimeMode | Terminated)
#define MASK_fmi3GetAdjointDerivative     MASK_fmi3GetDirectionalDerivative

/* Entering and exiting the Configuration or Reconfiguration Mode */
#define MASK_fmi3EnterConfigurationMode   (Instantiated | StepMode | EventMode | ClockActivationMode)
#define MASK_fmi3ExitConfigurationMode    (ConfigurationMode | ReconfigurationMode)

/* Clock related functions */
// TODO: fix masks
#define MASK_fmi3GetIntervalDecimal        MASK_AnyState
#define MASK_fmi3GetIntervalFraction       MASK_AnyState
#define MASK_fmi3SetIntervalDecimal        MASK_AnyState
#define MASK_fmi3SetIntervalFraction       MASK_AnyState
#define MASK_fmi3NewDiscreteStates         MASK_AnyState

/* Functions for Model Exchange */

#define MASK_fmi3EnterContinuousTimeMode       EventMode
#define MASK_fmi3CompletedIntegratorStep       ContinuousTimeMode

/* Providing independent variables and re-initialization of caching */
#define MASK_fmi3SetTime                       (EventMode | ContinuousTimeMode)
#define MASK_fmi3SetContinuousStates           ContinuousTimeMode

/* Evaluation of the model equations */
#define MASK_fmi3GetContinuousStateDerivatives (InitializationMode | EventMode | ContinuousTimeMode | Terminated)
#define MASK_fmi3GetEventIndicators            MASK_fmi3GetContinuousStateDerivatives
#define MASK_fmi3GetContinuousStates           MASK_fmi3GetContinuousStateDerivatives
#define MASK_fmi3GetNominalsOfContinuousStates MASK_fmi3GetContinuousStateDerivatives

#define MASK_fmi3GetNumberOfContinuousStates   MASK_AnyState
#define MASK_fmi3GetNumberOfEventIndicators    MASK_AnyState

/* Functions for Co-Simulation */

#define MASK_fmi3EnterStepMode            (InitializationMode | EventMode)
#define MASK_fmi3SetInputDerivatives      (Instantiated | InitializationMode | StepMode)
#define MASK_fmi3GetOutputDerivatives     (StepMode | StepDiscarded | Terminated)
#define MASK_fmi3DoStep                   StepMode
#define MASK_fmi3ActivateModelPartition   ClockActivationMode
#define MASK_fmi3DoEarlyReturn            IntermediateUpdateMode
#define MASK_fmi3GetDoStepDiscardedStatus StepMode

// ---------------------------------------------------------------------------
// Private helpers used below to validate function arguments
// ---------------------------------------------------------------------------

#define NOT_IMPLEMENTED \
do { \
    ModelInstance *comp = (ModelInstance *)instance; \
    logError(comp, "Function is not implemented."); \
    return fmi3Error; \
} while (0)

#define BEGIN_FUNCTION(F) \
Status status = OK; \
if (!instance) return fmi3Error; \
ModelInstance *S = (ModelInstance *)instance; \
if (!allowedState(S, MASK_fmi3##F, #F)) CALL(Error);

#define END_FUNCTION() \
TERMINATE: \
    return (fmi3Status)status;

static bool allowedState(ModelInstance *instance, int statesExpected, char *name) {

    if (!instance) {
        return false;
    }

    if (!(instance->state & statesExpected)) {
        logError(instance, "fmi3%s: Illegal call sequence.", name);
        return false;
    }

    return true;
}



    


double set_variable(unsigned int vr, double value, ModelInstance* instance) {
    if (!instance) {
        printf("Error: instance is null\n");
        return 0.0;
    }
    double valueList []= {value};
    size_t index = 0;
    setFloat64(instance, vr, valueList, 1, &index);
    return value;
}

const char* getVariableType(fmi3Instance instance, fmi3ValueReference vr) {
    // Query the variable attributes to determine its type
    fmi3Float64 realValue;
    fmi3Int64 intValue;
    fmi3Boolean boolValue;
    fmi3String stringValue;

    // Try to get the variable as a Real
    if (fmi3GetFloat64(instance, &vr, 1, &realValue, 1) == fmi3OK) {
        return "Float64";
    }

    // Try to get the variable as an Integer
    if (fmi3GetInt64(instance, &vr, 1, &intValue, 1) == fmi3OK) {
        return "Int64";
    }

    // Try to get the variable as a Boolean
    if (fmi3GetBoolean(instance, &vr, 1, &boolValue, 1) == fmi3OK) {
        return "Boolean";
    }

    // Try to get the variable as a String
    if (fmi3GetString(instance, &vr, 1, &stringValue, 1) == fmi3OK) {
        return "String";
    }

    // If none of the above, the type is unknown
    return "Unknown";
}

// Parse JSON specification (placeholder implementation)
void logMismatch(const char* inputName, const char* expectedType, const char* providedType) {
    fprintf(stderr, "Type mismatch for RTLola input '%s': expected %s, got %s\n",
            inputName, expectedType, providedType);
    
}

bool RTLolaMonitor_ValidateTypes(RTLolaMonitor* monitor, fmi3Instance instance) {

    //divide the these 3 checks individually for debugging 

    if (!monitor) {
        fprintf(stderr, "RTLolaMonitor_ValidateTypes: Monitor is NULL.\n");
    }
    if (!instance) {
        fprintf(stderr, "RTLolaMonitor_ValidateTypes: Instance is NULL.\n");
    }
    if (!monitor->rtlola_spec) {
        fprintf(stderr, "RTLolaMonitor_ValidateTypes: RTLola specification is NULL.\n");
        
    }
    if (!monitor || !monitor->rtlola_spec || !monitor->monitored_vrs) {
        fprintf(stderr, "RTLolaMonitor_ValidateTypes: Invalid monitor or specification.\n");
       // return false;
    }


    for (size_t i = 0; i < monitor->num_vars; i++) {
        const char* var_name = getRTLolaHeaderVariableName(monitor->monitored_vrs[i]);
        bool found = false;

        // Search for the variable in the RTLola specification inputs
        for (int j = 0; j < monitor->rtlola_spec->input_count; j++) {
            if (strcmp(monitor->rtlola_spec->inputs[j].name, var_name) == 0) {
                monitor->rtlola_spec->inputs[j].vr = monitor->monitored_vrs[i];
                
                found = true;

                // Get the type from the FMU
                const char* actual_type = getVariableType(instance, monitor->monitored_vrs[i]);
                if (!actual_type) {
                    fprintf(stderr, "Variable %s not found in FMU.\n", var_name);
                    return false;
                }

                // Check if the type matches
                const char* expected_type = monitor->rtlola_spec->inputs[j].type_;
                if (strcmp(expected_type, actual_type) != 0) {
                    logMismatch(var_name, expected_type, actual_type);
                    monitor->is_active = false;
                    RTLolaMonitor_Cleanup(monitor);
                    return false;
                }
                break;
            }
        }

        if (!found) {
            fprintf(stderr, "Variable %s not found in RTLola specification.\n", var_name);
            return false;
        }
    }

    return true;
}


void Native_RTLolaMonitor_Instantiate(ModelInstance *comp, const char* spec_path, const unsigned int* vrs, size_t num_vars, bool execMode) {
    
    if (!comp) {
        fprintf(stderr, "RTLolaMonitor_Instantiate: Component is NULL.\n");
        return;
    }
    RTLolaMonitor* monitor = &comp->rtlola_monitor;
    if (!monitor) {
        fprintf(stderr, "RTLolaMonitor_Init: Monitor is NULL.\n");
        return;
    }

    monitor->execMode = execMode;

    monitor->spec_path = strdup(spec_path);
    if (!monitor->spec_path) {
        fprintf(stderr, "RTLolaMonitor_Init: Failed to allocate memory for spec_path.\n");
        return;
    }

    monitor->num_vars = num_vars;
    if (num_vars > 0) {
        monitor->monitored_vrs = malloc(num_vars * sizeof(unsigned int));
        if (!monitor->monitored_vrs) {
            fprintf(stderr, "RTLolaMonitor_Init: Failed to allocate memory for monitored_vrs.\n");
            //free(monitor->spec_path); // Clean up spec_path if allocation fails
            monitor->spec_path = NULL;
            return;
        }
        memcpy(monitor->monitored_vrs, vrs, num_vars * sizeof(unsigned int));
    } else {
        monitor->monitored_vrs = NULL;
    }  
    /*
    if(!fileExists(monitor->spec_path)) {
        free(monitor->monitored_vrs);
        monitor->monitored_vrs = NULL;
        free(monitor->spec_path);
        monitor->spec_path = NULL;
        logFormatted(comp, LOG_ERROR, "RTLOLA", 
        "Spec file not found: %s", 
        comp->rtlola_spec);
        return;
        
        
    }
    logFormatted(comp, LOG_INFO, "RTLOLA", 
    "Loading specification file: %s", 
    comp->rtlola_spec);
    */
   

    monitor->is_active = false;
}


void RTLolaMonitor_Init(RTLolaMonitor* monitor, const char* spec_path, const unsigned int* vrs, size_t num_vars, bool execMode) {
    if (!monitor) {
        fprintf(stderr, "RTLolaMonitor_Init: Monitor is NULL.\n");
        return;
    }
    monitor->execMode = execMode;
    // Copy the specification path
    monitor->spec_path = strdup(spec_path);
    if (!monitor->spec_path) {
        fprintf(stderr, "RTLolaMonitor_Init: Failed to allocate memory for spec_path.\n");
        return;
    }
    printf("RTLola spec path is %s\n", monitor->spec_path);

    // Copy the monitored variables
    monitor->num_vars = num_vars;
    if (num_vars > 0) {
        monitor->monitored_vrs = malloc(num_vars * sizeof(unsigned int));
        if (!monitor->monitored_vrs) {
            fprintf(stderr, "RTLolaMonitor_Init: Failed to allocate memory for monitored_vrs.\n");
            //free(monitor->spec_path); // Clean up spec_path if allocation fails
            monitor->spec_path = NULL;
            return;
        }
        memcpy(monitor->monitored_vrs, vrs, num_vars * sizeof(unsigned int));
    } else {
        monitor->monitored_vrs = NULL;
    }
    
    
    char *json_spec = parse_specification(monitor->spec_path);
    RTLolaMonitor_ParseSpec(monitor, json_spec);
  
    // Initialize pipe file descriptors to invalid values
    monitor->input_pipe[0] = -1;
    monitor->input_pipe[1] = -1;
    monitor->output_pipe[0] = -1;
    monitor->output_pipe[1] = -1;

    // Initialize process ID to invalid value
    monitor->child_pid = -1;

    // Initialize file streams to NULL
    monitor->input_stream = NULL;
    monitor->output_stream = NULL;

    // Set the monitor as inactive initially
    monitor->is_active = false;
}

bool RTLolaMonitor_Start(RTLolaMonitor* monitor) {
    if (!monitor) {
        fprintf(stderr, "RTLolaMonitor_Start: Monitor is NULL.\n");
        return 0;
    }

    if (monitor->is_active) {
        fprintf(stderr, "RTLolaMonitor_Start: Monitor is already active.\n");
        return 1;
    }

    if (pipe(monitor->input_pipe) != 0 || pipe(monitor->output_pipe) != 0) {
        fprintf(stderr, "RTLolaMonitor_Start: Failed to create pipes.\n");
        return 0;
    }

    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "RTLolaMonitor_Start: Failed to fork process.\n");
        return 0;
    }

    char* execModeArg;
    char* timeFormatArg;
    
    if (monitor->execMode) {
        execModeArg = "--online";
        timeFormatArg = NULL; // No time format needed for online mode
    } else {
        execModeArg = "--offline";
        timeFormatArg = "relative-float-secs"; // Time format for offline mode
    }

    if (pid == 0) { // Child process
        setvbuf(stdout, NULL, _IOLBF, 0); // Line buffering
        close(monitor->input_pipe[1]); // Close unused write end
        close(monitor->output_pipe[0]); // Close unused read end

        dup2(monitor->input_pipe[0], STDIN_FILENO);
        dup2(monitor->output_pipe[1], STDOUT_FILENO);

        if (monitor->execMode) {
            // Online mode: Do not pass timeFormatArg
            execlp("rtlola-cli", "rtlola-cli", "monitor", "--stdin", "--stdout", execModeArg, monitor->spec_path, NULL);
        } else {
            // Offline mode: Pass timeFormatArg
            execlp("rtlola-cli", "rtlola-cli", "monitor", "--stdin", "--stdout", execModeArg, timeFormatArg, monitor->spec_path, NULL);
        }
    
        fprintf(stderr, "Child: Failed to execute rtlola-cli.\n");
        exit(1);
    } else { // Parent process
        close(monitor->input_pipe[0]); // Close unused read end
        close(monitor->output_pipe[1]); // Close unused write end

        // Set the output pipe to non-blocking mode
        int flags = fcntl(monitor->output_pipe[0], F_GETFL, 0);
        fcntl(monitor->output_pipe[0], F_SETFL, flags | O_NONBLOCK);

        monitor->child_pid = pid;
        monitor->is_active = true;

        monitor->input_stream = fdopen(monitor->input_pipe[1], "w");
        if (!monitor->input_stream) {
            fprintf(stderr, "RTLolaMonitor_Start: Failed to open input stream.\n");
            return 0;
        }

        monitor->output_stream = fdopen(monitor->output_pipe[0], "r");
        if (!monitor->output_stream) {
            fprintf(stderr, "RTLolaMonitor_Start: Failed to open output stream.\n");
            return 0;
        }
       // printf("Parent: Input pipe: read=%d, write=%d\n", monitor->input_pipe[0], monitor->input_pipe[1]);
       // printf("Parent: Output pipe: read=%d, write=%d\n", monitor->output_pipe[0], monitor->output_pipe[1]);

    // Debugging: Print the CSV header being sent
        printf("Parent: Sending CSV header:\n");

        // Write CSV header to the process
        fprintf(monitor->input_stream, "time");

        for (size_t i = 0; i < monitor->num_vars; i++) {
            fprintf(monitor->input_stream, ",%s", getRTLolaHeaderVariableName(monitor->monitored_vrs[i]));
        }
        fprintf(monitor->input_stream, "\n");
        fflush(monitor->input_stream);

        printf("RTLola process started with PID %d.\n", pid);
        return 1;
    }
}

bool rtlola_readoutput(RTLolaMonitor* monitor) {
    if (!monitor || !monitor->is_active || !monitor->output_stream) {
        fprintf(stderr, "rtlola_readoutput: Monitor is not active or invalid.\n");
        return false;
    }

    char buffer[1024];
    char result[4096] = {0}; // Adjust size as needed
    size_t total_read = 0;
    int retry_count = 0;
    const int max_retries = 5; // Maximum number of retries
    const int retry_delay_us = 1000; // Delay between retries in microseconds

    // Read the entire output buffer
    while (total_read < sizeof(result) - 1) {
        size_t bytes_read = fread(buffer, 1, sizeof(buffer) - 1, monitor->output_stream);

        if (bytes_read > 0) {
            // Append the read data to the result buffer
            memcpy(result + total_read, buffer, bytes_read);
            total_read += bytes_read;
            retry_count = 0; // Reset retry count on successful read
        } else {
            if (feof(monitor->output_stream)) {
                // End of stream
                break;
            } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available, retry after a short delay
                if (retry_count < max_retries) {
                    usleep(retry_delay_us);
                    retry_count++;
                } else {
                    fprintf(stderr, "Timeout: No data available after %d retries.\n", max_retries);
                    break;
                }
            } else {
                // Error reading from the stream
                fprintf(stderr, "Error reading from RTLola output stream: %s\n", strerror(errno));
                monitor->is_active = false;
                RTLolaMonitor_Cleanup(monitor);
                return false;
            }
        }
    }

    // Null-terminate the result buffer
    result[total_read] = '\0';

   
   
    // Print the RTLola output
    if (total_read > 0) {
        printf("RTLola Output:\n%s\n", result);

        // Evaluate triggers based on the output
        if(monitor->rtlola_spec){
            RTLolaMonitor_EvaluateTriggers(monitor, result);
        }
    } else {
        printf("No output from RTLola.\n");
    }

    return true;

}

bool RTLolaMonitor_HandleSpecSwitch(ModelInstance* comp) {
    if (!comp) {
        fprintf(stderr, "RTLolaMonitor_HandleSpecSwitch: ModelInstance is NULL.\n");
        return false;
    }

    // Check if a spec switch is requested
    if (comp->rtlola_monitor.spec_switch_requested) {
        printf("Handling RTLola specification switch...\n");

        // Stop the current RTLola process
        printf("Stopping current RTLola process...\n");
        RTLolaMonitor_Cleanup(&comp->rtlola_monitor);

        // Update the monitor's specification path
        if (comp->rtlola_spec) {
            printf("Updating specification path to: %s\n", comp->rtlola_spec);

            // Free the old specification path if it exists
            if (comp->rtlola_monitor.spec_path) {
                free((char*)comp->rtlola_monitor.spec_path);
                comp->rtlola_monitor.spec_path = NULL;
            }

            // Allocate memory for the new specification path and copy it
            comp->rtlola_monitor.spec_path = strdup(comp->rtlola_spec);
            if (!comp->rtlola_monitor.spec_path) {
                fprintf(stderr, "RTLolaMonitor_HandleSpecSwitch: Failed to allocate memory for new spec path.\n");
                return false;
            }
        }

        const unsigned int monitored_vrs[] = {vr_h};  // Add more as needed
        const size_t num_vars = sizeof(monitored_vrs)/sizeof(monitored_vrs[0]);
        // Allocate memory for the new monitored_vrs and copy the values
        comp->rtlola_monitor.num_vars = num_vars;
        comp->rtlola_monitor.monitored_vrs = malloc(comp->rtlola_monitor.num_vars * sizeof(unsigned int));
        if (!comp->rtlola_monitor.monitored_vrs) {
            fprintf(stderr, "RTLolaMonitor_HandleSpecSwitch: Failed to allocate memory for monitored_vrs.\n");
            return false;
        }
        
        memcpy(comp->rtlola_monitor.monitored_vrs, monitored_vrs, comp->rtlola_monitor.num_vars * sizeof(unsigned int));
        printf("Updating monitored variables... numeber of vars and monitored_vrs %ld %d\n", comp->rtlola_monitor.num_vars, comp->rtlola_monitor.monitored_vrs[0]);
        
        // Parse the new RTLola specification
        //RTLolaMonitor_ParseSpec(&comp->rtlola_monitor,  "bouncing_ball.json");
        
        // Update the number of monitored variables
        //comp->rtlola_monitor.num_vars = comp->rtlola_num_vars;

        // Start a new RTLola process with the new specification
        comp->rtlola_monitor.spec_switch_requested = false;
        printf("Starting new RTLola process...\n");
        if (!RTLolaMonitor_Start(&comp->rtlola_monitor)) {
            fprintf(stderr, "RTLolaMonitor_HandleSpecSwitch: Failed to restart RTLola with new spec.\n");
            return false;
        }

        // Clear the spec switch flag

        printf("RTLola specification switch completed successfully.\n");
    }

    return true;
}

bool RTLolaMonitor_SendData_v3(RTLolaMonitor* monitor, double time, fmi3Instance instance) {
    // ModelInstance* tempInstance = (ModelInstance*)instance;
     if (!monitor || !monitor->is_active || !monitor->input_stream) {
         fprintf(stderr, "RTLolaMonitor_SendData: Monitor is not active or invalid.\n");
         return false;
     }
     
     // Debugging: Print the data being sent
     printf("RTLolaMonitor_SendData: Sending data for time=%.6f\n", time);
 
     // Build and send the CSV line
     fprintf(monitor->input_stream, "%.6f", time);
    // double values[1]; // Array to store the retrieved value
     double valueList []= {0.0};
     for (size_t i = 0; i < monitor->num_vars; i++) {
         size_t nValues = 1; // Number of values to retrieve
         //size_t index =0;
         // Retrieve the value using fmi3GetFloat64
         fmi3Status valueStatus = fmi3GetFloat64(instance, &monitor->monitored_vrs[i], monitor->num_vars, valueList, nValues);
         
         if (valueStatus != fmi3OK) {
             fprintf(stderr, "Failed to retrieve value for variable %u.\n", monitor->monitored_vrs[i]);
             return false;
         }
 
         // Use the retrieved value
         double value = valueList[0];
         fprintf(monitor->input_stream, ",%.6f", value);
         
        // Debugging: Print each variable value
        //printf("RTLolaMonitor_SendData: Sending variable %s=%.6f\n", getRTLolaHeaderVariableName(monitor->monitored_vrs[i]), value);
     }
     fprintf(monitor->input_stream, "\n");
     fflush(monitor->input_stream);
 
     // Debugging: Print confirmation that data was sent
     //printf("RTLolaMonitor_SendData: Data sent successfully.\n");
 
     // Read the output immediately after sending data
     if (!rtlola_readoutput(monitor)) {
         fprintf(stderr, "Failed to read RTLola output.\n");
         return false;
     }
 
     return true;
}



const char** extract_input_names_from_spec(const RTLolaSpec* spec) {
    if (!spec || !spec->inputs) {
        fprintf(stderr, "extract_input_names: Invalid spec\n");
        return NULL;
    }

    const char** names = (const char**)malloc(spec->input_count * sizeof(const char*));
    if (!names) {
        fprintf(stderr, "extract_input_names: Memory allocation failed\n");
        return NULL;
    }

    for (int i = 0; i < spec->input_count; i++) {
        if (!spec->inputs[i].name) {
            fprintf(stderr, "extract_input_names: Input %d missing name\n", i);
            return NULL;
        }
        names[i] = strdup(spec->inputs[i].name);
        if (!names[i]) {
            fprintf(stderr, "extract_input_names: strdup failed for input %d\n", i);
        
            return NULL;
        }
    }

    return names;
}

void free_input_names(const char** names) {
    if (!names) return;
    
    // Free each string in the array
    for (const char** name_ptr = names; *name_ptr; name_ptr++) {
        free((void*)*name_ptr); // Cast away const to free
    }
    
    // Free the array itself
    free(names);
}

bool Native_RTLolaMonitor_Init(RTLolaMonitor *monitor, fmi3Instance instance) {
    // Validate input parameters
    
    if (!monitor || !monitor->spec_path) {
        fprintf(stderr, "Error: Invalid monitor or spec path\n");
        return false;
    }
   
    // Extract basic configuration from monitor
    const char* spec_path = monitor->spec_path;
    uint64_t num_vars = monitor->num_vars;
    
    char *json_spec = parse_specification(monitor->spec_path);
    RTLolaMonitor_ParseSpec(monitor, json_spec);

    // Extract input names from specification
    const char** input_names = extract_input_names_from_spec(monitor->rtlola_spec);
    if (!input_names) {
        fprintf(stderr, "Error: Failed to extract input names from spec\n");
        return false;
    }
    monitor->input_names = input_names;

    // Create new RTLola monitor instance
    RTLolaMonitorHandle* handle = rtlola_monitor_new(
        spec_path,      // Path to specification file
        1000,          // Timeout in milliseconds
        input_names,    // Array of input names
        num_vars        // Number of input variables
    );
    
    if (!handle) {
        fprintf(stderr, "Error: Failed to create RTLola monitor instance\n");
        free_input_names(input_names);
        return false;
    }
    logFormatted(instance, LOG_INFO, "RTLOLA",
        "Created RTLola monitor instance with spec: %s\n", spec_path);

    // Store handle and start monitoring
    monitor->rtlola_handle = handle;
    
    return true;
}


bool Native_RTLolaMonitor_Start(RTLolaMonitor *monitor, fmi3Instance instance) 
{
    // Start the monitor
    logFormatted(instance, LOG_INFO, "RTLOLA",
        "Starting RTLola monitor...\n");


    if (!rtlola_monitor_start(monitor->rtlola_handle)) {
        fprintf(stderr, "Error: Failed to start RTLola monitor\n");
        rtlola_monitor_free(monitor->rtlola_handle);
        free_input_names(monitor->input_names);
        return false;
    }
    return true;
}

bool Native_Handle_Spec_Switch(RTLolaMonitor* monitor, fmi3Instance instance, ModelState state) {
    if (!monitor) {
        fprintf(stderr, "Error: Invalid monitor or spec path\n");
        return false;
    }

    if (!monitor->spec_switch_requested) {
        return false;
    }
   
    
    if (state ==  InitializationMode){
        logFormatted(instance, LOG_INFO, "RTLOLA",
            "Setting initial RTLola specification...\n");
    } else  {
        logFormatted(instance, LOG_INFO, "RTLOLA",
            "Handling RTLola specification switch...\n");

        logFormatted(instance, LOG_INFO, "RTLOLA",
            "Setting new RTLola specification...\n");
    }

    // 1. Stop current monitor if active
    if (monitor->rtlola_handle) {
        rtlola_monitor_free(monitor->rtlola_handle);
        monitor->rtlola_handle = NULL;
    }

    // 2. Update specification path
  
    if (!monitor->spec_path) {
        fprintf(stderr, "Error: Failed to allocate memory for new spec path\n");
        return false;
    }

    // 3. Parse new specification
    char* json_spec = parse_specification(monitor->spec_path);
    printf("Parsed new specficaition successfully\n");
    logFormatted(instance, LOG_INFO, "RTLOLA",
        "Parsed new RTLola specification ");
    RTLolaMonitor_ParseSpec(monitor, json_spec);
    if (!json_spec) {
        fprintf(stderr, "Error: Failed to parse new specification\n");
        return false;
    }

    // 4. Extract input names via Rust FFI
    const char** input_names = extract_input_names_from_spec(monitor->rtlola_spec);
    if (!input_names) {
        fprintf(stderr, "Error: Failed to extract input names\n");
      //  free(json_spec);
        return false;
    }
    monitor->input_names = input_names;
    const char* new_spec_path = monitor->spec_path;
    // 5. Create new monitor instance
    RTLolaMonitorHandle* handle = rtlola_monitor_new(
        new_spec_path,
        1000, // timeout_ms
        input_names,
        monitor->num_vars
    );
    printf("Starting new RTLola monitor instance...\n");
    
    //free(json_spec);
      


    if (!handle) {
        fprintf(stderr, "Error: Failed to create new monitor instance\n");
        return false;
    }

    RTLolaMonitor_ValidateTypes(monitor, instance); 
    // 6. Start new monitor
    
    // 7. Update monitor state
    monitor->rtlola_handle = handle;
    monitor->spec_switch_requested = false;
   // printf("Restarting with a new RTLola monitor instance...\n");
    //Native_RTLolaMonitor_Start(monitor);
    
    return true;
}

bool Native_RTLolaMonitor_SendData(RTLolaMonitor *monitor, fmi3Instance instance) {
    if (!monitor || !monitor->rtlola_handle || !instance) {
        fprintf(stderr, "Error: Invalid monitor or instance\n");
        return false;
    }


    size_t num_vars = monitor->num_vars;
    if (num_vars == 0) {
        fprintf(stderr, "Warning: No variables to monitor\n");
        return true;
    }

    // Allocate array of input structures
    RTLolaInput *inputs = malloc(num_vars * sizeof(RTLolaInput));
    if (!inputs) {
        fprintf(stderr, "Error: Failed to allocate memory for inputs\n");
        return false;
    }

    // Get current simulation time
    fmi3Float64 currentTime;
    fmi3ValueReference time_vr = vr_time;
    if (fmi3GetFloat64(instance, &time_vr, 1, &currentTime, 1) != fmi3OK) {
        fprintf(stderr, "Error: Failed to get simulation time\n");
        free(inputs);
        return false;
    }

    // Populate each input with actual values from FMU
    for (size_t i = 0; i < num_vars; i++) {
        fmi3ValueReference vr = monitor->monitored_vrs[i];
        inputs[i].name = monitor->input_names[i];
  
        // Determine type and get value
        const char* type_str = getVariableType(instance, vr);
        if (!type_str) {
            fprintf(stderr, "Error: Failed to determine type for variable %zu\n", i);
            continue;
        }

        //Getter functions for only the types supported by RTLola
        if (strcmp(type_str, "Float64") == 0) {
            inputs[i].type = RTLOLA_TYPE_FLOAT64;
            if (fmi3GetFloat64(instance, &vr, 1, &inputs[i].value.float64_val, 1) != fmi3OK) {
                fprintf(stderr, "Warning: Failed to get Float64 value for %s\n", inputs[i].name);
            }
        }
        else if (strcmp(type_str, "Int64") == 0) {
            inputs[i].type = RTLOLA_TYPE_INT64;
            if (fmi3GetInt64(instance, &vr, 1, &inputs[i].value.int64_val, 1) != fmi3OK) {
                fprintf(stderr, "Warning: Failed to get Int64 value for %s\n", inputs[i].name);
            }
        }
        else if (strcmp(type_str, "UInt64") == 0) {
            inputs[i].type = RTLOLA_TYPE_UINT64;
            if (fmi3GetUInt64(instance, &vr, 1, &inputs[i].value.uint64_val, 1) != fmi3OK) {
                fprintf(stderr, "Warning: Failed to get UInt64 value for %s\n", inputs[i].name);
            }
        }
        else if (strcmp(type_str, "Boolean") == 0) {
            inputs[i].type = RTLOLA_TYPE_BOOL;
            fmi3Boolean temp;
            if (fmi3GetBoolean(instance, &vr, 1, &temp, 1) == fmi3OK) {
                inputs[i].value.bool_val = temp;
            } else {
                fprintf(stderr, "Warning: Failed to get Boolean value for %s\n", inputs[i].name);
            }
        }
        else if (strcmp(type_str, "String") == 0) {
            inputs[i].type = RTLOLA_TYPE_STRING;
            fmi3String temp;
            if (fmi3GetString(instance, &vr, 1, &temp, 1) == fmi3OK) {
                inputs[i].value.string_val = temp;
            } else {
                fprintf(stderr, "Warning: Failed to get String value for %s\n", inputs[i].name);
            }
        }
        else {
            fprintf(stderr, "Warning: Unknown type '%s' for variable %s\n", 
                    type_str, inputs[i].name);
            inputs[i].type = RTLOLA_TYPE_FLOAT64; // Default fallback
            inputs[i].value.float64_val = 0.0;
        }
    }

    // Process all inputs at current time
    char *result = rtlola_process_inputs(monitor->rtlola_handle, inputs, num_vars, currentTime);
    monitor->last_output = result; 

    logFormatted(instance, LOG_INFO, "RTLOLA",
        "RTLola Evaluation process result: \n%s", result ? result : "NULL");
    if (result != NULL) {
       //printf("%s\n", result);
        // set rtlola_output variable to output of RTLola process 
        const fmi3String output = monitor->last_output;  // fmi3String is const char*
        const fmi3ValueReference vr_output = vr_rtlola_output; 
        fmi3SetString(instance, &vr_output, 1, &output, 1);
        rtlola_free_string(result); // Don't forget to free the memory!
        


    } else {
        printf("Error processing inputs\n");
    }

    
    // Cleanup
    free(inputs);
    return true;
}
/*
Functions for RTLola evalutations
*/


void RTLolaMonitor_ParseSpec(RTLolaMonitor* monitor, const char* json_spec) {


    if (!monitor || !json_spec) {
        fprintf(stderr, "Invalid arguments passed to RTLolaMonitor_ParseSpec.\n");
        return;
    }
    
    // Parse the JSON string
    cJSON *json = cJSON_Parse(json_spec);
    if (!json) {
        fprintf(stderr, "Error parsing JSON: %s\n", cJSON_GetErrorPtr());
        return;
    }

    // Allocate memory for the RTLola specification
    monitor->rtlola_spec = (RTLolaSpec*)malloc(sizeof(RTLolaSpec));
    if (!monitor->rtlola_spec) {
        fprintf(stderr, "Memory allocation failed for RTLolaSpec.\n");
        cJSON_Delete(json);
        return;
    }

    // Parse inputs
    cJSON *inputs = cJSON_GetObjectItemCaseSensitive(json, "inputs");
    if (inputs && cJSON_IsArray(inputs)) {
        monitor->rtlola_spec->input_count = cJSON_GetArraySize(inputs);
        monitor->rtlola_spec->inputs = malloc(monitor->rtlola_spec->input_count * sizeof(Input));

        if (monitor->rtlola_spec->inputs) {
            for (int i = 0; i < monitor->rtlola_spec->input_count; i++) {
                cJSON *input = cJSON_GetArrayItem(inputs, i);
                cJSON *name = cJSON_GetObjectItemCaseSensitive(input, "name");
                cJSON *type_ = cJSON_GetObjectItemCaseSensitive(input, "type_");

                if (cJSON_IsString(name) && cJSON_IsString(type_)) {
                    monitor->rtlola_spec->inputs[i].name = strdup(name->valuestring);
                    monitor->rtlola_spec->inputs[i].type_ = strdup(type_->valuestring);
                } else {
                    monitor->rtlola_spec->inputs[i].name = NULL;
                    monitor->rtlola_spec->inputs[i].type_ = NULL;
                }
            }
        }
    } else {
        fprintf(stderr, "Invalid or missing 'inputs' array in JSON.\n");
        monitor->rtlola_spec->inputs = NULL;
        monitor->rtlola_spec->input_count = 0;
    }

    // Parse outputs (if needed)
    cJSON *outputs = cJSON_GetObjectItemCaseSensitive(json, "outputs");
    if (outputs && cJSON_IsArray(outputs)) {
        monitor->rtlola_spec->output_count = cJSON_GetArraySize(outputs);
        monitor->rtlola_spec->outputs = malloc(monitor->rtlola_spec->output_count * sizeof(Output));

        if (monitor->rtlola_spec->outputs) {
            for (int i = 0; i < monitor->rtlola_spec->output_count; i++) {
                cJSON *output = cJSON_GetArrayItem(outputs, i);
                cJSON *variable = cJSON_GetObjectItemCaseSensitive(output, "variable");
                cJSON *comparison = cJSON_GetObjectItemCaseSensitive(output, "comparison");

                if (cJSON_IsString(variable) && cJSON_IsString(comparison)) {
                    monitor->rtlola_spec->outputs[i].variable = strdup(variable->valuestring);
                    monitor->rtlola_spec->outputs[i].comparison = strdup(comparison->valuestring);
                } else {
                    monitor->rtlola_spec->outputs[i].variable = NULL;
                    monitor->rtlola_spec->outputs[i].comparison = NULL;
                }
            }
        }
    } else {
        fprintf(stderr, "Invalid or missing 'outputs' array in JSON.\n");
        monitor->rtlola_spec->outputs = NULL;
        monitor->rtlola_spec->output_count = 0;
    }

    // Parse triggers
    cJSON *triggers = cJSON_GetObjectItemCaseSensitive(json, "triggers");
    if (triggers && cJSON_IsArray(triggers)) {
        monitor->rtlola_spec->trigger_count = cJSON_GetArraySize(triggers);
        monitor->rtlola_spec->triggers = malloc(monitor->rtlola_spec->trigger_count * sizeof(Trigger));

        if (monitor->rtlola_spec->triggers) {
            for (int i = 0; i < monitor->rtlola_spec->trigger_count; i++) {
                cJSON *trigger = cJSON_GetArrayItem(triggers, i);
                cJSON *condition = cJSON_GetObjectItemCaseSensitive(trigger, "condition");
                cJSON *message = cJSON_GetObjectItemCaseSensitive(trigger, "message");

                if (cJSON_IsString(condition) && cJSON_IsString(message)) {
                    monitor->rtlola_spec->triggers[i].condition = strdup(condition->valuestring);
                    monitor->rtlola_spec->triggers[i].message = strdup(message->valuestring);
                } else {
                    monitor->rtlola_spec->triggers[i].condition = NULL;
                    monitor->rtlola_spec->triggers[i].message = NULL;
                }
            }
        }
    } else {
        fprintf(stderr, "Invalid or missing 'triggers' array in JSON.\n");
        monitor->rtlola_spec->triggers = NULL;
        monitor->rtlola_spec->trigger_count = 0;
    }
   /*
   logFormatted(comp, LOG_DEBUG, "RTLOLA", 
   "Parsing spec: %s (inputs=%d, triggers=%d)", 
   json_spec, 
   monitor->rtlola_spec->input_count, 
   monitor->rtlola_spec->trigger_count);
   
   if (!json_spec) {
    logFormatted(comp, LOG_ERROR, "RTLOLA", 
    "Failed to parse spec: %s", 
    monitor->spec_path);
    return;
    }

    logFormatted(comp, LOG_INFO, "RTLOLA", 
    "Spec parsed successfully: %s", 
    monitor->spec_path);

    */
        
    // Clean up
    cJSON_Delete(json);
}




// Evaluate triggers based on feedback
void RTLolaMonitor_EvaluateTriggers(RTLolaMonitor* monitor, const char* output) {
    if (!monitor || !monitor->rtlola_spec || !output) return;

    for (int i = 0; i < monitor->rtlola_spec->trigger_count; i++) {
        if (strstr(output, monitor->rtlola_spec->triggers[i].message)) {
            printf("Trigger detected: %s\n", monitor->rtlola_spec->triggers[i].message);
            RTLolaMonitor_TakeActions(monitor, monitor->rtlola_spec->triggers[i].message);
        }
    }
}

// Take actions based on triggered conditions
void RTLolaMonitor_TakeActions(RTLolaMonitor* monitor, const char* trigger_message) {
    if (!monitor || !trigger_message) return;

    // Example: Take actions based on the trigger message, or any abitrary condition you want to monitor
    if (strcmp(trigger_message, "Ball in motion") == 0) {
        // Action for "Ball in motion"
        printf("Action: Ball is in motion.\n");
    } else if (strcmp(trigger_message, "Ball hit ground") == 0) {
        // Action for "Ball hit ground"
        printf("Action: Ball hit the ground.\n");
    }
    // Add more actions as needed
}

void RTLolaMonitor_Cleanup(RTLolaMonitor* monitor) {
    if (!monitor) return;

    printf("Cleaning up RTLola monitor.\n");

    // Close input and output streams
    if (monitor->input_stream) {
        fclose(monitor->input_stream);
        monitor->input_stream = NULL;
    }
    if (monitor->output_stream) {
        fclose(monitor->output_stream);
        monitor->output_stream = NULL;
    }

    // Close pipe file descriptors
    if (monitor->input_pipe[0] != -1) close(monitor->input_pipe[0]);
    if (monitor->input_pipe[1] != -1) close(monitor->input_pipe[1]);
    if (monitor->output_pipe[0] != -1) close(monitor->output_pipe[0]);
    if (monitor->output_pipe[1] != -1) close(monitor->output_pipe[1]);

    // Terminate the child process
    if (monitor->child_pid != -1) {
        kill(monitor->child_pid, SIGTERM);
        waitpid(monitor->child_pid, NULL, 0);
        monitor->child_pid = -1;
    }

    // Free allocated memory
    if (monitor->spec_path) {
        free((char*)monitor->spec_path);
        monitor->spec_path = NULL;
    }
    if (monitor->monitored_vrs) {
        free(monitor->monitored_vrs);
        monitor->monitored_vrs = NULL;
    }

    if (monitor->rtlola_spec) {
        // Free inputs
        if (monitor->rtlola_spec->inputs) {
            for (int i = 0; i < monitor->rtlola_spec->input_count; i++) {
                if (monitor->rtlola_spec->inputs[i].name) {
                    free((char*)monitor->rtlola_spec->inputs[i].name);
                }
                if (monitor->rtlola_spec->inputs[i].type_) {
                    free((char*)monitor->rtlola_spec->inputs[i].type_);
                }
            }
            free(monitor->rtlola_spec->inputs);
        }

        // Free outputs
        if (monitor->rtlola_spec->outputs) {
            for (int i = 0; i < monitor->rtlola_spec->output_count; i++) {
                if (monitor->rtlola_spec->outputs[i].variable) {
                    free((char*)monitor->rtlola_spec->outputs[i].variable);
                }
                if (monitor->rtlola_spec->outputs[i].comparison) {
                    free((char*)monitor->rtlola_spec->outputs[i].comparison);
                }
            }
            free(monitor->rtlola_spec->outputs);
        }

        // Free triggers
        if (monitor->rtlola_spec->triggers) {
            for (int i = 0; i < monitor->rtlola_spec->trigger_count; i++) {
                if (monitor->rtlola_spec->triggers[i].condition) {
                    free((char*)monitor->rtlola_spec->triggers[i].condition);
                }
                if (monitor->rtlola_spec->triggers[i].message) {
                    free((char*)monitor->rtlola_spec->triggers[i].message);
                }
            }
            free(monitor->rtlola_spec->triggers);
        }

        free(monitor->rtlola_spec);
        monitor->rtlola_spec = NULL;
    }

    // Reset the monitor state
    monitor->is_active = false;
    monitor->spec_switch_requested = false;
    monitor->spec_switch_pending = false;

    //printf("RTLola monitor cleaned up.\n");
}


/***************************************************
 Common Functions
 ****************************************************/

const char* fmi3GetVersion(void) {
    return fmi3Version;
}

fmi3Status fmi3SetDebugLogging(fmi3Instance instance,
                               fmi3Boolean loggingOn,
                               size_t nCategories,
                               const fmi3String categories[]) {

    BEGIN_FUNCTION(SetDebugLogging);

    CALL(setDebugLogging(S, loggingOn, nCategories, categories));

    END_FUNCTION();
}

fmi3Instance fmi3InstantiateModelExchange(
    fmi3String                 instanceName,
    fmi3String                 instantiationToken,
    fmi3String                 resourcePath,
    fmi3Boolean                visible,
    fmi3Boolean                loggingOn,
    fmi3InstanceEnvironment    instanceEnvironment,
    fmi3LogMessageCallback     logMessage) {

    UNUSED(visible);

#ifndef MODEL_EXCHANGE
    UNUSED(instanceName);
    UNUSED(instantiationToken);
    UNUSED(resourcePath);
    UNUSED(loggingOn);
    UNUSED(instanceEnvironment);
    UNUSED(logMessage);

    return NULL;
#else
    return createModelInstance(
        (loggerType)logMessage,
        NULL,
        instanceEnvironment,
        instanceName,
        instantiationToken,
        resourcePath,
        loggingOn,
        ModelExchange);
#endif
}

fmi3Instance fmi3InstantiateCoSimulation(
    fmi3String                     instanceName,
    fmi3String                     instantiationToken,
    fmi3String                     resourcePath,
    fmi3Boolean                    visible,
    fmi3Boolean                    loggingOn,
    fmi3Boolean                    eventModeUsed,
    fmi3Boolean                    earlyReturnAllowed,
    const fmi3ValueReference       requiredIntermediateVariables[],
    size_t                         nRequiredIntermediateVariables,
    fmi3InstanceEnvironment        instanceEnvironment,
    fmi3LogMessageCallback         logMessage,
    fmi3IntermediateUpdateCallback intermediateUpdate) {

    UNUSED(visible);
    UNUSED(requiredIntermediateVariables);
    UNUSED(nRequiredIntermediateVariables);


    
#ifndef EVENT_UPDATE
    if (eventModeUsed) {
        if (logMessage) {
            logMessage(instanceEnvironment, fmi3Error, "error", "Event Mode is not supported.");
        }
        return NULL;
    }
#endif

    ModelInstance *instance = createModelInstance(
        (loggerType)logMessage,
        (intermediateUpdateType)intermediateUpdate,
        instanceEnvironment,
        instanceName,
        instantiationToken,
        resourcePath,
        loggingOn,
        CoSimulation);

    if (instance) {
        instance->earlyReturnAllowed = earlyReturnAllowed;
        instance->eventModeUsed      = eventModeUsed;
        instance->state              = Instantiated;
    }
    //fmi3InitializeRTLola(instance);
    
    logFormatted(instance, LOG_INFO, "FMI CALL", 
    "FMU for co-simulation instantiated: name='%s', resourceLocation='%s'", 
    instanceName, resourcePath);
    

    return instance;
}

fmi3Instance fmi3InstantiateScheduledExecution(
    fmi3String                     instanceName,
    fmi3String                     instantiationToken,
    fmi3String                     resourcePath,
    fmi3Boolean                    visible,
    fmi3Boolean                    loggingOn,
    fmi3InstanceEnvironment        instanceEnvironment,
    fmi3LogMessageCallback         logMessage,
    fmi3ClockUpdateCallback        clockUpdate,
    fmi3LockPreemptionCallback     lockPreemption,
    fmi3UnlockPreemptionCallback   unlockPreemption) {

    UNUSED(visible);

#ifndef SCHEDULED_CO_SIMULATION

    UNUSED(instanceName);
    UNUSED(instantiationToken);
    UNUSED(resourcePath);
    UNUSED(loggingOn);
    UNUSED(instanceEnvironment);
    UNUSED(logMessage);
    UNUSED(clockUpdate);
    UNUSED(lockPreemption);
    UNUSED(unlockPreemption);

    return NULL;
#else
    ModelInstance *instance = createModelInstance(
        (loggerType)logMessage,
        NULL,
        instanceEnvironment,
        instanceName,
        instantiationToken,
        resourcePath,
        loggingOn,
        ScheduledExecution
    );

    if (instance) {
        instance->state = Instantiated;
        instance->clockUpdate = clockUpdate;
        instance->lockPreemption = lockPreemption;
        instance->unlockPreemption = unlockPreemption;
    }
     

    return instance;
#endif
}

void fmi3FreeInstance(fmi3Instance instance) {
    RTLolaMonitor_Cleanup(&((ModelInstance*)instance)->rtlola_monitor);
    freeModelInstance((ModelInstance*)instance);
}

fmi3Status fmi3EnterInitializationMode(fmi3Instance instance,
                                       fmi3Boolean toleranceDefined,
                                       fmi3Float64 tolerance,
                                       fmi3Float64 startTime,
                                       fmi3Boolean stopTimeDefined,
                                       fmi3Float64 stopTime) {

  

 
    UNUSED(toleranceDefined);
    UNUSED(tolerance);

    BEGIN_FUNCTION(EnterInitializationMode);

    S->startTime = startTime;
    S->stopTime = stopTimeDefined ? stopTime : INFINITY;
    S->time = startTime;
    S->nextCommunicationPoint = startTime;
    S->state = InitializationMode;


    logFormatted(S, LOG_INFO, "FMI CALL",
        "FMU for %s: Entering initialization mode: startTime=%.6f, stopTime=%.6f",
        S->type == ModelExchange ? "model exchange" : "co-simulation",
        S->startTime, S->stopTime);

    if(S->RTLola_Mode){

        CALL(Native_RTLolaMonitor_Init(&S->rtlola_monitor, S));

        if(RTLolaMonitor_ValidateTypes(&S->rtlola_monitor, S)){

            //CALL(RTLolaMonitor_Start(&S->rtlola_monitor));
        }   
     //  CALL(initializeRTLolaMonitor(S)); 
    }
    if (&S->rtlola_monitor && S->rtlola_monitor.spec_switch_requested) {
        // Handle specification switch
        CALL(Native_Handle_Spec_Switch(&S->rtlola_monitor, S, S->state));
    } 
   
    
    END_FUNCTION();
}

fmi3Status fmi3ExitInitializationMode(fmi3Instance instance) {
    BEGIN_FUNCTION(ExitInitializationMode);

    // if values were set and no fmi3GetXXX triggered update before,
    // ensure calculated values are updated now
    if (S->isDirtyValues) {
        CALL(calculateValues(S));
        S->isDirtyValues = false;
    }

    switch (S->type) {
        case ModelExchange:
            S->state = EventMode;
            break;
        case CoSimulation:
            S->state = S->eventModeUsed ? EventMode : StepMode;
            break;
        case ScheduledExecution:
            S->state = ClockActivationMode;
            break;
    }
    if(S->rtlola_monitor.rtlola_handle){
        CALL(Native_RTLolaMonitor_Start(&S->rtlola_monitor, S));
    }
    logFormatted(S, LOG_INFO, "FMI CALL",
        "Exiting initialization mode");
    CALL(configurate(S));

    END_FUNCTION();
}

fmi3Status fmi3EnterEventMode(fmi3Instance instance) {

    BEGIN_FUNCTION(EnterEventMode);

    S->state = EventMode;

    END_FUNCTION();
}

fmi3Status fmi3Terminate(fmi3Instance instance) {

    BEGIN_FUNCTION(Terminate);

    S->state = Terminated;
    logFormatted(S, LOG_INFO, "FMI CALL",
        "fmi3Terminate %s",
        S->type == ModelExchange ? "model exchange" : "co-simulation");
    END_FUNCTION();
}

fmi3Status fmi3Reset(fmi3Instance instance) {

    BEGIN_FUNCTION(Reset);

    CALL(reset(S));

    END_FUNCTION();
}

fmi3Status fmi3GetFloat32(fmi3Instance instance,
                          const fmi3ValueReference valueReferences[],
                          size_t nValueReferences,
                          fmi3Float32 values[],
                          size_t nValues) {
    GET_VARIABLES(Float32);
}

fmi3Status fmi3GetFloat64(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3Float64 values[],
    size_t nValues) {
    GET_VARIABLES(Float64);
}

fmi3Status fmi3GetInt8(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3Int8 values[],
    size_t nValues) {
    GET_VARIABLES(Int8);
}

fmi3Status fmi3GetUInt8(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3UInt8 values[],
    size_t nValues) {
    GET_VARIABLES(UInt8);
}

fmi3Status fmi3GetInt16(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3Int16 values[],
    size_t nValues) {
    GET_VARIABLES(Int16);
}

fmi3Status fmi3GetUInt16(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3UInt16 values[],
    size_t nValues) {
    GET_VARIABLES(UInt16);
}

fmi3Status fmi3GetInt32(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3Int32 values[],
    size_t nValues) {
    GET_VARIABLES(Int32);
}

fmi3Status fmi3GetUInt32(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3UInt32 values[],
    size_t nValues) {
    GET_VARIABLES(UInt32);
}

fmi3Status fmi3GetInt64(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3Int64 values[],
    size_t nValues) {
    GET_VARIABLES(Int64);
}

fmi3Status fmi3GetUInt64(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3UInt64 values[],
    size_t nValues) {
    GET_VARIABLES(UInt64);
}

fmi3Status fmi3GetBoolean(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3Boolean values[],
    size_t nValues) {
    GET_VARIABLES(Boolean);
}

fmi3Status fmi3GetString(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3String values[],
    size_t nValues) {
    GET_VARIABLES(String);
}

fmi3Status fmi3GetBinary(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    size_t valueSizes[],
    fmi3Binary values[],
    size_t nValues) {

    BEGIN_FUNCTION(GetBinary);

    size_t index = 0;

    for (size_t i = 0; i < nValueReferences; i++) {
        CALL(getBinary(S, (ValueReference)valueReferences[i], valueSizes, (const char**)values, nValues, &index));
    }

    END_FUNCTION();
}

fmi3Status fmi3GetClock(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3Clock values[]) {

    BEGIN_FUNCTION(GetClock);

    for (size_t i = 0; i < nValueReferences; i++) {
        CALL(getClock(instance, (ValueReference)valueReferences[i], &values[i]));
    }
    END_FUNCTION();
}

fmi3Status fmi3SetFloat32(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Float32 values[],
    size_t nValues) {
    SET_VARIABLES(Float32);
}

fmi3Status fmi3SetFloat64(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Float64 values[],
    size_t nValues) {
    SET_VARIABLES(Float64);
}

fmi3Status fmi3SetInt8(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Int8 values[],
    size_t nValues) {
    SET_VARIABLES(Int8);
}

fmi3Status fmi3SetUInt8(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3UInt8 values[],
    size_t nValues) {
    SET_VARIABLES(UInt8);
}

fmi3Status fmi3SetInt16(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Int16 values[],
    size_t nValues) {
    SET_VARIABLES(Int16);
}

fmi3Status fmi3SetUInt16(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3UInt16 values[],
    size_t nValues) {
    SET_VARIABLES(UInt16);
}

fmi3Status fmi3SetInt32(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Int32 values[],
    size_t nValues) {
    SET_VARIABLES(Int32);
}

fmi3Status fmi3SetUInt32(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3UInt32 values[],
    size_t nValues) {
    SET_VARIABLES(UInt32);
}

fmi3Status fmi3SetInt64(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Int64 values[],
    size_t nValues) {
    SET_VARIABLES(Int64);
}

fmi3Status fmi3SetUInt64(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3UInt64 values[],
    size_t nValues) {
    SET_VARIABLES(UInt64);
}

fmi3Status fmi3SetBoolean(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Boolean values[],
    size_t nValues) {
    SET_VARIABLES(Boolean);
}

fmi3Status fmi3SetString(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3String values[],
    size_t nValues) {
    SET_VARIABLES(String);
}

fmi3Status fmi3SetBinary(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const size_t valueSizes[],
    const fmi3Binary values[],
    size_t nValues) {

    BEGIN_FUNCTION(SetBinary);

    size_t index = 0;

    for (size_t i = 0; i < nValueReferences; i++) {
        CALL(setBinary(S, (ValueReference)valueReferences[i], valueSizes, (const char* const*)values, nValues, &index));
    }

    END_FUNCTION();
}

fmi3Status fmi3SetClock(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Clock values[]) {

    BEGIN_FUNCTION(SetClock);

    for (size_t i = 0; i < nValueReferences; i++) {
        if (values[i]) {
            CALL(activateClock(instance, (ValueReference)valueReferences[i]));
        }
    }

    END_FUNCTION();
}

fmi3Status fmi3GetNumberOfVariableDependencies(fmi3Instance instance,
                                               fmi3ValueReference valueReference,
                                               size_t* nDependencies) {
    UNUSED(valueReference);
    UNUSED(nDependencies);

    NOT_IMPLEMENTED;
}

fmi3Status fmi3GetVariableDependencies(fmi3Instance instance,
    fmi3ValueReference dependent,
    size_t elementIndicesOfDependent[],
    fmi3ValueReference independents[],
    size_t elementIndicesOfIndependents[],
    fmi3DependencyKind dependencyKinds[],
    size_t nDependencies) {

    UNUSED(dependent);
    UNUSED(elementIndicesOfDependent);
    UNUSED(independents);
    UNUSED(elementIndicesOfIndependents);
    UNUSED(dependencyKinds);
    UNUSED(nDependencies);

    NOT_IMPLEMENTED;
}

fmi3Status fmi3GetFMUState(fmi3Instance instance, fmi3FMUState* FMUState) {

    BEGIN_FUNCTION(GetFMUState);

    CALL(getFMUState(S, FMUState));

    END_FUNCTION();
}

fmi3Status fmi3SetFMUState(fmi3Instance instance, fmi3FMUState FMUState) {

    BEGIN_FUNCTION(SetFMUState);

    if (nullPointer(S, "fmi3SetFMUState", "FMUState", FMUState)) {
        return fmi3Error;
    }

    CALL(setFMUState(S, FMUState));

    END_FUNCTION();
}

fmi3Status fmi3FreeFMUState(fmi3Instance instance, fmi3FMUState* FMUState) {

    BEGIN_FUNCTION(FreeFMUState);

    free(*FMUState);

    *FMUState = NULL;

    END_FUNCTION();
}

fmi3Status fmi3SerializedFMUStateSize(fmi3Instance instance,
    fmi3FMUState  FMUState,
    size_t* size) {

    UNUSED(instance);
    UNUSED(FMUState);

    BEGIN_FUNCTION(SerializedFMUStateSize);

    *size = sizeof(ModelInstance);

    END_FUNCTION();
}

fmi3Status fmi3SerializeFMUState(fmi3Instance instance,
    fmi3FMUState  FMUState,
    fmi3Byte serializedState[],
    size_t size) {

    BEGIN_FUNCTION(SerializeFMUState);

    if (nullPointer(S, "fmi3SerializeFMUState", "FMUstate", FMUState)) {
        return fmi3Error;
    }

    if (invalidNumber(S, "fmi3SerializeFMUState", "size", size, sizeof(ModelInstance))) {
        return fmi3Error;
    }

    memcpy(serializedState, FMUState, sizeof(ModelInstance));

    END_FUNCTION();
}

fmi3Status fmi3DeserializeFMUState(fmi3Instance instance,
    const fmi3Byte serializedState[],
    size_t size,
    fmi3FMUState* FMUState) {

    BEGIN_FUNCTION(DeserializeFMUState);

    if (invalidNumber(S, "fmi3DeserializeFMUState", "size", size, sizeof(ModelInstance))) {
        return fmi3Error;
    }

    if (*FMUState == NULL) {
        *FMUState = calloc(1, sizeof(ModelInstance));
        if (*FMUState == NULL) {
            printf("Failed to allocate memory for FMUState.\n");
            return fmi3Error;
        }
    }

    memcpy(*FMUState, serializedState, sizeof(ModelInstance));

    END_FUNCTION();
}

fmi3Status fmi3GetDirectionalDerivative(fmi3Instance instance,
    const fmi3ValueReference unknowns[],
    size_t nUnknowns,
    const fmi3ValueReference knowns[],
    size_t nKnowns,
    const fmi3Float64 seed[],
    size_t nSeed,
    fmi3Float64 sensitivity[],
    size_t nSensitivity) {

    UNUSED(nSeed);
    UNUSED(nSensitivity);

    BEGIN_FUNCTION(GetDirectionalDerivative);

    // TODO: check value references
    // TODO: assert nUnknowns == nDeltaOfUnknowns
    // TODO: assert nKnowns == nDeltaKnowns

    for (size_t i = 0; i < nUnknowns; i++) {
        sensitivity[i] = 0;
        for (size_t j = 0; j < nKnowns; j++) {
            double partialDerivative = 0;
            CALL(getPartialDerivative(S, (ValueReference)unknowns[i], (ValueReference)knowns[j], &partialDerivative));
            sensitivity[i] += partialDerivative * seed[j];
        }
    }

    END_FUNCTION();
}

fmi3Status fmi3GetAdjointDerivative(fmi3Instance instance,
    const fmi3ValueReference unknowns[],
    size_t nUnknowns,
    const fmi3ValueReference knowns[],
    size_t nKnowns,
    const fmi3Float64 seed[],
    size_t nSeed,
    fmi3Float64 sensitivity[],
    size_t nSensitivity) {

    UNUSED(nSeed);
    UNUSED(nSensitivity);

    BEGIN_FUNCTION(GetAdjointDerivative);

    // TODO: check value references

    for (size_t i = 0; i < nKnowns; i++) {
        sensitivity[i] = 0;
        for (size_t j = 0; j < nUnknowns; j++) {
            double partialDerivative = 0;
            CALL(getPartialDerivative(S, (ValueReference)unknowns[j], (ValueReference)knowns[i], &partialDerivative));
            sensitivity[i] += partialDerivative * seed[j];
        }
    }

    END_FUNCTION();
}

fmi3Status fmi3EnterConfigurationMode(fmi3Instance instance) {

    BEGIN_FUNCTION(EnterConfigurationMode);

    S->state = (S->state == Instantiated) ? ConfigurationMode : ReconfigurationMode;

    END_FUNCTION();
}

fmi3Status fmi3ExitConfigurationMode(fmi3Instance instance) {

    BEGIN_FUNCTION(ExitConfigurationMode);

    if (S->state == ConfigurationMode) {
        S->state = Instantiated;
    } else {
        switch (S->type) {
            case ModelExchange:
                S->state = EventMode;
                break;
            case CoSimulation:
                S->state = StepMode;
                break;
            case ScheduledExecution:
                S->state = ClockActivationMode;
                break;
        }
    }

#if defined(HAS_CONTINUOUS_STATES) || defined(HAS_EVENT_INDICATORS)
    CALL(configurate(S));
#endif

    END_FUNCTION();
}

fmi3Status fmi3GetIntervalDecimal(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3Float64 intervals[],
    fmi3IntervalQualifier qualifiers[]) {

    BEGIN_FUNCTION(GetIntervalDecimal);

    // TODO: Check nValueReferences != nValues ?

    for (size_t i = 0; i < nValueReferences; i++) {
        CALL(getInterval(instance, (ValueReference)valueReferences[i], &intervals[i], (int*)&qualifiers[i]));
    }

    END_FUNCTION();
}

fmi3Status fmi3GetIntervalFraction(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3UInt64 intervalCounters[],
    fmi3UInt64 resolutions[],
    fmi3IntervalQualifier qualifiers[]) {

    UNUSED(valueReferences);
    UNUSED(nValueReferences);
    UNUSED(intervalCounters);
    UNUSED(resolutions);
    UNUSED(qualifiers);

    NOT_IMPLEMENTED;
}

fmi3Status fmi3GetShiftDecimal(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3Float64 shifts[]) {

    UNUSED(valueReferences);
    UNUSED(nValueReferences);
    UNUSED(shifts);

    NOT_IMPLEMENTED;
}

fmi3Status fmi3GetShiftFraction(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    fmi3UInt64 shiftCounters[],
    fmi3UInt64 resolutions[]) {

    UNUSED(valueReferences);
    UNUSED(nValueReferences);
    UNUSED(shiftCounters);
    UNUSED(resolutions);

    NOT_IMPLEMENTED;
}

fmi3Status fmi3SetIntervalDecimal(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Float64 intervals[]) {

    UNUSED(valueReferences);
    UNUSED(nValueReferences);
    UNUSED(intervals);

    NOT_IMPLEMENTED;
}

fmi3Status fmi3SetIntervalFraction(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3UInt64 intervalCounters[],
    const fmi3UInt64 resolutions[]) {

    UNUSED(valueReferences);
    UNUSED(nValueReferences);
    UNUSED(intervalCounters);
    UNUSED(resolutions);

    NOT_IMPLEMENTED;
}

fmi3Status fmi3SetShiftDecimal(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Float64 shifts[]) {

    UNUSED(valueReferences);
    UNUSED(nValueReferences);
    UNUSED(shifts);

    NOT_IMPLEMENTED;
}

fmi3Status fmi3SetShiftFraction(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3UInt64 shiftCounters[],
    const fmi3UInt64 resolutions[]) {

    UNUSED(valueReferences);
    UNUSED(nValueReferences);
    UNUSED(shiftCounters);
    UNUSED(resolutions);

    NOT_IMPLEMENTED;
}

fmi3Status fmi3EvaluateDiscreteStates(fmi3Instance instance) {
    NOT_IMPLEMENTED;
}

fmi3Status fmi3UpdateDiscreteStates(fmi3Instance instance,
    fmi3Boolean* discreteStatesNeedUpdate,
    fmi3Boolean* terminateSimulation,
    fmi3Boolean* nominalsOfContinuousStatesChanged,
    fmi3Boolean* valuesOfContinuousStatesChanged,
    fmi3Boolean* nextEventTimeDefined,
    fmi3Float64* nextEventTime) {

    BEGIN_FUNCTION(NewDiscreteStates);

#ifdef EVENT_UPDATE
    CALL(eventUpdate(S));
#endif

    // copy internal eventInfo of component to output arguments
    if (discreteStatesNeedUpdate)          *discreteStatesNeedUpdate          = S->newDiscreteStatesNeeded;
    if (terminateSimulation)               *terminateSimulation               = S->terminateSimulation;
    if (nominalsOfContinuousStatesChanged) *nominalsOfContinuousStatesChanged = S->nominalsOfContinuousStatesChanged;
    if (valuesOfContinuousStatesChanged)   *valuesOfContinuousStatesChanged   = S->valuesOfContinuousStatesChanged;
    if (nextEventTimeDefined)              *nextEventTimeDefined              = S->nextEventTimeDefined;
    if (nextEventTime)                     *nextEventTime                     = S->nextEventTime;

    END_FUNCTION();
}

/***************************************************
 Functions for Model Exchange
 ****************************************************/

fmi3Status fmi3EnterContinuousTimeMode(fmi3Instance instance) {

    BEGIN_FUNCTION(EnterContinuousTimeMode);

    S->state = ContinuousTimeMode;

    END_FUNCTION();
}

fmi3Status fmi3CompletedIntegratorStep(fmi3Instance instance,
    fmi3Boolean  noSetFMUStatePriorToCurrentPoint,
    fmi3Boolean* enterEventMode,
    fmi3Boolean* terminateSimulation) {

    UNUSED(noSetFMUStatePriorToCurrentPoint);

    BEGIN_FUNCTION(CompletedIntegratorStep);

    ASSERT_NOT_NULL(enterEventMode);
    ASSERT_NOT_NULL(terminateSimulation);

    *enterEventMode = fmi3False;
    *terminateSimulation = fmi3False;

    END_FUNCTION();
}

/* Providing independent variables and re-initialization of caching */
fmi3Status fmi3SetTime(fmi3Instance instance, fmi3Float64 time) {

    BEGIN_FUNCTION(SetTime);

    S->time = time;

    END_FUNCTION();
}

fmi3Status fmi3SetContinuousStates(fmi3Instance instance,
    const fmi3Float64 continuousStates[],
    size_t nContinuousStates) {

    BEGIN_FUNCTION(SetContinuousStates);

#ifdef HAS_CONTINUOUS_STATES
    if (invalidNumber(S, "fmi3SetContinuousStates", "nContinuousStates", nContinuousStates, getNumberOfContinuousStates(S)))
        return fmi3Error;

    ASSERT_NOT_NULL(continuousStates);

    CALL(setContinuousStates(S, continuousStates, nContinuousStates));
#else
    UNUSED(continuousStates);
    UNUSED(nContinuousStates);
    return fmi3Error;
#endif

    END_FUNCTION();
}

/* Evaluation of the model equations */
fmi3Status fmi3GetContinuousStateDerivatives(fmi3Instance instance,
    fmi3Float64 derivatives[],
    size_t nContinuousStates) {

    BEGIN_FUNCTION(GetContinuousStateDerivatives);

#ifdef HAS_CONTINUOUS_STATES
    if (invalidNumber(S, "fmi3GetContinuousStateDerivatives", "nContinuousStates", nContinuousStates, getNumberOfContinuousStates(S)))
        return fmi3Error;

    if (nullPointer(S, "fmi3GetContinuousStateDerivatives", "derivatives[]", derivatives))
        return fmi3Error;

    CALL(getDerivatives(S, derivatives, nContinuousStates));
#else
    UNUSED(derivatives);
    UNUSED(nContinuousStates);
    return fmi3Error;
#endif

    END_FUNCTION();
}

fmi3Status fmi3GetEventIndicators(fmi3Instance instance,
    fmi3Float64 eventIndicators[],
    size_t nEventIndicators) {

    BEGIN_FUNCTION(GetEventIndicators);

#ifdef HAS_EVENT_INDICATORS
    if (invalidNumber(S, "fmi3GetEventIndicators", "nEventIndicators", nEventIndicators, getNumberOfEventIndicators(S))) {
        return fmi3Error;
    }

    CALL(getEventIndicators(S, eventIndicators, nEventIndicators));
#else

    UNUSED(eventIndicators);

    if (nEventIndicators > 0) {
        // TODO: log error
        return fmi3Error;
    }
#endif

    END_FUNCTION();
}

fmi3Status fmi3GetContinuousStates(fmi3Instance instance,
    fmi3Float64 continuousStates[],
    size_t nContinuousStates) {

    BEGIN_FUNCTION(GetContinuousStates);

#ifdef HAS_CONTINUOUS_STATES
    if (invalidNumber(S, "fmi3GetContinuousStates", "nContinuousStates", nContinuousStates, getNumberOfContinuousStates(S)))
        return fmi3Error;

    if (nullPointer(S, "fmi3GetContinuousStates", "continuousStates[]", continuousStates))
        return fmi3Error;

    CALL(getContinuousStates(S, continuousStates, nContinuousStates));
#else
    UNUSED(continuousStates);
    UNUSED(nContinuousStates);
    return fmi3Error;
#endif

    END_FUNCTION();
}

fmi3Status fmi3GetNominalsOfContinuousStates(fmi3Instance instance,
    fmi3Float64 nominals[],
    size_t nContinuousStates) {

    BEGIN_FUNCTION(GetNominalsOfContinuousStates);

#ifdef HAS_CONTINUOUS_STATES
    if (invalidNumber(S, "fmi3GetNominalContinuousStates", "nContinuousStates", nContinuousStates, getNumberOfContinuousStates(instance)))
        return fmi3Error;

    if (nullPointer(S, "fmi3GetNominalContinuousStates", "nominals[]", nominals))
        return fmi3Error;

    for (size_t i = 0; i < nContinuousStates; i++) {
        nominals[i] = 1;
    }

    return fmi3OK;
#else
    UNUSED(nominals);
    UNUSED(nContinuousStates);
    return fmi3Error;
#endif

    END_FUNCTION();
}

fmi3Status fmi3GetNumberOfEventIndicators(fmi3Instance instance,
    size_t* nEventIndicators) {

    BEGIN_FUNCTION(GetNumberOfEventIndicators);

    ASSERT_NOT_NULL(nEventIndicators);

#ifdef HAS_EVENT_INDICATORS
    *nEventIndicators = getNumberOfEventIndicators(instance);
#else
    *nEventIndicators = 0;
#endif


    END_FUNCTION();
}

fmi3Status fmi3GetNumberOfContinuousStates(fmi3Instance instance,
    size_t* nContinuousStates) {

    BEGIN_FUNCTION(GetNumberOfContinuousStates);

    ASSERT_NOT_NULL(nContinuousStates);

#ifdef HAS_CONTINUOUS_STATES
    *nContinuousStates = getNumberOfContinuousStates(instance);
#else
    *nContinuousStates = 0;
#endif

    END_FUNCTION();
}

/***************************************************
 Functions for Co-Simulation
 ****************************************************/

fmi3Status fmi3EnterStepMode(fmi3Instance instance) {

    BEGIN_FUNCTION(EnterStepMode);

    S->state = StepMode;

    END_FUNCTION();
}

fmi3Status fmi3GetOutputDerivatives(fmi3Instance instance,
    const fmi3ValueReference valueReferences[],
    size_t nValueReferences,
    const fmi3Int32 orders[],
    fmi3Float64 values[],
    size_t nValues) {

    UNUSED(nValues);

    BEGIN_FUNCTION(GetOutputDerivatives);

#ifdef GET_OUTPUT_DERIVATIVE
    for (size_t i = 0; i < nValueReferences; i++) {
        CALL(getOutputDerivative(S, (ValueReference)valueReferences[i], orders[i], &values[i]));
    }
#else
    UNUSED(valueReferences);
    UNUSED(nValueReferences);
    UNUSED(orders);
    UNUSED(values);

    NOT_IMPLEMENTED;
#endif

    END_FUNCTION();
}

fmi3Status fmi3DoStep(fmi3Instance instance,
    fmi3Float64 currentCommunicationPoint,
    fmi3Float64 communicationStepSize,
    fmi3Boolean noSetFMUStatePriorToCurrentPoint,
    fmi3Boolean* eventHandlingNeeded,
    fmi3Boolean* terminateSimulation,
    fmi3Boolean* earlyReturn,
    fmi3Float64* lastSuccessfulTime) {

    UNUSED(noSetFMUStatePriorToCurrentPoint);

    BEGIN_FUNCTION(DoStep);

    /*if (!RTLolaMonitor_HandleSpecSwitch(S)) {
        return fmi3Error; // Exit if the spec switch failed
    }*/
    if(Native_Handle_Spec_Switch(&S->rtlola_monitor, S, S->state)){
        CALL(Native_RTLolaMonitor_Start(&S->rtlola_monitor, S));
    }


    if (fabs(currentCommunicationPoint - S->nextCommunicationPoint) > EPSILON) {
        logError(S, "Expected currentCommunicationPoint = %.16g but was %.16g.",
            S->nextCommunicationPoint, currentCommunicationPoint);
        S->state = Terminated;
        return fmi3Error;
    }

    if (communicationStepSize <= 0) {
        logError(S, "Communication step size must be > 0 but was %g.", communicationStepSize);
        S->state = Terminated;
        return fmi3Error;
    }

    if (currentCommunicationPoint + communicationStepSize > S->stopTime + EPSILON) {
        logError(S, "At communication point %.16g a step size of %.16g was requested but stop time is %.16g.",
            currentCommunicationPoint, communicationStepSize, S->stopTime);
        S->state = Terminated;
        return fmi3Error;
    }

    const fmi3Float64 nextCommunicationPoint = currentCommunicationPoint + communicationStepSize + EPSILON;

    bool nextCommunicationPointReached;

    *eventHandlingNeeded = fmi3False;
    *terminateSimulation = fmi3False;
    *earlyReturn         = fmi3False;

    while (true) {

        const fmi3Float64 nextSolverStepTime = S->time + FIXED_SOLVER_STEP;

        nextCommunicationPointReached = nextSolverStepTime > nextCommunicationPoint;

        if (nextCommunicationPointReached || (*eventHandlingNeeded && S->earlyReturnAllowed)) {
            break;
        }

#ifdef EVENT_UPDATE
        if (*eventHandlingNeeded) {
            eventUpdate(S);
            *eventHandlingNeeded = fmi3False;
        }
#endif

        bool stateEvent, timeEvent;

        CALL(doFixedStep(S, &stateEvent, &timeEvent));

#ifdef EVENT_UPDATE
        if (stateEvent || timeEvent) {

            if (S->eventModeUsed) {
                *eventHandlingNeeded = fmi3True;
            } else {
                CALL(eventUpdate(S));
#ifdef HAS_EVENT_INDICATORS
                CALL(getEventIndicators(S, S->prez, S->nz));
#endif
            }

            if (S->earlyReturnAllowed) {
                break;
            }
        }
#endif

        if (S->terminateSimulation) {
            break;
        }
    }

    *terminateSimulation = S->terminateSimulation;
    *earlyReturn         = S->earlyReturnAllowed && !nextCommunicationPointReached;
    *lastSuccessfulTime  = S->time;

    if (nextCommunicationPointReached) {
        S->nextCommunicationPoint = currentCommunicationPoint + communicationStepSize;
    } else {
        S->nextCommunicationPoint = S->time;
    }
    if (&S->rtlola_monitor.is_active) {
        logFormatted(S, 
            LOG_INFO,                   // Severity level
            "FMI CALL",               // Category
            "Do Step: t=%.3f, stepsize=%.3f",     // Format string
            S->time, communicationStepSize           // Arguments matching format
            );
        CALL(Native_RTLolaMonitor_SendData(&S->rtlola_monitor, S));
    }


    
    END_FUNCTION();
}

fmi3Status fmi3ActivateModelPartition(fmi3Instance instance,
    fmi3ValueReference clockReference,
    fmi3Float64 activationTime) {

    BEGIN_FUNCTION(ActivateModelPartition);

    CALL(activateModelPartition(S, (ValueReference)clockReference, activationTime));

    END_FUNCTION();
}
