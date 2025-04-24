#if FMI_VERSION != 2
#error FMI_VERSION must be 2
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
#include "cosimulation.h"
#include "RTLolaMonitor.h"



// C-code FMUs have functions names prefixed with MODEL_IDENTIFIER_.
// Define DISABLE_PREFIX to build a binary FMU.
#ifndef DISABLE_PREFIX
#define pasteA(a,b)     a ## b
#define pasteB(a,b)    pasteA(a,b)
#define FMI2_FUNCTION_PREFIX pasteB(MODEL_IDENTIFIER, _)
#endif
#include "fmi2Functions.h"

#define ASSERT_NOT_NULL(p) \
do { \
    if (!p) { \
        logError(S, "Argument %s must not be NULL.", xstr(p)); \
        S->state = Terminated; \
        return (fmi2Status)Error; \
    } \
} while (false)

#define GET_VARIABLES(T) \
do { \
    if (nvr == 0) goto TERMINATE; \
    ASSERT_NOT_NULL(vr); \
    ASSERT_NOT_NULL(value); \
    size_t index = 0; \
    if (S->isDirtyValues) { \
        CALL(calculateValues(S)); \
        S->isDirtyValues = false; \
    } \
    for (size_t i = 0; i < nvr; i++) { \
        CALL(get ## T(S, vr[i], value, nvr, &index)); \
    } \
} while (false)

#define SET_VARIABLES(T) \
do { \
    if (nvr == 0) goto TERMINATE; \
    ASSERT_NOT_NULL(vr); \
    ASSERT_NOT_NULL(value); \
    size_t index = 0; \
    for (size_t i = 0; i < nvr; i++) { \
        CALL(set ## T(S, vr[i], value, nvr, &index)); \
    } \
    if (nvr > 0) S->isDirtyValues = true; \
} while (false)

#define GET_BOOLEAN_VARIABLES \
do { \
    for (size_t i = 0; i < nvr; i++) { \
        bool v; \
        size_t index = 0; \
        CALL(getBoolean(S, vr[i], &v, nvr, &index)); \
        value[i] = v; \
    } \
} while (false)

#define SET_BOOLEAN_VARIABLES \
do { \
    for (size_t i = 0; i < nvr; i++) { \
        const bool v = value[i]; \
        size_t index = 0; \
        CALL(setBoolean(S, vr[i], &v, nvr, &index)); \
    } \
} while (false)

#ifndef DT_EVENT_DETECT
#define DT_EVENT_DETECT 1e-10
#endif

// ---------------------------------------------------------------------------
// Function calls allowed state masks for both Model-exchange and Co-simulation
// ---------------------------------------------------------------------------
#define MASK_fmi2GetTypesPlatform        (StartAndEnd | Instantiated | InitializationMode \
| EventMode | ContinuousTimeMode \
| StepComplete | StepInProgress | StepFailed | StepCanceled \
| Terminated)
#define MASK_fmi2GetVersion              MASK_fmi2GetTypesPlatform
#define MASK_fmi2SetDebugLogging         (Instantiated | InitializationMode \
| EventMode | ContinuousTimeMode \
| StepComplete | StepInProgress | StepFailed | StepCanceled \
| Terminated)
#define MASK_fmi2Instantiate             (StartAndEnd)
#define MASK_fmi2FreeInstance            (Instantiated | InitializationMode \
| EventMode | ContinuousTimeMode \
| StepComplete | StepFailed | StepCanceled \
| Terminated)
#define MASK_fmi2SetupExperiment         Instantiated
#define MASK_fmi2EnterInitializationMode Instantiated
#define MASK_fmi2ExitInitializationMode  InitializationMode
#define MASK_fmi2Terminate               (EventMode | ContinuousTimeMode \
| StepComplete | StepFailed)
#define MASK_fmi2Reset                   MASK_fmi2FreeInstance
#define MASK_fmi2GetReal                 (InitializationMode \
| EventMode | ContinuousTimeMode \
| StepComplete | StepFailed | StepCanceled \
| Terminated)
#define MASK_fmi2GetInteger              MASK_fmi2GetReal
#define MASK_fmi2GetBoolean              MASK_fmi2GetReal
#define MASK_fmi2initializeRTLola        MASK_fmi2GetReal
#define MASK_fmi2GetString               MASK_fmi2GetReal
#define MASK_fmi2SetReal                 (Instantiated | InitializationMode \
| EventMode | ContinuousTimeMode \
| StepComplete)
#define MASK_fmi2SetInteger              (Instantiated | InitializationMode \
| EventMode \
| StepComplete)
#define MASK_fmi2SetBoolean              MASK_fmi2SetInteger
#define MASK_fmi2SetString               MASK_fmi2SetInteger
#define MASK_fmi2GetFMUstate             MASK_fmi2FreeInstance
#define MASK_fmi2SetFMUstate             MASK_fmi2FreeInstance
#define MASK_fmi2FreeFMUstate            MASK_fmi2FreeInstance
#define MASK_fmi2SerializedFMUstateSize  MASK_fmi2FreeInstance
#define MASK_fmi2SerializeFMUstate       MASK_fmi2FreeInstance
#define MASK_fmi2DeSerializeFMUstate     MASK_fmi2FreeInstance
#define MASK_fmi2GetDirectionalDerivative (InitializationMode \
| EventMode | ContinuousTimeMode \
| StepComplete | StepFailed | StepCanceled \
| Terminated)

// ---------------------------------------------------------------------------
// Function calls allowed state masks for Model-exchange
// ---------------------------------------------------------------------------
#define MASK_fmi2EnterEventMode          (EventMode | ContinuousTimeMode)
#define MASK_fmi2NewDiscreteStates       EventMode
#define MASK_fmi2EnterContinuousTimeMode EventMode
#define MASK_fmi2CompletedIntegratorStep ContinuousTimeMode
#define MASK_fmi2SetTime                 (EventMode | ContinuousTimeMode)
#define MASK_fmi2SetContinuousStates     ContinuousTimeMode
#define MASK_fmi2GetEventIndicators      (InitializationMode \
| EventMode | ContinuousTimeMode \
| Terminated)
#define MASK_fmi2GetContinuousStates     MASK_fmi2GetEventIndicators
#define MASK_fmi2GetDerivatives          (EventMode | ContinuousTimeMode \
| Terminated)
#define MASK_fmi2GetNominalsOfContinuousStates ( Instantiated \
| EventMode | ContinuousTimeMode \
| Terminated)

// ---------------------------------------------------------------------------
// Function calls allowed state masks for Co-simulation
// ---------------------------------------------------------------------------
#define MASK_fmi2SetRealInputDerivatives (Instantiated | InitializationMode \
| StepComplete)
#define MASK_fmi2GetRealOutputDerivatives (StepComplete | StepFailed | StepCanceled \
| Terminated | Error)
#define MASK_fmi2DoStep                  StepComplete
#define MASK_fmi2CancelStep              StepInProgress
#define MASK_fmi2GetStatus               (StepComplete | StepInProgress | StepFailed \
| Terminated)
#define MASK_fmi2GetRealStatus           MASK_fmi2GetStatus
#define MASK_fmi2GetIntegerStatus        MASK_fmi2GetStatus
#define MASK_fmi2GetBooleanStatus        MASK_fmi2GetStatus
#define MASK_fmi2GetStringStatus         MASK_fmi2GetStatus

#define BEGIN_FUNCTION(F) \
Status status = OK; \
if (!c) return fmi2Error; \
ModelInstance *S = (ModelInstance *)c; \
if (!allowedState(S, MASK_fmi2##F, #F)) CALL(Error);

#define END_FUNCTION() \
TERMINATE: \
    return (fmi2Status)status;

#define CALL(f) do { \
    const Status _s = f; \
    if (_s > status) { \
        status = _s; \
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

static bool allowedState(ModelInstance *instance, int statesExpected, char *name) {

    if (!instance) {
        return false;
    }

    if (!(instance->state & statesExpected)) {
        logError(instance, "fmi2%s: Illegal call sequence.", name);
        return false;
    }

    return true;

}




const char* getVariableName(ValueReference vr) {
    switch (vr) {
        case vr_time: return "time";
        case vr_h:    return "h";
        case vr_v:    return "v";
        case vr_g:    return "g";
        case vr_e:    return "e";
        case vr_v_min: return "v_min";
        default:      return "unknown";
        }
}



const char* getHeaderVariableName(const unsigned int vr) {
    switch (vr) {
        case vr_time: return "time";
        case vr_h:    
        return "h";
        case vr_v:    return "v";
        case vr_g:    return "g";
        case vr_e:    return "e";
        case vr_v_min: return "v_min";
        default:     return "unknown";
    }
}

    
double get_variable(unsigned int vr, ModelInstance* instance) {
    if (!instance) {
        printf("Error: instance is null\n");
        return 0.0;
    }
    double value = 0.0;
    size_t index = 0;
    double valueList []= {0.0};
    getFloat64(instance, vr, valueList, 1, &index);
   
    value = valueList[0];
    //double value = 0.0;
    //fmi2Component c = (fmi2Component)instance;
    //Status status = fmi2GetReal(c, &vr, 1, valueList);
    //if (status > Error) {
      //  value = valueList[0];
        //return value;
   // }
    return value; // Return a default value if the call fails
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
/*
#############################################
# RTLola Monitor Functions linear solution  #
#############################################
*/
void RTLolaMonitor_Init(RTLolaMonitor* monitor, const char* spec_path, const unsigned int* vrs, size_t num_vars) {
    if (!monitor) {
        fprintf(stderr, "RTLolaMonitor_Init: Monitor is NULL.\n");
        return;
    }

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

    if (pid == 0) { // Child process
        setvbuf(stdout, NULL, _IOLBF, 0); // Line buffering
        close(monitor->input_pipe[1]); // Close unused write end
        close(monitor->output_pipe[0]); // Close unused read end

        dup2(monitor->input_pipe[0], STDIN_FILENO);
        dup2(monitor->output_pipe[1], STDOUT_FILENO);

        execlp("rtlola-cli", "rtlola-cli", "monitor", "--stdin", "--stdout", "--online", monitor->spec_path, NULL);
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
        printf("Parent: Input pipe: read=%d, write=%d\n", monitor->input_pipe[0], monitor->input_pipe[1]);
        printf("Parent: Output pipe: read=%d, write=%d\n", monitor->output_pipe[0], monitor->output_pipe[1]);

    // Debugging: Print the CSV header being sent
        printf("Parent: Sending CSV header:\n");

        // Write CSV header to the process
        fprintf(monitor->input_stream, "time");
        for (size_t i = 0; i < monitor->num_vars; i++) {
            fprintf(monitor->input_stream, ",%s", getHeaderVariableName(monitor->monitored_vrs[i]));
        }
        fprintf(monitor->input_stream, "\n");
        fflush(monitor->input_stream);

        printf("RTLola process started with PID %d.\n", pid);
        return 1;
    }
}


bool is_process_running(pid_t pid) {
    int status;
    pid_t result = waitpid(pid, &status, WNOHANG);
    if (result == 0) {
        return true; // Process is still running
    }
    return false; // Process has terminated
}

bool rtlola_readoutput(RTLolaMonitor* monitor) {
    if (!monitor || !monitor->is_active || !monitor->output_stream) {
        fprintf(stderr, "rtlola_readoutput: Monitor is not active or invalid.\n");
        return false;
    }

    // Debugging: Print the current state of the monitor
    printf("rtlola_readoutput: Reading output from RTLola process (PID %d)\n", monitor->child_pid);

    // Check if the RTLola process is still running
    if (!is_process_running(monitor->child_pid)) {
        fprintf(stderr, "RTLola process has terminated.\n");
        monitor->is_active = false;
        return false;
    }

    char buffer[1024];
    while (true) {
        if (fgets(buffer, sizeof(buffer), monitor->output_stream) != NULL) {
            // Debugging: Print the output received from RTLola
            printf("RTLola Output: %s", buffer);
        } else {
            if (feof(monitor->output_stream)) {
                // Debugging: Print if the pipe is closed
                fprintf(stderr, "RTLola process closed the output pipe.\n");
                monitor->is_active = false;
                return false;
            } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Debugging: Print if no data is available
                printf("No data available (non-blocking)\n");
                clearerr(monitor->output_stream); // Clear the error flag
                break;
            } else {
                // Debugging: Print the error message
                fprintf(stderr, "Error reading from RTLola output stream: %s\n", strerror(errno));
                monitor->is_active = false;
                RTLolaMonitor_Cleanup(monitor); // Clean up resources
                return false;
            }
        }
    }

    return true;
}

bool RTLolaMonitor_SendData_v2(RTLolaMonitor* monitor, double time, fmi2Component instance) {
    ModelInstance* tempInstance = (ModelInstance*)instance;
    if (!monitor || !monitor->is_active || !monitor->input_stream) {
        fprintf(stderr, "RTLolaMonitor_SendData: Monitor is not active or invalid.\n");
        return false;
    }

    // Debugging: Print the data being sent
    printf("RTLolaMonitor_SendData: Sending data for time=%.6f\n", time);

    // Build and send the CSV line
    fprintf(monitor->input_stream, "%.6f", time);
    for (size_t i = 0; i < monitor->num_vars; i++) {
        double value = get_variable(monitor->monitored_vrs[i], tempInstance);
        fprintf(monitor->input_stream, ",%.6f", value);
        // Debugging: Print each variable value
        printf("RTLolaMonitor_SendData: Sending variable %s=%.6f\n", getHeaderVariableName(monitor->monitored_vrs[i]), value);
    }
    fprintf(monitor->input_stream, "\n");
    fflush(monitor->input_stream);
    usleep(10000);
    // Debugging: Print confirmation that data was sent
    printf("RTLolaMonitor_SendData: Data sent successfully.\n");

    // Read the output immediately after sending data
    if (!rtlola_readoutput(monitor)) {
        fprintf(stderr, "Failed to read RTLola output.\n");
        return false;
    }

    return true;
}


bool adjustFMUVariables(RTLolaMonitor *monitor, const unsigned int *vrs, fmi2Component instance) {
    ModelInstance* tempInstance = (ModelInstance*)instance;
    if (!monitor || !monitor->is_active || !monitor->input_stream) {
        fprintf(stderr, "AdjustingFMUVariables: Monitor is not active or invalid.\n");
        return false;
    }
    UNUSED(vrs);
    for (size_t i = 0; i < monitor->num_vars; i++) {
        double value = set_variable(monitor->monitored_vrs[i], 1.0, tempInstance); // Example value 1.0
        printf("Setting %s: variable to %f\n", getHeaderVariableName(monitor->monitored_vrs[i]), value);
    }
    return true;
}

void RTLolaMonitor_Cleanup(RTLolaMonitor* monitor) {
    if (!monitor) return;

    printf("Cleaning up RTLola monitor.\n");

    // Close input and output streams
    if (monitor->input_pipe[1] != -1 && monitor->input_stream != NULL) {
        fclose(monitor->input_stream);
    }
    if (monitor->output_pipe[0] != -1 && monitor->output_stream != NULL) {
        fclose(monitor->output_stream);
    }

    // Terminate the child process
    if (monitor->child_pid != -1) {
        kill(monitor->child_pid, SIGTERM);
        waitpid(monitor->child_pid, NULL, 0);
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

    // Reset the monitor state
    monitor->is_active = false;
    printf("RTLola monitor cleaned up.\n");
}


/* 
######################################################
# RTLola Monitor Functions parallel windows solution #
######################################################






void RTLolaMonitor_Init(RTLolaMonitor* monitor, 
const char* spec_path,
const unsigned int* vrs,
size_t num_vars)
{
    // Copy specification path
    #ifdef _WIN32
    monitor->spec_path = _strdup(spec_path);
    #else   
    monitor->spec_path = strdup(spec_path);
    #endif
    if (!monitor->spec_path) {
        fprintf(stderr, "Failed to allocate memory for spec_path.\n");
        return;
    }
    printf("rtlola spec path is %s\n", monitor->spec_path);
    
    // Copy monitored variables
    monitor->num_vars = num_vars;
    monitor->monitored_vrs = malloc(num_vars * sizeof(unsigned int));
    if (!monitor->monitored_vrs) {
        fprintf(stderr, "Failed to allocate memory for monitored_vrs.\n");
        free((void*)monitor->spec_path); // Clean up spec_path if allocation fails
        return;
    }
    memcpy(monitor->monitored_vrs, vrs, num_vars * sizeof(unsigned int));
    
    // Initialize pipe handles and runtime state
    monitor->hChildStd_IN_Rd = NULL;
    monitor->hChildStd_IN_Wr = NULL;
    monitor->hChildStd_OUT_Rd = NULL;
    monitor->hChildStd_OUT_Wr = NULL;
    monitor->hProcess = NULL;
    monitor->hThread = NULL;
    monitor->is_active = false;
    
    // Create a mutex for synchronization
    monitor->hMutex = CreateMutex(NULL, FALSE, NULL);
    if (!monitor->hMutex) {
        fprintf(stderr, "Failed to create mutex. Error: %lu\n", GetLastError());
        return;
    }
}

// Thread function to read rtlola-cli output
// Thread function to read rtlola-cli output

DWORD WINAPI read_rtlola_output(LPVOID arg) {
    RTLolaMonitor* monitor = (RTLolaMonitor*)arg;
    if (!monitor || !monitor->hChildStd_OUT_Rd) {
        fprintf(stderr, "Invalid monitor or output pipe in thread function.\n");
        return 1; // Return non-zero to indicate failure
    }

    char buffer[128];
    DWORD bytesRead;
    while (ReadFile(monitor->hChildStd_OUT_Rd, buffer, sizeof(buffer) - 1, &bytesRead, NULL) && bytesRead > 0) {
        // Discard the output (do not print it)
        buffer[bytesRead] = '\0'; // Null-terminate the buffer
        printf("RTLola Output: %s\n", buffer);
        fflush(stdout); // Ensure the output is flushed
        
    }

    return 0; // Return zero to indicate success
}

bool RTLolaMonitor_Start(RTLolaMonitor* monitor) {
    if (!monitor) {
        fprintf(stderr, "Monitor is NULL.\n");
        return false;
    }

    if (monitor->is_active) {
        fprintf(stderr, "Monitor is already active.\n");
        return true;
    }

    // Create pipes for stdin and stdout
    SECURITY_ATTRIBUTES saAttr;
    saAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
    saAttr.bInheritHandle = TRUE; // Ensure the pipe handles are inherited by the child process
    saAttr.lpSecurityDescriptor = NULL;

    // Create a pipe for the child process's STDOUT
    if (!CreatePipe(&monitor->hChildStd_OUT_Rd, &monitor->hChildStd_OUT_Wr, &saAttr, 0)) {
        fprintf(stderr, "Failed to create STDOUT pipe. Error: %lu\n", GetLastError());
        return false;
    }

    // Create a pipe for the child process's STDIN
    if (!CreatePipe(&monitor->hChildStd_IN_Rd, &monitor->hChildStd_IN_Wr, &saAttr, 0)) {
        fprintf(stderr, "Failed to create STDIN pipe. Error: %lu\n", GetLastError());
        CloseHandle(monitor->hChildStd_OUT_Rd);
        CloseHandle(monitor->hChildStd_OUT_Wr);
        return false;
    }

    // Set up the process startup info
    STARTUPINFO siStartInfo;
    ZeroMemory(&siStartInfo, sizeof(STARTUPINFO));
    siStartInfo.cb = sizeof(STARTUPINFO);
    siStartInfo.hStdError = monitor->hChildStd_OUT_Wr; // Redirect stderr to stdout
    siStartInfo.hStdOutput = monitor->hChildStd_OUT_Wr; // Redirect stdout
    siStartInfo.hStdInput = monitor->hChildStd_IN_Rd;   // Redirect stdin
    siStartInfo.dwFlags |= STARTF_USESTDHANDLES;

    PROCESS_INFORMATION piProcInfo;
    ZeroMemory(&piProcInfo, sizeof(PROCESS_INFORMATION));

    // Create the process
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "rtlola-cli monitor --stdin --stdout --online %s", monitor->spec_path);
    if (!CreateProcess(NULL, cmd, NULL, NULL, TRUE, 0, NULL, NULL, &siStartInfo, &piProcInfo)) {
        fprintf(stderr, "Failed to create process. Error: %lu\n", GetLastError());
        CloseHandle(monitor->hChildStd_OUT_Rd);
        CloseHandle(monitor->hChildStd_OUT_Wr);
        CloseHandle(monitor->hChildStd_IN_Rd);
        CloseHandle(monitor->hChildStd_IN_Wr);
        return false;
    }

    // Close unused pipe ends
    CloseHandle(monitor->hChildStd_OUT_Wr); // We don't need the write end of the stdout pipe
    CloseHandle(monitor->hChildStd_IN_Rd);  // We don't need the read end of the stdin pipe

    // Set the process as active
    monitor->is_active = true;
    monitor->hProcess = piProcInfo.hProcess;
    monitor->hThread = piProcInfo.hThread;

    // Build and send the CSV header
    char header[512];
    int header_len = snprintf(header, sizeof(header), "time");
    for (size_t i = 0; i < monitor->num_vars; i++) {
        header_len += snprintf(header + header_len, sizeof(header) - header_len, ",%s", getHeaderVariableName(monitor->monitored_vrs[i]));
    }
    header_len += snprintf(header + header_len, sizeof(header) - header_len, "\n");

    DWORD bytesWritten;
    if (!WriteFile(monitor->hChildStd_IN_Wr, header, header_len, &bytesWritten, NULL)) {
        fprintf(stderr, "Failed to write CSV header to rtlola-cli process. Error: %lu\n", GetLastError());
        return false;
    }

    // Start a thread to read the output
    HANDLE output_thread = CreateThread(
        NULL,                   // Default security attributes
        0,                      // Default stack size
        read_rtlola_output,     // Thread function
        monitor,                // Pass the monitor as an argument
        0,                      // Default creation flags
        NULL                    // Don't need the thread ID
    );

    if (output_thread == NULL) {
        fprintf(stderr, "Failed to create output thread. Error: %lu\n", GetLastError());
        return false;
    }

    // Optionally, close the thread handle if you don't need it
    CloseHandle(output_thread);

    return true;
}


bool RTLolaMonitor_SendData(RTLolaMonitor* monitor, double time, fmi2Component instance) {
    ModelInstance* tempInstance = (ModelInstance*)instance;
    if (!monitor || !monitor->is_active || !monitor->hChildStd_IN_Wr) {
        fprintf(stderr, "Monitor is not active or invalid.\n");
        return false;
    }
    
    // Build the data string
    char buffer[512];
    int len = snprintf(buffer, sizeof(buffer), "%.6f", time);
    for (size_t i = 0; i < monitor->num_vars; i++) {
        double value = get_variable(monitor->monitored_vrs[i], tempInstance);
        len += snprintf(buffer + len, sizeof(buffer) - len, ",%.6f", value);
    }
    len += snprintf(buffer + len, sizeof(buffer) - len, "\n");
    
    // Lock the mutex before writing to the pipe
    WaitForSingleObject(monitor->hMutex, INFINITE);
    
    // Write the data to the rtlola-cli process
    DWORD bytesWritten;
    if (!WriteFile(monitor->hChildStd_IN_Wr, buffer, len, &bytesWritten, NULL)) {
        fprintf(stderr, "Failed to write to rtlola-cli process. Error: %lu\n", GetLastError());
        ReleaseMutex(monitor->hMutex); // Unlock the mutex before returning
        return false;
    }
    
    // Unlock the mutex
    ReleaseMutex(monitor->hMutex);
    
    return true;
}

void RTLolaMonitor_Cleanup(RTLolaMonitor* monitor) {
    if (!monitor) return;

    // Close input and output streams
    if (monitor->hChildStd_OUT_Rd) {
        CloseHandle(monitor->hChildStd_OUT_Rd);
        monitor->hChildStd_OUT_Rd = NULL;
    }
    if (monitor->hChildStd_IN_Wr) {
        CloseHandle(monitor->hChildStd_IN_Wr);
        monitor->hChildStd_IN_Wr = NULL;
    }

    // Terminate the RTLola process
    if (monitor->hProcess) {
        TerminateProcess(monitor->hProcess, 0); // Forcefully terminate the process
        CloseHandle(monitor->hProcess);
        monitor->hProcess = NULL;
    }
    if (monitor->hThread) {
        CloseHandle(monitor->hThread);
        monitor->hThread = NULL;
    }

    // Clean up the mutex
    if (monitor->hMutex) {
        CloseHandle(monitor->hMutex);
        monitor->hMutex = NULL;
    }

    // Free allocated memory
    if (monitor->spec_path) {
        free((void*)monitor->spec_path);
        monitor->spec_path = NULL;
    }
    if (monitor->monitored_vrs) {
        free(monitor->monitored_vrs);
        monitor->monitored_vrs = NULL;
    }

    // Reset the monitor state
    monitor->is_active = false;
}
*/
/*
############################################
# RTLola Monitor Functions parallel linux  #
############################################




// Initialize the monitor by copying the spec path, monitored variables,
// and setting initial states for pipes and streams. 
void RTLolaMonitor_Init(RTLolaMonitor* monitor, const char* spec_path, const unsigned int* vrs, size_t num_vars) {
    if (!monitor) {
        fprintf(stderr, "RTLolaMonitor_Init: Monitor is NULL.\n");
        return;
    }
    
    monitor->spec_path = strdup(spec_path);
    if (!monitor->spec_path) {
        fprintf(stderr, "RTLolaMonitor_Init: Failed to allocate memory for spec_path.\n");
        return;
    }
    
    monitor->input_stream = NULL;
    monitor->output_stream = NULL; 
    printf("RTLola spec path is %s\n", monitor->spec_path);
    
    monitor->num_vars = num_vars;
    if (num_vars > 0) {
        monitor->monitored_vrs = malloc(num_vars * sizeof(unsigned int));
        if (!monitor->monitored_vrs) {
            fprintf(stderr, "RTLolaMonitor_Init: Failed to allocate memory for monitored_vrs.\n");
            //free(monitor->spec_path);
            //monitor->spec_path = NULL;
            return;
        }
        memcpy(monitor->monitored_vrs, vrs, num_vars * sizeof(unsigned int));
    } else {
        monitor->monitored_vrs = NULL;
    }
    
    monitor->input_pipe[0] = monitor->input_pipe[1] = -1;
    monitor->output_pipe[0] = monitor->output_pipe[1] = -1;
    monitor->child_pid = -1;
    monitor->input_stream = NULL;
    monitor->output_stream = NULL;
    monitor->is_active = false;
}

void* read_rtlola_output(void* arg) {
    RTLolaMonitor* monitor = (RTLolaMonitor*)arg;
    if (!monitor) {
        fprintf(stderr, "Output thread: Monitor is NULL.\n");
        return NULL;
    }
    
    monitor->output_stream = fdopen(monitor->output_pipe[0], "r");
    if (!monitor->output_stream) {
        fprintf(stderr, "Output thread: Failed to open output stream.\n");
        return NULL;
    }
    
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), monitor->output_stream) != NULL) {
        printf("RTLola Output: %s", buffer);
        fflush(stdout);
    }
    
    printf("RTLola output thread finished.\n");
    return NULL;
}

int RTLolaMonitor_Start(RTLolaMonitor* monitor) {
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
    
    if (pid == 0) { // Child process
    dup2(monitor->input_pipe[0], STDIN_FILENO);
    dup2(monitor->output_pipe[1], STDOUT_FILENO);
    
    close(monitor->input_pipe[1]);
    close(monitor->output_pipe[0]);
    
    setvbuf(stdout, NULL, _IOLBF, 0);
    
    execlp("rtlola-cli", "rtlola-cli", "monitor", "--stdin", "--stdout", "--online", monitor->spec_path, NULL);
    
    fprintf(stderr, "Child: Failed to execute rtlola-cli.\n");
    exit(1);
} else { // Parent process
close(monitor->input_pipe[0]);
close(monitor->output_pipe[1]);

monitor->child_pid = pid;
monitor->is_active = true;

monitor->input_stream = fdopen(monitor->input_pipe[1], "w");
if (!monitor->input_stream) {
    fprintf(stderr, "RTLolaMonitor_Start: Failed to open input stream.\n");
    return 0;
}

fprintf(monitor->input_stream, "time");
for (size_t i = 0; i < monitor->num_vars; i++) {
    fprintf(monitor->input_stream, ",%s", getHeaderVariableName(monitor->monitored_vrs[i]));
}
fprintf(monitor->input_stream, "\n");
fflush(monitor->input_stream);

if (pthread_create(&monitor->reader_thread, NULL, read_rtlola_output, (void *)monitor) != 0) {
    fprintf(stderr, "RTLolaMonitor_Start: Failed to create output thread.\n");
    return 0;
}
pthread_detach(monitor->reader_thread);

printf("RTLola process started with PID %d.\n", pid);
return 1;
}
}

int RTLolaMonitor_SendData(RTLolaMonitor* monitor, double time, fmi2Component instance) {
    ModelInstance* tempInstance = (ModelInstance*)instance;
    if (!monitor || !monitor->is_active || !monitor->input_stream) {
        fprintf(stderr, "RTLolaMonitor_SendData: Monitor is not active or invalid.\n");
        return 0;
    }
    
    fprintf(monitor->input_stream, "%.6f", time);
    for (size_t i = 0; i < monitor->num_vars; i++) {
        double value = get_variable(monitor->monitored_vrs[i], tempInstance);
        fprintf(monitor->input_stream, ",%.6f", value);
    }
    fprintf(monitor->input_stream, "\n");
    fflush(monitor->input_stream);
    
    printf("Data sent to RTLola.\n");
    return 1;
}

void RTLolaMonitor_Cleanup(RTLolaMonitor* monitor) {
    if (!monitor)
    return;
    
    printf("Cleaning up RTLola monitor.\n");
    
    if (monitor->input_pipe[1] != -1 && monitor->input_stream != NULL) {
        fclose(monitor->input_stream);
    }
    if (monitor->output_pipe[0] != -1 && monitor->output_stream != NULL) {
        fclose(monitor->output_stream);
    }
    
    if (monitor->child_pid != -1) {
        kill(monitor->child_pid, SIGTERM);
        waitpid(monitor->child_pid, NULL, 0);
    }
    
    if (monitor->spec_path) {
        free((char*)monitor->spec_path);
        monitor->spec_path = NULL;
    }
    if (monitor->monitored_vrs) {
        free(monitor->monitored_vrs);
        monitor->monitored_vrs = NULL;
    }
    
    monitor->is_active = false;
    printf("RTLola monitor cleaned up.\n");
}
*/
// ---------------------------------------------------------------------------
// FMI functions
// ---------------------------------------------------------------------------

fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
fmi2String fmuResourceLocation, const fmi2CallbackFunctions *functions,
fmi2Boolean visible, fmi2Boolean loggingOn) {
    
UNUSED(visible);

if (!functions || !functions->logger) {
        return NULL;
    }
    fmi2Component fmiC = 
        createModelInstance(
        (loggerType)functions->logger,
        NULL,
        functions->componentEnvironment,
        instanceName,
        fmuGUID,
        fmuResourceLocation,
        loggingOn,
        (InterfaceType)fmuType);
   // fmi2initializeRTLola(fmiC);
    return fmiC;
}

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean toleranceDefined, fmi2Real tolerance,
                            fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime) {

  
    printf("ENTERING Set up Expiriment MODE fmi2FUNCTION\n");

   
    UNUSED(toleranceDefined);
    UNUSED(tolerance);

    BEGIN_FUNCTION(SetupExperiment)

    S->startTime = startTime;
    S->stopTime = stopTimeDefined ? stopTime : INFINITY;
    S->time = startTime;
    S->nextCommunicationPoint = startTime;



    END_FUNCTION();
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) {

    BEGIN_FUNCTION(EnterInitializationMode)
  
    printf("ENTERING INITIALIZATION MODE fmi2FUNCTION\n");
    //fmi2GetRTLolaStatus();
    S->state = InitializationMode;
    
    if(S->RTLola_Mode){
        CALL(RTLolaMonitor_Start(&S->rtlola_monitor));
     //  CALL(initializeRTLolaMonitor(S)); 
    }

    END_FUNCTION();
}

fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    printf("EXITING INITIALIZATION MODE fmi2FUNCTION\n");
  
    BEGIN_FUNCTION(ExitInitializationMode);
    //fmi2InitializeRTLola(c);




    // if values were set and no fmi2GetXXX triggered update before,
    // ensure calculated values are updated now
    if (S->isDirtyValues) {
        CALL(calculateValues(S));
        S->isDirtyValues = false;
    }

    S->state = S->type == ModelExchange ? EventMode : StepComplete;
    //fmi2initializeRTLola(c);
    CALL(configurate(S));

    END_FUNCTION();
}

char* modelStateToString(ModelState state) {
    switch (state) {
        case StartAndEnd:        return "StartAndEnd";
        case Instantiated:       return "Instantiated";
        case InitializationMode: return "InitializationMode";
        case EventMode:          return "EventMode";
        case ContinuousTimeMode:return "ContinuousTimeMode";
        case StepComplete:      return "StepComplete";
        case StepInProgress:    return "StepInProgress";
        case StepFailed:        return "StepFailed";
        case StepCanceled:     return "StepCanceled";
        case Terminated:        return "Terminated";
        default:                return "UnknownState";
    }
}

fmi2Status fmi2Terminate(fmi2Component c) {

    BEGIN_FUNCTION(Terminate)



    
    
    S->state = Terminated;



    END_FUNCTION();
}

fmi2Status fmi2Reset(fmi2Component c) {

    BEGIN_FUNCTION(Reset);

    CALL(reset(S));

    END_FUNCTION();
}

void fmi2FreeInstance(fmi2Component c) {
    //Free RTLola monitor
    RTLolaMonitor_Cleanup(&((ModelInstance*)c)->rtlola_monitor);
    freeModelInstance((ModelInstance*)c);
}

// ---------------------------------------------------------------------------
// FMI functions: class methods not depending of a specific model instance
// ---------------------------------------------------------------------------

const char* fmi2GetVersion(void) {
    return fmi2Version;
}

const char* fmi2GetTypesPlatform(void) {
    return fmi2TypesPlatform;
}

// ---------------------------------------------------------------------------
// FMI functions: logging control, setters and getters for Real, Integer,
// Boolean, String
// ---------------------------------------------------------------------------



fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories, const fmi2String categories[]) {

    BEGIN_FUNCTION(SetDebugLogging)

    CALL(setDebugLogging(S, loggingOn, nCategories, categories));

    END_FUNCTION();
}

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]) {

    BEGIN_FUNCTION(GetReal)

    if (nvr > 0 && nullPointer(S, "fmi2GetReal", "vr[]", vr))
        return fmi2Error;

    if (nvr > 0 && nullPointer(S, "fmi2GetReal", "value[]", value))
        return fmi2Error;

    if (nvr > 0 && S->isDirtyValues) {
        calculateValues(S);
        S->isDirtyValues = false;
    }

    GET_VARIABLES(Float64);

    END_FUNCTION();
}

fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]) {

    BEGIN_FUNCTION(GetInteger)

    if (nvr > 0 && nullPointer(S, "fmi2GetInteger", "vr[]", vr))
            return fmi2Error;

    if (nvr > 0 && nullPointer(S, "fmi2GetInteger", "value[]", value))
            return fmi2Error;

    if (nvr > 0 && S->isDirtyValues) {
        calculateValues(S);
        S->isDirtyValues = false;
    }

    GET_VARIABLES(Int32);

    END_FUNCTION();
}

fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]) {

    BEGIN_FUNCTION(GetBoolean)

    if (nvr > 0 && nullPointer(S, "fmi2GetBoolean", "vr[]", vr))
            return fmi2Error;

    if (nvr > 0 && nullPointer(S, "fmi2GetBoolean", "value[]", value))
            return fmi2Error;

    if (nvr > 0 && S->isDirtyValues) {
        calculateValues(S);
        S->isDirtyValues = false;
    }

    GET_BOOLEAN_VARIABLES;

    END_FUNCTION();
}

fmi2Status fmi2GetString (fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[]) {

    BEGIN_FUNCTION(GetString)

    if (nvr>0 && nullPointer(S, "fmi2GetString", "vr[]", vr))
            return fmi2Error;

    if (nvr>0 && nullPointer(S, "fmi2GetString", "value[]", value))
            return fmi2Error;

    if (nvr > 0 && S->isDirtyValues) {
        calculateValues(S);
        S->isDirtyValues = false;
    }

    GET_VARIABLES(String);

    END_FUNCTION();
}

fmi2Status fmi2SetReal (fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]) {

    BEGIN_FUNCTION(SetReal)

    if (nvr > 0 && nullPointer(S, "fmi2SetReal", "vr[]", vr))
        return fmi2Error;

    if (nvr > 0 && nullPointer(S, "fmi2SetReal", "value[]", value))
        return fmi2Error;

    SET_VARIABLES(Float64);

    END_FUNCTION();
}

fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]) {

    BEGIN_FUNCTION(SetInteger)

    if (nvr > 0 && nullPointer(S, "fmi2SetInteger", "vr[]", vr))
        return fmi2Error;

    if (nvr > 0 && nullPointer(S, "fmi2SetInteger", "value[]", value))
        return fmi2Error;

    SET_VARIABLES(Int32);

    END_FUNCTION();
}

fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]) {

    BEGIN_FUNCTION(SetBoolean)

    if (nvr>0 && nullPointer(S, "fmi2SetBoolean", "vr[]", vr))
        return fmi2Error;

    if (nvr>0 && nullPointer(S, "fmi2SetBoolean", "value[]", value))
        return fmi2Error;

    SET_BOOLEAN_VARIABLES;

    END_FUNCTION();
}

fmi2Status fmi2SetString (fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]) {

    BEGIN_FUNCTION(SetString);

    if (nvr>0 && nullPointer(S, "fmi2SetString", "vr[]", vr))
        return fmi2Error;

    if (nvr>0 && nullPointer(S, "fmi2SetString", "value[]", value))
        return fmi2Error;

    SET_VARIABLES(String);

    END_FUNCTION();
}

fmi2Status fmi2GetFMUstate (fmi2Component c, fmi2FMUstate* FMUstate) {

    BEGIN_FUNCTION(GetFMUstate);

    getFMUState(S, FMUstate);

    END_FUNCTION();
}

fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate) {

    BEGIN_FUNCTION(SetFMUstate);

    if (nullPointer(S, "fmi2SetFMUstate", "FMUstate", FMUstate)) {
        return fmi2Error;
    }

    setFMUState(S, FMUstate);

    END_FUNCTION();
}

fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* FMUstate) {

    BEGIN_FUNCTION(FreeFMUstate);

    free(*FMUstate);

    *FMUstate = NULL;

    END_FUNCTION();
}

fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate FMUstate, size_t *size) {

    UNUSED(c);
    UNUSED(FMUstate);

    BEGIN_FUNCTION(SerializedFMUstateSize);

    *size = sizeof(ModelInstance);

    END_FUNCTION();
}

fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size) {

    BEGIN_FUNCTION(SerializeFMUstate);

    if (nullPointer(S, "fmi2SerializeFMUstate", "FMUstate", FMUstate)) {
        return fmi2Error;
    }

    if (invalidNumber(S, "fmi2SerializeFMUstate", "size", size, sizeof(ModelInstance))) {
        return fmi2Error;
    }

    memcpy(serializedState, FMUstate, sizeof(ModelInstance));

    END_FUNCTION();
}

fmi2Status fmi2DeSerializeFMUstate (fmi2Component c, const fmi2Byte serializedState[], size_t size, fmi2FMUstate* FMUstate) {

    BEGIN_FUNCTION(DeSerializeFMUstate);

    if (invalidNumber(S, "fmi2DeSerializeFMUstate", "size", size, sizeof(ModelInstance))) {
        return fmi2Error;
    }

    if (*FMUstate == NULL) {
        *FMUstate = calloc(1, sizeof(ModelInstance));
    }

    memcpy(*FMUstate, serializedState, sizeof(ModelInstance));

    END_FUNCTION();
}

fmi2Status fmi2GetDirectionalDerivative(fmi2Component c, const fmi2ValueReference vUnknown_ref[], size_t nUnknown,
                                        const fmi2ValueReference vKnown_ref[] , size_t nKnown,
                                        const fmi2Real dvKnown[], fmi2Real dvUnknown[]) {

    BEGIN_FUNCTION(GetDirectionalDerivative);

    // TODO: check value references
    // TODO: assert nUnknowns == nDeltaOfUnknowns
    // TODO: assert nKnowns == nDeltaKnowns

    for (size_t i = 0; i < nUnknown; i++) {
        dvUnknown[i] = 0;
        for (size_t j = 0; j < nKnown; j++) {
            double partialDerivative = 0;
            CALL(getPartialDerivative(S, vUnknown_ref[i], vKnown_ref[j], &partialDerivative));
            dvUnknown[i] += partialDerivative * dvKnown[j];
        }
    }

    END_FUNCTION();
}

// ---------------------------------------------------------------------------
// Functions for FMI for Co-Simulation
// ---------------------------------------------------------------------------
/* Simulating the slave */
fmi2Status fmi2SetRealInputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                     const fmi2Integer order[], const fmi2Real value[]) {

    UNUSED(vr);
    UNUSED(nvr);
    UNUSED(order);
    UNUSED(value);

    BEGIN_FUNCTION(SetRealInputDerivatives);

    logError(S, "fmi2SetRealInputDerivatives: ignoring function call."
            " This model cannot interpolate inputs: canInterpolateInputs=\"fmi2False\"");
    CALL(Error);

    END_FUNCTION();
}

fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                      const fmi2Integer order[], fmi2Real value[]) {

    BEGIN_FUNCTION(GetRealOutputDerivatives);

#ifdef GET_OUTPUT_DERIVATIVE
    for (size_t i = 0; i < nvr; i++) {
        CALL(getOutputDerivative(S, vr[i], order[i], &value[i]));
    }
#else
    UNUSED(vr);
    UNUSED(nvr);
    UNUSED(order);
    UNUSED(value);

    logError(S, "fmi2GetRealOutputDerivatives: ignoring function call."
        " This model cannot compute derivatives of outputs: MaxOutputDerivativeOrder=\"0\"");
    CALL(Error);
#endif

    END_FUNCTION();
}

fmi2Status fmi2CancelStep(fmi2Component c) {

    BEGIN_FUNCTION(CancelStep);

    logError(S, "fmi2CancelStep: Can be called when fmi2DoStep returned fmi2Pending."
        " This is not the case.");
    CALL(Error);
    
    END_FUNCTION();
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentCommunicationPoint,
    fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
        
        //fmi2initializeRTLola(c);
    UNUSED(noSetFMUStatePriorToCurrentPoint);
        
    BEGIN_FUNCTION(DoStep);
        
   // char * currentModelState = modelStateToString(S->state); 
    //printf("current model state is %s\n", currentModelState);
    if (fabs(currentCommunicationPoint - S->nextCommunicationPoint) > EPSILON) {
        logError(S, "Expected currentCommunicationPoint = %.16g but was %.16g.",
            S->nextCommunicationPoint, currentCommunicationPoint);
        S->state = Terminated;
        CALL(Error);
    }

    if (communicationStepSize <= 0) {
        logError(S, "Communication step size must be > 0 but was %g.", communicationStepSize);
        CALL(Error);
    }

    if (currentCommunicationPoint + communicationStepSize > S->stopTime + EPSILON) {
        logError(S, "At communication point %.16g a step size of %.16g was requested but stop time is %.16g.",
            currentCommunicationPoint, communicationStepSize, S->stopTime);
        CALL(Error);
    }

    const fmi2Real nextCommunicationPoint = currentCommunicationPoint + communicationStepSize + EPSILON;

    while (true) {

        if (S->time + FIXED_SOLVER_STEP > nextCommunicationPoint) {
            break;  // next communcation point reached
        }

        bool stateEvent, timeEvent;

        CALL(doFixedStep(S, &stateEvent, &timeEvent));

#ifdef EVENT_UPDATE
        if (stateEvent || timeEvent) {
            CALL(eventUpdate(S));
        }
#endif

        if (S->terminateSimulation) {
            status = Discard;
            goto TERMINATE;
        }
    }
    //fmi2ValueReference vr = vr_h;
    // After the step is complete, retrieve and print the height variable
    
    //const unsigned int variablesToChange[] = {vr_h};
    if (&S->rtlola_monitor.is_active) {
        //printf("fmi2DoStep: currentCommunicationPoint = %.16g, communicationStepSize = %.16g\n", currentCommunicationPoint, communicationStepSize);
        
        CALL(RTLolaMonitor_SendData(&S->rtlola_monitor, S->time, c));
        //CALL(sendInputDataToRTLola(S));
        
       // CALL(adjustFMUVariables(&S->rtlola_monitor, variablesToChange, c));
        //CALL(adjustFMUVariables(RTLolaMonitor* monitor, , double speedLimit));
    }
    S->nextCommunicationPoint = currentCommunicationPoint + communicationStepSize;

    END_FUNCTION();
}





/* Inquire slave status */
static Status getStatus(char* fname, ModelInstance* S, const fmi2StatusKind s) {

    switch(s) {
    case fmi2DoStepStatus: logError(S,
        "%s: Can be called with fmi2DoStepStatus when fmi2DoStep returned fmi2Pending."
        " This is not the case.", fname);
        break;
    case fmi2PendingStatus: logError(S,
        "%s: Can be called with fmi2PendingStatus when fmi2DoStep returned fmi2Pending."
        " This is not the case.", fname);
        break;
    case fmi2LastSuccessfulTime: logError(S,
        "%s: Can be called with fmi2LastSuccessfulTime when fmi2DoStep returned fmi2Discard."
        " This is not the case.", fname);
        break;
    case fmi2Terminated: logError(S,
        "%s: Can be called with fmi2Terminated when fmi2DoStep returned fmi2Discard."
        " This is not the case.", fname);
        break;
    }

    return Discard;
}

fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status *value) {

    UNUSED(value);

    BEGIN_FUNCTION(GetStatus);

    CALL(getStatus("fmi2GetStatus", S, s));

    END_FUNCTION();
}

fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real *value) {

    BEGIN_FUNCTION(GetRealStatus);

    if (s == fmi2LastSuccessfulTime) {
        *value = S->time;
        goto TERMINATE;
    }

    CALL(getStatus("fmi2GetRealStatus", c, s));

    END_FUNCTION();
}

fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer *value) {

    UNUSED(value);

    BEGIN_FUNCTION(GetIntegerStatus);

    CALL(getStatus("fmi2GetIntegerStatus", c, s));

    END_FUNCTION();
}

fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean *value) {

    BEGIN_FUNCTION(GetBooleanStatus);

    if (s == fmi2Terminated) {
        *value = S->terminateSimulation;
        goto TERMINATE;
    }

    CALL(getStatus("fmi2GetBooleanStatus", c, s));

    END_FUNCTION();
}

fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String *value) {

    UNUSED(value);

    BEGIN_FUNCTION(GetStringStatus);

    CALL(getStatus("fmi2GetStringStatus", c, s));

    END_FUNCTION();
}

// ---------------------------------------------------------------------------
// Functions for FMI2 for Model Exchange
// ---------------------------------------------------------------------------
/* Enter and exit the different modes */
fmi2Status fmi2EnterEventMode(fmi2Component c) {

    BEGIN_FUNCTION(EnterEventMode);

    S->state = EventMode;

    END_FUNCTION();
}

fmi2Status fmi2NewDiscreteStates(fmi2Component c, fmi2EventInfo *eventInfo) {

    BEGIN_FUNCTION(NewDiscreteStates);

#ifdef EVENT_UPDATE
    CALL(eventUpdate(S));
#endif

    eventInfo->newDiscreteStatesNeeded           = S->newDiscreteStatesNeeded;
    eventInfo->terminateSimulation               = S->terminateSimulation;
    eventInfo->nominalsOfContinuousStatesChanged = S->nominalsOfContinuousStatesChanged;
    eventInfo->valuesOfContinuousStatesChanged   = S->valuesOfContinuousStatesChanged;
    eventInfo->nextEventTimeDefined              = S->nextEventTimeDefined;
    eventInfo->nextEventTime                     = S->nextEventTime;

    END_FUNCTION();
}

fmi2Status fmi2EnterContinuousTimeMode(fmi2Component c) {

    BEGIN_FUNCTION(EnterContinuousTimeMode);

    S->state = ContinuousTimeMode;

    END_FUNCTION();
}

fmi2Status fmi2CompletedIntegratorStep(fmi2Component c, fmi2Boolean noSetFMUStatePriorToCurrentPoint,
                                     fmi2Boolean *enterEventMode, fmi2Boolean *terminateSimulation) {

    UNUSED(noSetFMUStatePriorToCurrentPoint);

    BEGIN_FUNCTION(CompletedIntegratorStep);

    if (nullPointer(S, "fmi2CompletedIntegratorStep", "enterEventMode", enterEventMode))
        CALL(Error);

    if (nullPointer(S, "fmi2CompletedIntegratorStep", "terminateSimulation", terminateSimulation))
        CALL(Error);

    *enterEventMode = fmi2False;
    *terminateSimulation = fmi2False;

    END_FUNCTION();
}

/* Providing independent variables and re-initialization of caching */
fmi2Status fmi2SetTime(fmi2Component c, fmi2Real time) {

    BEGIN_FUNCTION(SetTime);

    S->time = time;

    END_FUNCTION();
}

fmi2Status fmi2SetContinuousStates(fmi2Component c, const fmi2Real x[], size_t nx){

    BEGIN_FUNCTION(SetContinuousStates);

#ifdef HAS_CONTINUOUS_STATES
    if (invalidNumber(S, "fmi2SetContinuousStates", "nx", nx, getNumberOfContinuousStates(S)))
        CALL(Error);

    if (nullPointer(S, "fmi2SetContinuousStates", "x[]", x))
        CALL(Error);

    CALL(setContinuousStates(S, x, nx));
#else
    UNUSED(x);

    if (invalidNumber(S, "fmi2SetContinuousStates", "nx", nx, 0))
        CALL(Error);
#endif

    END_FUNCTION();
}

/* Evaluation of the model equations */
fmi2Status fmi2GetDerivatives(fmi2Component c, fmi2Real derivatives[], size_t nx) {

    BEGIN_FUNCTION(GetDerivatives);

#ifdef HAS_CONTINUOUS_STATES
    if (invalidNumber(S, "fmi2GetDerivatives", "nx", nx, getNumberOfContinuousStates(S)))
        CALL(Error);

    if (nullPointer(S, "fmi2GetDerivatives", "derivatives[]", derivatives))
        CALL(Error);

    CALL(getDerivatives(S, derivatives, nx));
#else
    UNUSED(derivatives);

    if (invalidNumber(S, "fmi2GetDerivatives", "nx", nx, 0))
        CALL(Error);
#endif

    END_FUNCTION();
}

fmi2Status fmi2GetEventIndicators(fmi2Component c, fmi2Real eventIndicators[], size_t ni) {

    BEGIN_FUNCTION(GetEventIndicators);

#ifdef HAS_EVENT_INDICATORS
    if (invalidNumber(S, "fmi2GetEventIndicators", "ni", ni, getNumberOfEventIndicators(S)))
        CALL(Error);

    CALL(getEventIndicators(S, eventIndicators, ni));
#else
    UNUSED(eventIndicators);

    if (invalidNumber(S, "fmi2GetEventIndicators", "ni", ni, 0))
        CALL(Error);
#endif

    END_FUNCTION();
}

fmi2Status fmi2GetContinuousStates(fmi2Component c, fmi2Real states[], size_t nx) {

    BEGIN_FUNCTION(GetContinuousStates);

#ifdef HAS_CONTINUOUS_STATES
    if (invalidNumber(S, "fmi2GetContinuousStates", "nx", nx, getNumberOfContinuousStates(S)))
        CALL(Error);

    if (nullPointer(S, "fmi2GetContinuousStates", "states[]", states))
        CALL(Error);

    CALL(getContinuousStates(S, states, nx));
#else
    UNUSED(states);

    if (invalidNumber(S, "fmi2GetContinuousStates", "nx", nx, 0))
        CALL(Error);
#endif

    END_FUNCTION();
}

fmi2Status fmi2GetNominalsOfContinuousStates(fmi2Component c, fmi2Real x_nominal[], size_t nx) {

    BEGIN_FUNCTION(GetNominalsOfContinuousStates);

#ifdef HAS_CONTINUOUS_STATES
    if (invalidNumber(S, "fmi2GetNominalContinuousStates", "nx", nx, getNumberOfContinuousStates(S)))
        CALL(Error);

    if (nullPointer(S, "fmi2GetNominalContinuousStates", "x_nominal[]", x_nominal))
        CALL(Error);

    for (size_t i = 0; i < nx; i++) {
        x_nominal[i] = 1;
    }
#else
    UNUSED(x_nominal);

    if (invalidNumber(S, "fmi2GetNominalContinuousStates", "nx", nx, 0))
        CALL(Error);
#endif

    END_FUNCTION();
}
