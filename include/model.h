#pragma once
#ifndef MODEL_H
#define MODEL_H

#define MODEL_H_INCLUDED  

#if FMI_VERSION != 1 && FMI_VERSION != 2 && FMI_VERSION != 3
#error FMI_VERSION must be one of 1, 2 or 3
#endif

#define UNUSED(x) (void)(x)

#include <stddef.h>  // for size_t
#include <stdbool.h> // for bool
#include <stdint.h>

// if linux platform use these includes 

#ifdef __linux__
#include <unistd.h>    // For pipe(), fork(), dup2(), execlp(), close()
#include <sys/wait.h>  // For wait() (if you need to wait for the child process)
#endif  

 

#include <stdlib.h>    // For exit(), EXIT_FAILURE, etc.
#include <stdio.h>     // For printf(), perror(), etc.
#include <sys/types.h> // For pid_t
#include <string.h>    // For string operations like memset(), strcpy()
#include <stdarg.h>
#include "config.h"
#include "RTLolaMonitor.h"
#include "cJSON.h"

#if FMI_VERSION == 1

#define not_modelError (Instantiated| Initialized | Terminated)

typedef enum {
    Instantiated = 1<<0,
    Initialized  = 1<<1,
    Terminated   = 1<<2,
    modelError   = 1<<3
} ModelState;

#elif FMI_VERSION == 2

typedef enum {
    StartAndEnd        = 1<<0,
    Instantiated       = 1<<1,
    InitializationMode = 1<<2,

    // ME states
    EventMode          = 1<<3,
    ContinuousTimeMode = 1<<4,

    // CS states
    StepComplete       = 1<<5,
    StepInProgress     = 1<<6,
    StepFailed         = 1<<7,
    StepCanceled       = 1<<8,

    Terminated         = 1<<9,
} ModelState;

#else

typedef enum {
    StartAndEnd            = 1 << 0,
    ConfigurationMode      = 1 << 1,
    Instantiated           = 1 << 2,
    InitializationMode     = 1 << 3,
    EventMode              = 1 << 4,
    ContinuousTimeMode     = 1 << 5,
    StepMode               = 1 << 6,
    ClockActivationMode    = 1 << 7,
    StepDiscarded          = 1 << 8,
    ReconfigurationMode    = 1 << 9,
    IntermediateUpdateMode = 1 << 10,
    Terminated             = 1 << 11,
} ModelState;

#endif

typedef enum {
    ModelExchange,
    CoSimulation,
    ScheduledExecution,
} InterfaceType;

typedef enum {
    OK,
    Warning,
    Discard,
    Error,
    Fatal,
    Pending,
    Yes
} Status;

typedef enum {
    LOG_ERROR = Error,
    LOG_WARNING = Warning,
    LOG_INFO = OK,
    LOG_DEBUG = OK,  // Custom level for verbose output

} LogLevel;

// Helper macro for conditional logging


#if FMI_VERSION < 3
typedef void (*loggerType) (void *componentEnvironment, const char *instanceName, int status, const char *category, const char *message, ...);
#else
typedef void (*loggerType) (void *componentEnvironment, int status, const char *category, const char *message);
#endif

typedef void (*lockPreemptionType)   (void);
typedef void (*unlockPreemptionType) (void);


typedef void (*intermediateUpdateType) (void *instanceEnvironment,
                                        double intermediateUpdateTime,
                                        bool intermediateVariableSetRequested,
                                        bool intermediateVariableGetAllowed,
                                        bool intermediateStepFinished,
                                        bool canReturnEarly,
                                        bool *earlyReturnRequested,
                                        double *earlyReturnTime);

typedef void(*clockUpdateType) (void *instanceEnvironment);

typedef struct {

    double startTime;
    double stopTime;
    double time;
    const char *instanceName;
    InterfaceType type;
    const char *resourceLocation;

    Status status;

    // callback functions
    loggerType logger;
    intermediateUpdateType intermediateUpdate;
    clockUpdateType clockUpdate;

    lockPreemptionType lockPreemption;
    unlockPreemptionType unlockPreemption;


    //Rtlola monitoring
    FILE *rtlola_input;
    FILE *rtlola_output;
    char *rtlola_spec;
    bool RTLola_Mode;
    ValueReference *rtlola_var_refs;
    size_t rtlola_num_vars;
    RTLolaMonitor rtlola_monitor;


    bool logCustomLogger;
    bool logEvents;
    bool logErrors;

    void *componentEnvironment;
    ModelState state;

    // event info
    bool newDiscreteStatesNeeded;
    bool terminateSimulation;
    bool nominalsOfContinuousStatesChanged;
    bool valuesOfContinuousStatesChanged;
    bool nextEventTimeDefined;
    double nextEventTime;
    bool clocksTicked;

    bool isDirtyValues;

    ModelData modelData;

    // internal solver steps
    uint64_t nSteps;
  
    // Co-Simulation
    bool earlyReturnAllowed;
    bool eventModeUsed;
    double nextCommunicationPoint;

    // solver
    size_t nz;
    double* z;
    double* prez;

    size_t nx;
    double* x;
    double* dx;

} ModelInstance;

ModelInstance *createModelInstance(
    loggerType logger,
    intermediateUpdateType intermediateUpdate,
    void *componentEnvironment,
    const char *instanceName,
    const char *instantiationToken,
    const char *resourceLocation,
    bool loggingOn,
    InterfaceType interfaceType);

void freeModelInstance(ModelInstance *comp);

Status configurate(ModelInstance* comp);

Status reset(ModelInstance* comp);

void setStartValues(ModelInstance* comp);

Status calculateValues(ModelInstance *comp);

Status getFloat32 (ModelInstance* comp, ValueReference vr, float       values[], size_t nValues, size_t *index);
Status getFloat64 (ModelInstance* comp, ValueReference vr, double      values[], size_t nValues, size_t *index);
Status getInt8    (ModelInstance* comp, ValueReference vr, int8_t      values[], size_t nValues, size_t *index);
Status getUInt8   (ModelInstance* comp, ValueReference vr, uint8_t     values[], size_t nValues, size_t *index);
Status getInt16   (ModelInstance* comp, ValueReference vr, int16_t     values[], size_t nValues, size_t *index);
Status getUInt16  (ModelInstance* comp, ValueReference vr, uint16_t    values[], size_t nValues, size_t *index);
Status getInt32   (ModelInstance* comp, ValueReference vr, int32_t     values[], size_t nValues, size_t *index);
Status getUInt32  (ModelInstance* comp, ValueReference vr, uint32_t    values[], size_t nValues, size_t *index);
Status getInt64   (ModelInstance* comp, ValueReference vr, int64_t     values[], size_t nValues, size_t *index);
Status getUInt64  (ModelInstance* comp, ValueReference vr, uint64_t    values[], size_t nValues, size_t *index);
Status getBoolean (ModelInstance* comp, ValueReference vr, bool        values[], size_t nValues, size_t *index);
Status getString  (ModelInstance* comp, ValueReference vr, const char* values[], size_t nValues, size_t *index);
Status getBinary  (ModelInstance* comp, ValueReference vr, size_t sizes[], const char* values[], size_t nValues, size_t *index);

Status setFloat32 (ModelInstance* comp, ValueReference vr, const float       values[], size_t nValues, size_t *index);
Status setFloat64 (ModelInstance* comp, ValueReference vr, const double      values[], size_t nValues, size_t *index);
Status setInt8    (ModelInstance* comp, ValueReference vr, const int8_t      values[], size_t nValues, size_t *index);
Status setUInt8   (ModelInstance* comp, ValueReference vr, const uint8_t     values[], size_t nValues, size_t *index);
Status setInt16   (ModelInstance* comp, ValueReference vr, const int16_t     values[], size_t nValues, size_t *index);
Status setUInt16  (ModelInstance* comp, ValueReference vr, const uint16_t    values[], size_t nValues, size_t *index);
Status setInt32   (ModelInstance* comp, ValueReference vr, const int32_t     values[], size_t nValues, size_t *index);
Status setUInt32  (ModelInstance* comp, ValueReference vr, const uint32_t    values[], size_t nValues, size_t *index);
Status setInt64   (ModelInstance* comp, ValueReference vr, const int64_t     values[], size_t nValues, size_t *index);
Status setUInt64  (ModelInstance* comp, ValueReference vr, const uint64_t    values[], size_t nValues, size_t *index);
Status setBoolean (ModelInstance* comp, ValueReference vr, const bool        values[], size_t nValues, size_t *index);
Status setString  (ModelInstance* comp, ValueReference vr, const char* const values[], size_t nValues, size_t *index);
Status setBinary  (ModelInstance* comp, ValueReference vr, const size_t sizes[], const char* const values[], size_t nValues, size_t *index);

Status activateClock(ModelInstance* comp, ValueReference vr);
Status getClock(ModelInstance* comp, ValueReference vr, bool* value);
Status setClock(ModelInstance* comp, ValueReference vr, const bool* value);

Status getInterval(ModelInstance* comp, ValueReference vr, double* interval, int* qualifier);

Status activateModelPartition(ModelInstance* comp, ValueReference vr, double activationTime);

size_t getNumberOfEventIndicators(ModelInstance* comp);
size_t getNumberOfContinuousStates(ModelInstance* comp);
Status getContinuousStates(ModelInstance *comp, double x[], size_t nx);
Status setContinuousStates(ModelInstance *comp, const double x[], size_t nx);
Status getDerivatives(ModelInstance *comp, double dx[], size_t nx);
Status getOutputDerivative(ModelInstance *comp, ValueReference valueReference, int order, double *value);
Status getPartialDerivative(ModelInstance *comp, ValueReference unknown, ValueReference known, double *partialDerivative);
Status getEventIndicators(ModelInstance *comp, double z[], size_t nz);
Status eventUpdate(ModelInstance *comp);

//RTLOLa funcitonality 
void setMonitorValueRefrences(ModelInstance *comp);
Status getMonitorValues(ModelInstance *comp);
const char* getVariableNameFromValueReference(const unsigned int vr);
unsigned int getValueReferenceFromVariableName(const char* name);
//Unused
void setRTLolaMode(ModelInstance *comp);
void startRTLolaMonitor(ModelInstance *comp);
//void sendDataToRTLola(ModelInstance *comp);
char* modelStateToString(ModelState state);

bool invalidNumber(ModelInstance *comp, const char *f, const char *arg, size_t actual, size_t expected);
bool invalidState(ModelInstance *comp, const char *f, int statesExpected);
bool nullPointer(ModelInstance* comp, const char *f, const char *arg, const void *p);
void logError(ModelInstance *comp, const char *message, ...);
Status setDebugLogging(ModelInstance *comp, bool loggingOn, size_t nCategories, const char * const categories[]);
void logEvent(ModelInstance *comp, const char *message, ...);
void logError(ModelInstance *comp, const char *message, ...);
void logFormatted(ModelInstance *comp, int level, const char *category, const char *format, ...);

Status getFMUState(ModelInstance* comp, void** FMUState);
Status setFMUState(ModelInstance* comp, void* FMUState);

// shorthand to access the variables
#define M(v) (comp->modelData.v)

// "stringification" macros
#define xstr(s) str(s)
#define str(s) #s

// assert size of nValues for scalar variables
#define ASSERT_NVALUES(N) do { \
    const size_t expected_nValues = *index + (N); \
    if (expected_nValues > nValues) { \
        logError(comp, "Expected nValues >= %zu but was %zu.", expected_nValues, nValues); \
        return Error; \
    } \
} while (0)


/*
#define LOG(comp, level, category, format, ...) \
do { \
if (comp->logger && (comp->logEvents || level == LOG_ERROR)) { \
va_list args; \
va_start(args, format); \
logMessage(comp, level, category, format, args); \
va_end(args); \
} \
} while (0)
*/




#endif // MODEL_H