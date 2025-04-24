#ifndef config_h
#define config_h
#include <string.h>
// define class name and unique id
#define MODEL_IDENTIFIER BouncingBall
#define INSTANTIATION_TOKEN "{1AE5E10D-9521-4DE3-80B9-D0EAAA7D5AF1}"

#define CO_SIMULATION
#define MODEL_EXCHANGE

#define HAS_CONTINUOUS_STATES
#define HAS_EVENT_INDICATORS

#define SET_FLOAT64
#define SET_STRING
#define GET_FLOAT64
/*
#define SET_INT32
#define GET_INT32
*/
#define GET_STRING
#define GET_OUTPUT_DERIVATIVE
#define EVENT_UPDATE

#define STRING_MAX_LEN 4096
#define FIXED_SOLVER_STEP 1e-3
#define DEFAULT_STOP_TIME 3

typedef enum {
    vr_time, vr_h, vr_der_h, vr_v, vr_der_v, vr_g, vr_e, vr_v_min,
    vr_rtlola_spec,      // Path to RTLola spec file
    vr_rtlola_output,    // String representing RTLola output
    vr_rtlola_vars,      // Array of variables to monitor
    vr_rtlola_num_vars   // Number of variables (for array size)
} ValueReference;

typedef struct {

    double h;
    double v;
    double g;
    double e;
    char rtlola_spec[STRING_MAX_LEN];
    char rtlola_output[STRING_MAX_LEN];
    ValueReference *rtlola_var_refs;
    size_t rtlola_num_vars;
} ModelData;

#endif /* config_h */
