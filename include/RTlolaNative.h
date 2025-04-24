#ifndef RTLOLA_NATIVE_H
#define RTLOLA_NATIVE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration of the RTLolaMonitorHandle
typedef struct RTLolaMonitorHandle RTLolaMonitorHandle;

// RTLola value types
typedef enum {
    RTLOLA_TYPE_UINT64 = 0,
    RTLOLA_TYPE_INT64 = 1,
    RTLOLA_TYPE_FLOAT64 = 2,
    RTLOLA_TYPE_BOOL = 3,
    RTLOLA_TYPE_STRING = 4
} RTLolaValueType;

// Structure for RTLola input values
typedef struct {
    const char* name;       // Input name
    RTLolaValueType type;   // Value type
    union {
        uint64_t uint64_val;
        int64_t int64_val;
        double float64_val;
        bool bool_val;
        const char* string_val;
    } value;
} RTLolaInput;

// Monitor creation/destruction
RTLolaMonitorHandle* rtlola_monitor_new(
    const char* spec, 
    uint64_t timeout_ms,
    const char** input_names,
    uint64_t num_inputs
);



void rtlola_monitor_free(RTLolaMonitorHandle* handle);

// Monitor operations
bool rtlola_monitor_start(RTLolaMonitorHandle* handle);

char * rtlola_process_inputs(
    RTLolaMonitorHandle* handle,
    const RTLolaInput* inputs,
    size_t num_inputs,
    double time
);

void rtlola_free_string(char* s);

// Helper functions for the C implementation
const char** extract_input_names(const char* json_spec);
void free_input_names(const char** names);


#ifdef __cplusplus
}
#endif

#endif // RTLOLA_NATIVE_H