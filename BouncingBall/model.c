#include <math.h>  // for fabs()
#include <float.h> // for DBL_MIN
#include <stddef.h>  // for size_t
#include <stdbool.h> // for bool
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <math.h>
#ifdef _WIN32
    #define POPEN _popen
    #define PCLOSE _pclose
    #define strdup _strdup
#else
    #define strdup strdup
    #define POPEN popen
    #define PCLOSE pclose
#endif

#include "config.h"
#include "model.h"

#define V_MIN (0.1)
#define EVENT_EPSILON (1e-10)



//Struct for monitor capabillities  

void setStartValues(ModelInstance *comp) {
    M(h) =  1;
    M(v) =  0;
    M(g) = -9.81;
    M(e) =  0.7;
    
  
    //M(rtlola_spec) = "path/to/spec.lola";

    strncpy(M(rtlola_spec),  "specifications/bouncing_ball_spec.lola", STRING_MAX_LEN);
    strncpy(M(rtlola_output), "Set me!", STRING_MAX_LEN);
    setRTLolaMode(comp);
    setMonitorValueRefrences(comp);
    const unsigned int monitored_vrs[] = {vr_h};  // Local array
    RTLolaMonitor_Setup(&comp->rtlola_monitor, 
    M(rtlola_spec),
    monitored_vrs,
    M(rtlola_num_vars)
);
    //getMonitorValues(comp);

    
}

Status calculateValues(ModelInstance *comp) {
    UNUSED(comp);
    // nothing to do
    return OK;
}
const char* getVariableNameFromValueReference(const unsigned int vr) {

    switch (vr) {
        case vr_time: return "time";
        case vr_h:    return "h";
        case vr_v:    return "v";
        case vr_g:    return "g";
        case vr_e:    return "e";
        case vr_v_min: return "v_min";
        default:     return "unknown";
    }
}

unsigned int getValueReferenceFromVariableName(const char* name) {
    if (strcmp(name, "time") == 0) return vr_time;
    if (strcmp(name, "h") == 0)    return vr_h;
    if (strcmp(name, "v") == 0)    return vr_v;
    if (strcmp(name, "g") == 0)    return vr_g;
    if (strcmp(name, "e") == 0)    return vr_e;
    if (strcmp(name, "v_min") == 0) return vr_v_min;
    
    return -1; // Unknown variable
}


Status getFloat64(ModelInstance* comp, ValueReference vr, double values[], size_t nValues, size_t* index) {

    ASSERT_NVALUES(1);
   
    switch (vr) {
        case vr_time:    
            values[(*index)++] = comp->time;
            return OK;
        case vr_h:
            values[(*index)++] = M(h);
            return OK;
        case vr_der_h:
        case vr_v:
            values[(*index)++] = M(v);
            return OK;
        case vr_der_v:
        case vr_g:
            values[(*index)++] = M(g);
            return OK;
        case vr_e:
            values[(*index)++] = M(e);
            return OK;
        case vr_v_min:
            values[(*index)++] = V_MIN;
            return OK;
        default:
            logError(comp, "Get Float64 is not allowed for value reference %u.", vr);
            return Error;
    }
}




Status setFloat64(ModelInstance* comp, ValueReference vr, const double value[], size_t nValues, size_t* index) {

    ASSERT_NVALUES(1);
  
    switch (vr) {

        case vr_h:
#if FMI_VERSION > 1
            if (comp->type == ModelExchange &&
                comp->state != Instantiated &&
                comp->state != InitializationMode &&
                comp->state != ContinuousTimeMode) {
                logError(comp, "Variable \"h\" can only be set in Instantiated Mode, Initialization Mode, and Continuous Time Mode.");
                return Error;
            }
#endif

            M(h) = value[(*index)++];
            //printf("h set to: %f\n", M(h));
            return OK;

        case vr_v:
#if FMI_VERSION > 1
            if (comp->type == ModelExchange &&
                comp->state != Instantiated &&
                comp->state != InitializationMode &&
                comp->state != ContinuousTimeMode) {
                logError(comp, "Variable \"v\" can only be set in Instantiated Mode, Initialization Mode, and Continuous Time Mode.");
                return Error;
            }
#endif
            M(v) = value[(*index)++];
            return OK;

        case vr_g:
#if FMI_VERSION > 1
            if (comp->type == ModelExchange &&
                comp->state != Instantiated &&
                comp->state != InitializationMode) {
                logError(comp, "Variable g can only be set after instantiation or in initialization mode.");
                return Error;
            }
#endif
            M(g) = value[(*index)++];
            return OK;

        case vr_e:
#if FMI_VERSION > 1
            if (comp->type == ModelExchange &&
                comp->state != Instantiated &&
                comp->state != InitializationMode &&
                comp->state != EventMode) {
                logError(comp, "Variable e can only be set after instantiation, in initialization mode or event mode.");
                return Error;
            }
#endif
            M(e) = value[(*index)++];
            return OK;

        case vr_v_min:
            logError(comp, "Variable v_min (value reference %u) is constant and cannot be set.", vr_v_min);
            return Error;

        default:
            logError(comp, "Unexpected value reference: %u.", vr);
            return Error;
    }
}
/*
Status getInt32(ModelInstance* comp, ValueReference vr, int32_t values[], size_t nValues, size_t* index) {
    
ASSERT_NVALUES(1);

switch (vr) {
    case vr_rtlola_num_vars:
    values[(*index)++] = M(rtlola_num_vars);
    break;
    
    default:
    logError(comp, "Get Int32 is not allowed for value reference %u.", vr);
    return Error;
}

return OK;
}

Status setInt32(ModelInstance* comp, ValueReference vr, const int32_t values[], size_t nValues, size_t* index) {
    
ASSERT_NVALUES(1);

switch (vr) {
    case vr_rtlola_num_vars:
    M(rtlola_num_vars) = values[(*index)++];
    break;
    default:
    logError(comp, "Set Int32 is not allowed for value reference %u.", vr);
    return Error;
}

comp->isDirtyValues = true;

return OK;
}
*/


Status getString(ModelInstance* comp, ValueReference vr, const char* values[], size_t nValues, size_t* index) {

    ASSERT_NVALUES(1);

    switch (vr) {
        case vr_rtlola_spec:
            values[(*index)++] = M(rtlola_spec);
            break;

        case vr_rtlola_output:
            values[(*index)++] = M(rtlola_output);
            break;
        default:
            logError(comp, "Get String is not allowed for value reference %u.", vr);
            return Error;
    }

    return OK;
}


Status setString(ModelInstance* comp, ValueReference vr, const char* const values[], size_t nValues, size_t* index) {
    ASSERT_NVALUES(1);


    switch (vr) {
        case vr_rtlola_spec:
    
            // Free the old specification path if it exists
            if (comp->rtlola_spec) {
                free(comp->rtlola_spec);
                comp->rtlola_spec = NULL;
            }
            strcpy(M(rtlola_spec), values[(*index)++]);
            // Allocate memory for the new specification path and copy it
            comp->rtlola_spec = strdup(M(rtlola_spec));
            if (!comp->rtlola_spec) {
                logError(comp, "Failed to allocate memory for rtlola_spec.");
                return Error;
            }
           
            // Signal that a specification switch is requested
            size_t new_num_vars =  nValues - 1; // Exclude the first value (the spec itself)
            unsigned int *new_monitor_var_refs = malloc(new_num_vars * sizeof(unsigned int));
            for (size_t i = 0; i < new_num_vars; i++) {
                new_monitor_var_refs[i] = getValueReferenceFromVariableName(values[(*index)++]);
                //printf("new_monitor_var_refs[%ld] = %u\n", i, new_monitor_var_refs[i]);
            }
            if (comp->rtlola_monitor.monitored_vrs) {
                free(comp->rtlola_monitor.monitored_vrs);
            }
            
            // Assign the new array
            comp->rtlola_monitor.monitored_vrs = new_monitor_var_refs;
            comp->rtlola_monitor.num_vars = new_num_vars;
            //print monitor varables
        
            comp->rtlola_monitor.spec_path = strdup(M(rtlola_spec));
            if (!comp->rtlola_monitor.spec_path) {
                logError(comp, "Failed to allocate memory for rtlola_spec.");
                return Error;
            }
            comp->rtlola_monitor.spec_switch_requested = true;
            break;

        case vr_rtlola_vars: {
            printf("Setting rtlola_vars\n");
            // Parse comma-separated value references
            free(comp->rtlola_var_refs);
            comp->rtlola_var_refs = NULL;
            comp->rtlola_num_vars = 0;

            const char* input = values[(*index)++];
            char* copy = strdup(input);
            char* token = strtok(copy, ",");

            while (token) {
                // Parse the value reference
                ValueReference parsed_vr = (ValueReference)atoi(token);

                // Allocate memory for the new value reference
                comp->rtlola_var_refs = realloc(comp->rtlola_var_refs,
                    (comp->rtlola_num_vars + 1) * sizeof(ValueReference));
                if (!comp->rtlola_var_refs) {
                    logError(comp, "Failed to allocate memory for rtlola_var_refs.");
                    free(copy);
                    return Error;
                }

                // Store the parsed value reference
                comp->rtlola_var_refs[comp->rtlola_num_vars++] = parsed_vr;
                token = strtok(NULL, ",");
            }
            free(copy);
            break;
        }
        case vr_rtlola_output:
            strcpy(M(rtlola_output), values[(*index)++]);
            break;


        default:
            logError(comp, "Set String not allowed for VR %u", vr);
            return Error;
    }

    comp->isDirtyValues = true;
    return OK;
}

Status getOutputDerivative(ModelInstance *comp, ValueReference valueReference, int order, double *value) {

    if (order != 1) {
        logError(comp, "The output derivative order %d for value reference %u is not available.", order, valueReference);
        return Error;
    }

    switch (valueReference) {
    case vr_h:
        *value = M(v);
        return OK;
    case vr_v:
        *value = M(g);
        return OK;
    default:
        logError(comp, "The output derivative for value reference %u is not available.", valueReference);
        return Error;
    }
}

Status eventUpdate(ModelInstance *comp) {
    
    //log info 
    logFormatted(comp, LOG_INFO, "EVENT", "FMU Event update called --> Calculating ball bounce.");
    if (M(h) <= 0 && M(v) < 0) {

        M(h) = DBL_MIN;  // slightly above 0 to avoid zero-crossing
        M(v) = -M(v) * M(e);

        if (M(v) < V_MIN) {
            // stop bouncing
            M(v) = 0;
            M(g) = 0;
        }

        comp->valuesOfContinuousStatesChanged = true;
    } else {
        comp->valuesOfContinuousStatesChanged = false;
    }

    comp->nominalsOfContinuousStatesChanged = false;
    comp->terminateSimulation  = false;
    comp->nextEventTimeDefined = false;

    return OK;
}

size_t getNumberOfEventIndicators(ModelInstance* comp) {

    UNUSED(comp);

    return 1;
}

size_t getNumberOfContinuousStates(ModelInstance* comp) {

    UNUSED(comp);

    return 2;
}

Status getContinuousStates(ModelInstance *comp, double x[], size_t nx) {

    UNUSED(nx);

    x[0] = M(h);
    x[1] = M(v);

    return OK;
}

Status setContinuousStates(ModelInstance *comp, const double x[], size_t nx) {

    UNUSED(nx);

    M(h) = x[0];
    M(v) = x[1];

    return OK;
}

Status getDerivatives(ModelInstance *comp, double dx[], size_t nx) {

    UNUSED(nx);

    dx[0] = M(v);
    dx[1] = M(g);

    return OK;
}

Status getEventIndicators(ModelInstance *comp, double z[], size_t nz) {

    UNUSED(nz);

    if (M(h) > -EVENT_EPSILON && M(h) <= 0 && M(v) > 0) {
        // hysteresis for better stability
        z[0] = -EVENT_EPSILON;
    } else {
        z[0] = M(h);
    }
    
    return OK;
}

/*
########Functions for RTLola monitoring###########
##################################################
*/



void setMonitorValueRefrences(ModelInstance *comp){
    const ValueReference vars[] = {vr_h};  // Add more as needed
    const size_t num_vars = sizeof(vars)/sizeof(vars[0]);
    
    free(M(rtlola_var_refs));
    M(rtlola_var_refs) = malloc(num_vars * sizeof(ValueReference));
    if (!M(rtlola_var_refs)) {
        logError(comp, "Memory allocation failed");
        return;
    }
    
    memcpy(M(rtlola_var_refs), vars, num_vars * sizeof(ValueReference));
    M(rtlola_num_vars) = num_vars;  

}

void setRTLolaMode(ModelInstance *comp) {
    comp->RTLola_Mode = (M(rtlola_spec) != NULL);
}
