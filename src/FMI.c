#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#ifdef _WIN32
#else
#include <stdarg.h>
#include <dlfcn.h>
#endif

#ifdef _MSC_VER
#define strdup _strdup
#endif

#include "FMI.h"


void FMIPrintToStdErr(const char* message, va_list args) {
    vfprintf(stderr, message, args);
    fputs("\n", stderr);
}

FMILogErrorMessage* logErrorMessage = FMIPrintToStdErr;

void FMILogError(const char* message, ...) {
    va_list args;
    va_start(args, message);
    logErrorMessage(message, args);
    va_end(args);
}

FMIStatus FMICalloc(void** memory, size_t count, size_t size) {

    if (!memory) {
        FMILogError("Pointer to memory must not be NULL.");
        return FMIError;
    }

    if (count == 0 || size == 0) {
        *memory = NULL;
        return FMIOK;
    }

    *memory = calloc(count, size);

    if (!*memory) {
        FMILogError("Failed to reallocate memory.");
        return FMIError;
    }

    return FMIOK;
}

FMIStatus FMIRealloc(void** memory, size_t size) {

    if (!memory) {
        FMILogError("Pointer to memory must not be NULL.");
        return FMIError;
    }

    if (size == 0) {

        if (*memory) {
            free(*memory);
            *memory = NULL;
        }

    } else {

        void* temp = realloc(*memory, size);

        if (!temp) {
            FMILogError("Failed to reallocate memory.");
            return FMIError;
        }

        *memory = temp;
    }

    return FMIOK;
}

void FMIFree(void** memory) {

    if (*memory) {
        free(*memory);
        *memory = NULL;
    }
}

FMIInstance *FMICreateInstance(const char *instanceName, FMILogMessage *logMessage, FMILogFunctionCall *logFunctionCall) {

    FMIInstance* instance = (FMIInstance*)calloc(1, sizeof(FMIInstance));

    if (!instance) {
        return NULL;
    }

    instance->libraryHandle = NULL;

    instance->logMessage = logMessage;
    instance->logFunctionCall = logFunctionCall;

    instance->logMessageBufferSize = 1024;
    instance->logMessageBuffer = (char*)calloc(instance->logMessageBufferSize, sizeof(char));
    instance->logMessageBufferPosition = 0;

    instance->name = strdup(instanceName);

    instance->status = FMIOK;

    return instance;
}

FMIStatus FMILoadPlatformBinary(FMIInstance* instance, const char* libraryPath) {

# ifdef _WIN32
    WCHAR dllDirectory[MAX_PATH];

    // convert path to unicode
    mbstowcs(dllDirectory, libraryPath, MAX_PATH);

    // replace forward slashes with backslashes
    for (size_t i = 0; i < wcslen(dllDirectory); i++) {
        if (dllDirectory[i] == L'/') {
            dllDirectory[i] = L'\\';
        }
    }

    wchar_t* last = wcsrchr(dllDirectory, L'\\');

    // remove the file spec
    if (last) {
        *last = L'\0';
    }

    // add the binaries directory temporarily to the DLL path to allow discovery of dependencies
    DLL_DIRECTORY_COOKIE dllDirectoryCookie = AddDllDirectory(dllDirectory);

    instance->libraryHandle = LoadLibraryExA(libraryPath, NULL, LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);

    // remove the binaries directory from the DLL path
    if (dllDirectoryCookie) {
        RemoveDllDirectory(dllDirectoryCookie);
    }
# else
    instance->libraryHandle = dlopen(libraryPath, RTLD_LAZY);
# endif

    if (!instance->libraryHandle) {
        FMILogError("Failed to load shared library %s.", libraryPath);
        return FMIError;
    }

    return FMIOK;
}

void FMIFreeInstance(FMIInstance *instance) {

    if (!instance) {
        return;
    }

    // unload the shared library
    if (instance->libraryHandle) {
# ifdef _WIN32
        FreeLibrary(instance->libraryHandle);
# else
        dlclose(instance->libraryHandle);
# endif
        instance->libraryHandle = NULL;
    }

    FMIFree((void**)&instance->logMessageBuffer);

    FMIFree((void**)&instance->name);

    FMIFree((void**)&instance->fmi1Functions);
    FMIFree((void**)&instance->fmi2Functions);
    FMIFree((void**)&instance->fmi3Functions);

    FMIFree((void**)&instance);
}

void FMIClearLogMessageBuffer(FMIInstance* instance) {

    instance->logMessageBufferPosition = 0;

    if (instance->logMessageBufferSize > 0) {
        instance->logMessageBuffer[0] = '\0';
    }
}

void FMIAppendToLogMessageBuffer(FMIInstance* instance, const char* format, ...) {

    va_list args;

    va_start(args, format);
    const int length = vsnprintf(&instance->logMessageBuffer[instance->logMessageBufferPosition], instance->logMessageBufferSize - instance->logMessageBufferPosition, format, args);
    va_end(args);

    if (length < instance->logMessageBufferSize - instance->logMessageBufferPosition) {

        instance->logMessageBufferPosition += length;

    } else {

        while (instance->logMessageBufferSize < instance->logMessageBufferPosition + length) {
            instance->logMessageBufferSize *= 2;
        }

        if (FMIRealloc((void**)&instance->logMessageBuffer, instance->logMessageBufferSize) != FMIOK) {
            return;
        }

        va_start(args, format);
        instance->logMessageBufferPosition += vsnprintf(&instance->logMessageBuffer[instance->logMessageBufferPosition], instance->logMessageBufferSize - instance->logMessageBufferPosition, format, args);
        va_end(args);

    }
}

void FMIAppendArrayToLogMessageBuffer(FMIInstance* instance, const void* values, size_t nValues, const size_t sizes[], FMIVariableType variableType) {

    for (size_t i = 0; i < nValues;) {

        // pointer to the last byte (terminator)
        char* s = &instance->logMessageBuffer[instance->logMessageBufferPosition];

        // remaining bytes in the buffer
        size_t n = instance->logMessageBufferSize - instance->logMessageBufferPosition;

        int length;

        switch (variableType) {
        case FMIFloat32Type:
            length = snprintf(s, n, "%.7g", ((float*)values)[i]);
            break;
        case FMIFloat64Type:
            length = snprintf(s, n, "%.16g", ((double*)values)[i]);
            break;
        case FMIInt8Type:
            length = snprintf(s, n, "%" PRId8, ((int8_t *)values)[i]);
            break;
        case FMIUInt8Type:
            length = snprintf(s, n, "%" PRIu8, ((uint8_t *)values)[i]);
            break;
        case FMIInt16Type:
            length = snprintf(s, n, "%" PRId16, ((int16_t *)values)[i]);
            break;
        case FMIUInt16Type:
            length = snprintf(s, n, "%" PRIu16, ((uint16_t *)values)[i]);
            break;
        case FMIInt32Type:
            length = snprintf(s, n, "%" PRId32, ((int32_t *)values)[i]);
            break;
        case FMIUInt32Type:
            length = snprintf(s, n, "%" PRIu32, ((uint32_t*)values)[i]);
            break;
        case FMIInt64Type:
            length = snprintf(s, n, "%" PRId64, ((int64_t *)values)[i]);
            break;
        case FMIUInt64Type:
            length = snprintf(s, n, "%" PRIu64, ((uint64_t *)values)[i]);
            break;
        case FMIBooleanType:
            switch (instance->fmiMajorVersion) {
                case FMIMajorVersion1:
                    length = snprintf(s, n, "%d", ((char*)values)[i]);
                    break;
                case FMIMajorVersion2:
                    length = snprintf(s, n, "%d", ((int*)values)[i]);
                    break;
                case FMIMajorVersion3:
                    length = snprintf(s, n, "%d", ((bool*)values)[i]);
                    break;
            }
            break;
        case FMIStringType:
            length = snprintf(s, n, "\"%s\"", ((const char**)values)[i]);
            break;
        case FMIBinaryType: {
            const size_t size = sizes[i];
            const unsigned char* v = ((const unsigned char**)values)[i];
            length = 2 + size * 2;
            if (length < n) {
                snprintf(s, n, "0x");
                s += 2;
                n -= 2;
                for (size_t j = 0; j < size; j++) {
                    snprintf(s, n, "%02hhx", v[j]);
                    s += 2;
                    n -= 2;
                }
            }
            break;
        }
        case FMIClockType:
            length = snprintf(s, n, "%d", ((bool *)values)[i]);
            break;
        case FMIValueReferenceType:
            length = snprintf(s, n, "%u", ((FMIValueReference*)values)[i]);
            break;
        case FMISizeTType:
            length = snprintf(s, n, "%zu", ((size_t*)values)[i]);
            break;
        default:
            i++;
            continue;
        }

        if (length + sizeof(", ") < instance->logMessageBufferSize - instance->logMessageBufferPosition) {

            instance->logMessageBufferPosition += length;
            i++;

        } else {

            while (instance->logMessageBufferSize < instance->logMessageBufferPosition + length) {
                instance->logMessageBufferSize *= 2;
            }

            if (FMIRealloc((void**)&instance->logMessageBuffer, instance->logMessageBufferSize) != FMIOK) {
                return;
            }

            continue;
        }

        if (i < nValues) {
            s = &instance->logMessageBuffer[instance->logMessageBufferPosition];
            n = instance->logMessageBufferSize - instance->logMessageBufferPosition;
            instance->logMessageBufferPosition += snprintf(s, n, ", ");
        }
    }
}

FMIStatus FMIPathToURI(const char *path, char *uri, const size_t uriLength) {

    const size_t pathLen = strlen(path);

    if (uriLength < strlen(path) + 8) {
        return FMIError;
    }

#ifdef _WIN32
    const char* scheme = "file:///";
#else
    const char* scheme = "file://";
#endif

    strcpy(uri, scheme);

    size_t p = strlen(scheme);

    // percent encode special characters
    for (size_t i = 0; i < pathLen; i++) {

        if (uriLength < p + 4) {
            return FMIError;
        }

        char c = path[i];

#ifdef _WIN32
        if (c == '\\') {
            c = '/';
        }
#endif

        bool encode = true;

        const char* legal = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-._~:/?#[]@!$&'()*+,;=";

        for (size_t j = 0; j < 84; j++) {
            if (c == legal[j]) {
                encode = false;
                break;
            }
        }

        if (encode) {
            sprintf(&uri[p], "%%%2X", c);
            p += 3;
        } else {
            uri[p++] = c;
        }

    }

    uri[p] = '\0';

    return FMIOK;
}

FMIStatus FMIPlatformBinaryPath(const char *unzipdir, const char *modelIdentifier, FMIMajorVersion fmiMajorVersion, char *platformBinaryPath, size_t size) {

    char* separator = ""; // optional separator after the unzipdir

    const char last = unzipdir[strlen(unzipdir) - 1];

    if (last != '/' && last != '\\') {
        separator = FMI_FILE_SEPARATOR;
    }

    const char* platform = fmiMajorVersion < FMIMajorVersion3 ? FMI_PLATFORM : FMI_PLATFORM_TUPLE;

    const int rc = snprintf(platformBinaryPath, size, "%s%sbinaries" FMI_FILE_SEPARATOR "%s" FMI_FILE_SEPARATOR "%s" FMI_SHARED_LIBRARY_EXTENSION,
        unzipdir, separator, platform, modelIdentifier);

    if (rc >= size) {
        return FMIError;
    }

    return FMIOK;
}
