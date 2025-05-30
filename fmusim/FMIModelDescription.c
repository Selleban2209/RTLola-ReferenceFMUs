#include "FMIModelDescription.h"

#include <string.h>
#include <stdint.h>

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>

#ifdef _WIN32
#include <Windows.h>
#endif

#include "structured_variable_name.tab.h"

// to regenerate run
//
// python xsdflatten.py fmi3ModelDescription.xsd > fmi3Merged.xsd
//
// and
//
// xxd -i fmi3Merged.xsd > fmi3schema.h
#include "fmi1schema.h"
#include "fmi2schema.h"
#include "fmi3schema.h"


#include "FMIUtil.h"


#define CALL(f) do { status = f; if (status > FMIOK) goto TERMINATE; } while (0)

static bool getBooleanAttribute(const xmlNodePtr node, const char* name) {
    char* literal = (char*)xmlGetProp(node, (xmlChar*)name);
    bool value = literal && (strcmp(literal, "true") == 0 || strcmp(literal, "1") == 0);
    xmlFree(literal);
    return value;
}

static int getIntAttribute(const xmlNodePtr node, const char* name) {
    char* literal = (char*)xmlGetProp(node, (xmlChar*)name);
    if (!literal) {
        return 0;
    }
    const int value = strtol(literal, NULL, 0);
    xmlFree(literal);
    return value;
}

static double getDoubleAttribute(const xmlNodePtr node, const char* name, double defaultValue) {
    
    char* literal = (char*)xmlGetProp(node, (xmlChar*)name);
    
    if (literal) {
        const double value = strtod(literal, NULL);
        xmlFree(literal);
        return value;
    }
    
    return defaultValue;
}

static uint32_t getUInt32Attribute(const xmlNodePtr node, const char* name) {
    char* literal = (char*)xmlGetProp(node, (xmlChar*)name);
    uint32_t value = strtoul(literal, NULL, 0);
    xmlFree(literal);
    return value;
}

static FMIVariableNamingConvention getVariableNamingConvention(const xmlNodePtr node) {
    const char* value = (char*)xmlGetProp(node, (xmlChar*)"variableNamingConvention");
    FMIVariableNamingConvention variableNamingConvention = (value && !strcmp(value, "structured")) ? FMIStructured : FMIFlat;
    xmlFree((void*)value);
    return variableNamingConvention;
}

static FMIModelDescription* readModelDescriptionFMI1(xmlNodePtr root) {

    FMIStatus status = FMIOK;

    FMIModelDescription* modelDescription = NULL;

    CALL(FMICalloc((void**)&modelDescription, 1, sizeof(FMIModelDescription)));

    modelDescription->fmiMajorVersion          = FMIMajorVersion1;
    modelDescription->modelName                = (char*)xmlGetProp(root, (xmlChar*)"modelName");
    modelDescription->instantiationToken       = (char*)xmlGetProp(root, (xmlChar*)"guid");
    modelDescription->description              = (char*)xmlGetProp(root, (xmlChar*)"description");
    modelDescription->generationTool           = (char*)xmlGetProp(root, (xmlChar*)"generationTool");
    modelDescription->generationDateAndTime    = (char*)xmlGetProp(root, (xmlChar*)"generationDateAndTime");
    modelDescription->variableNamingConvention = getVariableNamingConvention(root);

    const char* numberOfContinuousStates = (char*)xmlGetProp(root, (xmlChar*)"numberOfContinuousStates");
    
    if (numberOfContinuousStates) {
        modelDescription->nContinuousStates = atoi(numberOfContinuousStates);
    }
    
    xmlFree((void*)numberOfContinuousStates);

    const char* numberOfEventIndicators = (char*)xmlGetProp(root, (xmlChar*)"numberOfEventIndicators");
    
    if (numberOfEventIndicators) {
        modelDescription->nEventIndicators = atoi(numberOfEventIndicators);
    }
    
    xmlFree((void*)numberOfEventIndicators);

    xmlXPathContextPtr xpathCtx = xmlXPathNewContext(root->doc);

    xmlXPathObjectPtr xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/Implementation/CoSimulation_StandAlone", xpathCtx);
    
    if (xpathObj->nodesetval->nodeNr == 1) {
        CALL(FMICalloc((void**)&modelDescription->coSimulation, 1, sizeof(FMICoSimulationInterface)));
        modelDescription->coSimulation->modelIdentifier = (char*)xmlGetProp(root, (xmlChar*)"modelIdentifier");
    } else {
        CALL(FMICalloc((void**)&modelDescription->modelExchange, 1, sizeof(FMIModelExchangeInterface)));
        modelDescription->modelExchange->modelIdentifier = (char*)xmlGetProp(root, (xmlChar*)"modelIdentifier");
    }
    
    xmlXPathFreeObject(xpathObj);

    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/DefaultExperiment", xpathCtx);
    
    if (xpathObj->nodesetval->nodeNr == 1) {
        CALL(FMICalloc((void**)&modelDescription->defaultExperiment, 1, sizeof(FMIDefaultExperiment)));
        const xmlNodePtr node = xpathObj->nodesetval->nodeTab[0];
        modelDescription->defaultExperiment->startTime = (char*)xmlGetProp(node, (xmlChar*)"startTime");
        modelDescription->defaultExperiment->stopTime  = (char*)xmlGetProp(node, (xmlChar*)"stopTime");
        modelDescription->defaultExperiment->tolerance = (char*)xmlGetProp(node, (xmlChar*)"tolerance");
    }
    
    xmlXPathFreeObject(xpathObj);

    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/ModelVariables/ScalarVariable/*[self::Real or self::Integer or self::Enumeration or self::Boolean or self::String]", xpathCtx);

    modelDescription->nModelVariables = xpathObj->nodesetval->nodeNr;

    CALL(FMICalloc((void**)&modelDescription->modelVariables, xpathObj->nodesetval->nodeNr, sizeof(FMIModelVariable*)));

    for (size_t i = 0; i < xpathObj->nodesetval->nodeNr; i++) {

        xmlNodePtr typeNode = xpathObj->nodesetval->nodeTab[i];
        xmlNodePtr variableNode = typeNode->parent;

        FMIModelVariable* variable;
        
        CALL(FMICalloc((void**)&variable, 1, sizeof(FMIModelVariable)));

        modelDescription->modelVariables[i] = variable;

        variable->line = variableNode->line;

        variable->name        = (char*)xmlGetProp(variableNode, (xmlChar*)"name");
        variable->description = (char*)xmlGetProp(variableNode, (xmlChar*)"description");

        variable->min     = (char*)xmlGetProp(typeNode, (xmlChar*)"min");
        variable->max     = (char*)xmlGetProp(typeNode, (xmlChar*)"max");
        variable->nominal = (char*)xmlGetProp(typeNode, (xmlChar*)"nominal");
        variable->start   = (char*)xmlGetProp(typeNode, (xmlChar*)"start");

        const char* typeName = (char*)typeNode->name;

        const char* causality = (char*)xmlGetProp(variableNode, (xmlChar*)"causality");

        if (!causality) {
            // default
            variable->causality = FMILocal;
        } else if (!strcmp(causality, "input")) {
            variable->causality = FMIInput;
        } else if (!strcmp(causality, "output")) {
            variable->causality = FMIOutput;
        } else {
            // "internal" or "none"
            variable->causality = FMILocal;
        }

        xmlFree((void*)causality);

        const bool fixed = getBooleanAttribute(typeNode, "fixed");

        const char* variability = (char*)xmlGetProp(variableNode, (xmlChar*)"variability");

        if (!variability) { // default
            variable->variability = FMIContinuous;
        } else if (!strcmp(variability, "constant")) {
            variable->variability = FMIConstant;
        } else if (!strcmp(variability, "parameter")) {
            variable->causality = FMIParameter;
            variable->variability = fixed ? FMIFixed : FMITunable;
        } else if (!strcmp(variability, "discrete")) {
            variable->variability = FMIDiscrete;
        } else if (!strcmp(variability, "continuous")) {
            variable->variability = FMIContinuous;
        }

        xmlFree((void*)variability);

        if (!strcmp(typeName, "Real")) {
            variable->type = FMIRealType;
        } else if (!strcmp(typeName, "Integer") || !strcmp(typeName, "Enumeration")) {
            variable->type = FMIIntegerType;
        } else if (!strcmp(typeName, "Boolean")) {
            variable->type = FMIBooleanType;
        } else if (!strcmp(typeName, "String")) {
            variable->type = FMIStringType;
        } else {
            continue;
        }

        variable->valueReference = getUInt32Attribute(variableNode, "valueReference");

    }

    size_t nProblems = 0;

    // check variabilities
    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {
        const FMIModelVariable* variable = modelDescription->modelVariables[i];
        if (variable->type != FMIRealType && variable->variability == FMIContinuous) {
            FMILogError("Variable \"%s\" is not of type Real but has variability = continuous.\n", variable->name);
            nProblems++;
        }
    }

    xmlXPathFreeObject(xpathObj);

    xmlXPathFreeContext(xpathCtx);

    if (nProblems > 0) {
        FMIFreeModelDescription(modelDescription);
        modelDescription = NULL;
    }

TERMINATE:

    if (status != FMIOK) {
        FMIFree((void**)&modelDescription);
    }

    return modelDescription;
}

static FMIStatus readUnknownsFMI2(xmlXPathContextPtr xpathCtx, FMIModelDescription* modelDescription, const char* path, size_t* nUnkonwns, FMIUnknown*** unknowns) {

    FMIStatus status = FMIOK;

    xmlXPathObjectPtr xpathObj = xmlXPathEvalExpression((xmlChar*)path, xpathCtx);
    
    *nUnkonwns = xpathObj->nodesetval->nodeNr;

    CALL(FMICalloc((void**)unknowns, *nUnkonwns, sizeof(FMIUnknown*)));

    for (size_t i = 0; i < xpathObj->nodesetval->nodeNr; i++) {

        xmlNodePtr unkownNode = xpathObj->nodesetval->nodeTab[i];

        FMIUnknown* unknown;

        CALL(FMICalloc((void**)&unknown, 1, sizeof(FMIUnknown)));

        (*unknowns)[i] = unknown;

        const char* indexLiteral = (char*)xmlGetProp(unkownNode, (xmlChar*)"index");

        unknown->modelVariable = FMIModelVariableForIndexLiteral(modelDescription, indexLiteral);

        xmlFree((void*)indexLiteral);
    }

    xmlXPathFreeObject(xpathObj);

TERMINATE:

    return status;
}

static FMIStatus readUnknownsFMI3(xmlXPathContextPtr xpathCtx, FMIModelDescription* modelDescription, const char* path, size_t* nUnkonwns, FMIUnknown*** unknowns) {

    FMIStatus status = FMIOK;

    xmlXPathObjectPtr xpathObj = xmlXPathEvalExpression((xmlChar*)path, xpathCtx);

    *nUnkonwns = xpathObj->nodesetval->nodeNr;

    CALL(FMICalloc((void**)unknowns, *nUnkonwns, sizeof(FMIUnknown*)));

    for (size_t i = 0; i < xpathObj->nodesetval->nodeNr; i++) {

        xmlNodePtr unknownNode = xpathObj->nodesetval->nodeTab[i];

        FMIUnknown* unknown;

        CALL(FMICalloc((void**)&unknown, 1, sizeof(FMIUnknown)));

        (*unknowns)[i] = unknown;

        FMIValueReference valueReference = getUInt32Attribute(unknownNode, "valueReference");

        for (size_t j = 0; j < modelDescription->nModelVariables; j++) {

            FMIModelVariable* variable = modelDescription->modelVariables[j];
            
            if (variable->valueReference == valueReference) {
                unknown->modelVariable = variable;
                break;
            }
        }
    }

    xmlXPathFreeObject(xpathObj);

TERMINATE:

    return status;
}

static FMIStatus readSourceFiles(xmlXPathContextPtr xpathCtx, const char* path, size_t* nSourceFiles, char*** sourceFiles) {

    FMIStatus status = FMIOK;

    xmlXPathObjectPtr xpathObj = xmlXPathEvalExpression((xmlChar*)path, xpathCtx);

    if (xpathObj->nodesetval->nodeNr > 0) {

        *nSourceFiles = xpathObj->nodesetval->nodeNr;

        CALL(FMICalloc((void**)sourceFiles, xpathObj->nodesetval->nodeNr, sizeof(char*)));

        for (int i = 0; i < xpathObj->nodesetval->nodeNr; i++) {
            xmlNodePtr fileNode = xpathObj->nodesetval->nodeTab[i];
            (*sourceFiles)[i] = (char*)xmlGetProp(fileNode, (xmlChar*)"name");
        }
    }

    xmlXPathFreeObject(xpathObj);

TERMINATE:
    
    return status;
}

static FMIModelDescription* readModelDescriptionFMI2(xmlNodePtr root) {

    FMIStatus status = FMIOK;

    size_t nProblems = 0;

    FMIModelDescription* modelDescription = NULL;
    
    CALL(FMICalloc((void**)&modelDescription, 1, sizeof(FMIModelDescription)));

    modelDescription->fmiMajorVersion          = FMIMajorVersion2;
    modelDescription->modelName                = (char*)xmlGetProp(root, (xmlChar*)"modelName");
    modelDescription->instantiationToken       = (char*)xmlGetProp(root, (xmlChar*)"guid");
    modelDescription->description              = (char*)xmlGetProp(root, (xmlChar*)"description");
    modelDescription->generationTool           = (char*)xmlGetProp(root, (xmlChar*)"generationTool");
    modelDescription->generationDateAndTime    = (char*)xmlGetProp(root, (xmlChar*)"generationDateAndTime");
    modelDescription->variableNamingConvention = getVariableNamingConvention(root);

    const char* numberOfEventIndicators = (char*)xmlGetProp(root, (xmlChar*)"numberOfEventIndicators");

    if (numberOfEventIndicators) {
        modelDescription->nEventIndicators = atoi(numberOfEventIndicators);
    }

    xmlXPathContextPtr xpathCtx = xmlXPathNewContext(root->doc);

    xmlXPathObjectPtr xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/ModelExchange", xpathCtx);
    if (xpathObj->nodesetval->nodeNr == 1) {
        CALL(FMICalloc((void**)&modelDescription->modelExchange, 1, sizeof(FMIModelExchangeInterface)));
        const xmlNodePtr node = xpathObj->nodesetval->nodeTab[0];
        modelDescription->modelExchange->modelIdentifier = (char*)xmlGetProp(node, (xmlChar*)"modelIdentifier");
        modelDescription->modelExchange->providesDirectionalDerivatives = getBooleanAttribute(node, "providesDirectionalDerivative");
        modelDescription->modelExchange->needsCompletedIntegratorStep = !getBooleanAttribute(node, "completedIntegratorStepNotNeeded");
        CALL(readSourceFiles(xpathCtx, "/fmiModelDescription/ModelExchange/SourceFiles/File", &modelDescription->modelExchange->nSourceFiles, &modelDescription->modelExchange->sourceFiles));
    }
    xmlXPathFreeObject(xpathObj);

    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/CoSimulation", xpathCtx);
    if (xpathObj->nodesetval->nodeNr == 1) {
        CALL(FMICalloc((void**)&modelDescription->coSimulation, 1, sizeof(FMICoSimulationInterface)));
        modelDescription->coSimulation->modelIdentifier = (char*)xmlGetProp(xpathObj->nodesetval->nodeTab[0], (xmlChar*)"modelIdentifier");
        CALL(readSourceFiles(xpathCtx, "/fmiModelDescription/CoSimulation/SourceFiles/File", &modelDescription->coSimulation->nSourceFiles, &modelDescription->coSimulation->sourceFiles));
    }
    xmlXPathFreeObject(xpathObj);

    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/DefaultExperiment", xpathCtx);
    if (xpathObj->nodesetval->nodeNr == 1) {
        CALL(FMICalloc((void**)&modelDescription->defaultExperiment, 1, sizeof(FMIDefaultExperiment)));
        const xmlNodePtr node = xpathObj->nodesetval->nodeTab[0];
        modelDescription->defaultExperiment->startTime = (char*)xmlGetProp(node, (xmlChar*)"startTime");
        modelDescription->defaultExperiment->stopTime  = (char*)xmlGetProp(node, (xmlChar*)"stopTime");
        modelDescription->defaultExperiment->tolerance = (char*)xmlGetProp(node, (xmlChar*)"tolerance");
        modelDescription->defaultExperiment->stepSize  = (char*)xmlGetProp(node, (xmlChar*)"stepSize");
    }
    xmlXPathFreeObject(xpathObj);

    // unit definitions
    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/UnitDefinitions/Unit", xpathCtx);

    modelDescription->nUnits = xpathObj->nodesetval->nodeNr;
    CALL(FMICalloc((void**)&modelDescription->units, xpathObj->nodesetval->nodeNr, sizeof(FMIUnit*)));

    for (size_t i = 0; i < xpathObj->nodesetval->nodeNr; i++) {

        FMIUnit* unit = NULL;

        CALL(FMICalloc((void**)&unit, 1, sizeof(FMIUnit)));
            
        modelDescription->units[i] = unit;

        xmlNodePtr unitNode = xpathObj->nodesetval->nodeTab[i];

        xmlNodePtr childNode = unitNode->children;

        while (childNode) {

            if (childNode->name && !strcmp(childNode->name, "BaseUnit")) {

                FMIBaseUnit* baseUnit = NULL;
                
                CALL(FMICalloc((void**)&baseUnit, 1, sizeof(FMIBaseUnit)));

                baseUnit->kg  = getIntAttribute(childNode, "kg");
                baseUnit->m   = getIntAttribute(childNode, "m");
                baseUnit->s   = getIntAttribute(childNode, "s");
                baseUnit->A   = getIntAttribute(childNode, "A");
                baseUnit->K   = getIntAttribute(childNode, "K");
                baseUnit->mol = getIntAttribute(childNode, "mol");
                baseUnit->cd  = getIntAttribute(childNode, "cd");
                baseUnit->rad = getIntAttribute(childNode, "rad");

                baseUnit->factor = getDoubleAttribute(childNode, "factor", 1.0);
                baseUnit->offset = getDoubleAttribute(childNode, "offset", 0.0);

                unit->baseUnit = baseUnit;

            } else if (childNode->name && !strcmp(childNode->name, "DisplayUnit")) {

                CALL(FMIRealloc((void**)&unit->displayUnits, (unit->nDisplayUnits + 1) * sizeof(FMIDisplayUnit*)));

                FMIDisplayUnit* displayUnit = NULL;

                CALL(FMICalloc((void**)&displayUnit, 1, sizeof(FMIDisplayUnit)));

                displayUnit->name = (char*)xmlGetProp(childNode, (xmlChar*)"name");
                displayUnit->factor = getDoubleAttribute(childNode, "factor", 1.0);
                displayUnit->offset = getDoubleAttribute(childNode, "offset", 0.0);

                unit->displayUnits[unit->nDisplayUnits] = displayUnit;
                unit->nDisplayUnits++;
            }

            childNode = childNode->next;
        }

        unit->name = (char*)xmlGetProp(unitNode, (xmlChar*)"name");
    }

    // type definitions
    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/TypeDefinitions/SimpleType/*[self::Real or self::Integer or self::Enumeration or self::Boolean or self::String]", xpathCtx);

    if (xpathObj->nodesetval) {

        modelDescription->nTypeDefinitions = xpathObj->nodesetval->nodeNr;
        CALL(FMICalloc((void**)&modelDescription->typeDefinitions, xpathObj->nodesetval->nodeNr, sizeof(FMITypeDefinition*)));

        for (size_t i = 0; i < xpathObj->nodesetval->nodeNr; i++) {

            const xmlNodePtr typeNode = xpathObj->nodesetval->nodeTab[i];
            const xmlNodePtr simpleTypeNode = typeNode->parent;

            FMITypeDefinition* typeDefinition = NULL;

            CALL(FMICalloc((void**)&typeDefinition, 1, sizeof(FMITypeDefinition)));

            modelDescription->typeDefinitions[i] = typeDefinition;

            typeDefinition->name = (char*)xmlGetProp(simpleTypeNode, (xmlChar*)"name");
            
            typeDefinition->quantity = (char*)xmlGetProp(typeNode, (xmlChar*)"quantity");

            const char* unit = (char*)xmlGetProp(typeNode, (xmlChar*)"unit");
            
            if (unit) {
                typeDefinition->unit = FMIUnitForName(modelDescription, unit);
                if (!typeDefinition->unit) {
                    FMILogError("Unit \"%s\" of type defintion \"%s\" is not defined.", unit, typeDefinition->name);
                    nProblems++;
                }
                xmlFree((void*)unit);
            }

            const char* displayUnit = (char*)xmlGetProp(typeNode, (xmlChar*)"displayUnit");

            if (displayUnit) {
                typeDefinition->displayUnit = FMIDisplayUnitForName(typeDefinition->unit, displayUnit);
                if (!typeDefinition->displayUnit) {
                    FMILogError("Display unit \"%s\" of type defintion \"%s\" is not defined.", displayUnit, typeDefinition->name);
                    nProblems++;
                }
                xmlFree((void*)displayUnit);
            }

            typeDefinition->relativeQuantity = getBooleanAttribute(typeNode, "relativeQuantity");
            typeDefinition->min              = (char*)xmlGetProp(typeNode, (xmlChar*)"min");
            typeDefinition->max              = (char*)xmlGetProp(typeNode, (xmlChar*)"max");
            typeDefinition->nominal          = (char*)xmlGetProp(typeNode, (xmlChar*)"nominal");
            typeDefinition->unbounded        = getBooleanAttribute(typeNode, "unbounded");

            // TODO: variable type
        }
    }

    // model variables
    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/ModelVariables/ScalarVariable/*[self::Real or self::Integer or self::Enumeration or self::Boolean or self::String]", xpathCtx);

    modelDescription->nModelVariables = xpathObj->nodesetval->nodeNr;
    CALL(FMICalloc((void**)&modelDescription->modelVariables, modelDescription->nModelVariables, sizeof(FMIModelVariable*)));

    for (size_t i = 0; i < xpathObj->nodesetval->nodeNr; i++) {

        const xmlNodePtr typeNode = xpathObj->nodesetval->nodeTab[i];
        const xmlNodePtr variableNode = typeNode->parent;

        FMIModelVariable* variable;
        
        CALL(FMICalloc((void**)&variable, 1, sizeof(FMIModelVariable)));

        modelDescription->modelVariables[i] = variable;

        variable->name        = (char*)xmlGetProp(variableNode, (xmlChar*)"name");        
        variable->description = (char*)xmlGetProp(variableNode, (xmlChar*)"description");

        variable->min     = (char*)xmlGetProp(typeNode, (xmlChar*)"min");
        variable->max     = (char*)xmlGetProp(typeNode, (xmlChar*)"max");
        variable->nominal = (char*)xmlGetProp(typeNode, (xmlChar*)"nominal");
        variable->start   = (char*)xmlGetProp(typeNode, (xmlChar*)"start");

        variable->line = variableNode->line;

        variable->derivative = (FMIModelVariable*)xmlGetProp(typeNode, (xmlChar*)"derivative");

        const char* unit = (char*)xmlGetProp(typeNode, (xmlChar*)"unit");
        
        if (unit) {
            variable->unit = FMIUnitForName(modelDescription, unit);
            if (!variable->unit) {
                FMILogError("Unit \"%s\" of model variable \"%s\" (line %hu) is not defined.", unit, variable->name, variable->line);
                nProblems++;
            }
            xmlFree((void*)unit);
        }

        const char* declaredType = (char*)xmlGetProp(typeNode, (xmlChar*)"declaredType");
        
        if (declaredType) {
            variable->declaredType = FMITypeDefintionForName(modelDescription, declaredType);
            if (!variable->declaredType) {
                FMILogError("Declared type \"%s\" of model variable \"%s\" (line %hu) is not defined.", declaredType, variable->name, variable->line);
                nProblems++;
            }
            xmlFree((void*)declaredType);
        }

        const char* typeName = (char*)typeNode->name;

        const char* variability = (char*)xmlGetProp(variableNode, (xmlChar*)"variability");
        
        if (!variability) {
            variable->variability = FMIContinuous;
        } else if(!strcmp(variability, "constant")) {
            variable->variability = FMIConstant;
        } else if (!strcmp(variability, "fixed")) {
            variable->variability = FMIFixed;
        } else if (!strcmp(variability, "tunable")) {
            variable->variability = FMITunable;
        } else if (!strcmp(variability, "discrete")) {
            variable->variability = FMIDiscrete;
        } else {
            variable->variability = FMIContinuous;
        }

        xmlFree((void*)variability);

        if (!strcmp(typeName, "Real")) {
            variable->type =  FMIRealType;
        } else if (!strcmp(typeName, "Integer") || !strcmp(typeName, "Enumeration")) {
            variable->type = FMIIntegerType;
        } else if (!strcmp(typeName, "Boolean")) {
            variable->type = FMIBooleanType;
        } else if (!strcmp(typeName, "String")) {
            variable->type = FMIStringType;
        } else {
            continue;
        }

        const char* vr = (char*)xmlGetProp(variableNode, (xmlChar*)"valueReference");

        variable->valueReference = FMIValueReferenceForLiteral(vr);

        xmlFree((void*)vr);
        
        const char* causality = (char*)xmlGetProp(variableNode, (xmlChar*)"causality");

        if (!causality) {
            variable->causality = FMILocal;
        } else if (!strcmp(causality, "parameter")) {
            variable->causality = FMIParameter;
        } else if (!strcmp(causality, "input")) {
            variable->causality = FMIInput;
        } else if (!strcmp(causality, "output")) {
            variable->causality = FMIOutput;
        } else if (!strcmp(causality, "independent")) {
            variable->causality = FMIIndependent;
        } else {
            variable->causality = FMILocal;
        }

        xmlFree((void*)causality);
    }

    xmlXPathFreeObject(xpathObj);

    CALL(readUnknownsFMI2(xpathCtx, modelDescription, "/fmiModelDescription/ModelStructure/Outputs/Unknown", &modelDescription->nOutputs, &modelDescription->outputs));
    CALL(readUnknownsFMI2(xpathCtx, modelDescription, "/fmiModelDescription/ModelStructure/Derivatives/Unknown", &modelDescription->nContinuousStates, &modelDescription->derivatives));
    CALL(readUnknownsFMI2(xpathCtx, modelDescription, "/fmiModelDescription/ModelStructure/InitialUnknowns/Unknown", &modelDescription->nInitialUnknowns, &modelDescription->initialUnknowns));

    xmlXPathFreeContext(xpathCtx);

    // resolve derivatives
    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {
        FMIModelVariable* variable = modelDescription->modelVariables[i];
        if (variable->derivative) {
            char* literal = (char*)variable->derivative;
            variable->derivative = FMIModelVariableForIndexLiteral(modelDescription, literal);
            if (!variable->derivative) {
                FMILogError("Failed to resolve attribute derivative=\"%s\" for model variable \"%s\".", literal, variable->name);
                nProblems++;
            }
            xmlFree(literal);
        }
    }

    // check variabilities
    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {
        FMIModelVariable* variable = modelDescription->modelVariables[i];
        if (variable->type != FMIRealType && variable->variability == FMIContinuous) {
            FMILogError("Variable \"%s\" is not of type Real but has variability = continuous.\n", variable->name);
            nProblems++;
        }
    }

    nProblems += FMIValidateModelDescription(modelDescription);

    if (nProblems > 0) {
        FMIFreeModelDescription(modelDescription);
        modelDescription = NULL;
    }

TERMINATE:
    
    // TODO
    
    return modelDescription;
}

static FMIModelDescription* readModelDescriptionFMI3(xmlNodePtr root) {

    FMIStatus status = FMIOK;

    size_t nProblems = 0;
    
    FMIModelDescription* modelDescription = NULL;

    CALL(FMICalloc((void**)&modelDescription, 1, sizeof(FMIModelDescription)));

    modelDescription->fmiMajorVersion          = FMIMajorVersion3;
    modelDescription->modelName                = (char*)xmlGetProp(root, (xmlChar*)"modelName");
    modelDescription->instantiationToken       = (char*)xmlGetProp(root, (xmlChar*)"instantiationToken");
    modelDescription->description              = (char*)xmlGetProp(root, (xmlChar*)"description");
    modelDescription->generationTool           = (char*)xmlGetProp(root, (xmlChar*)"generationTool");
    modelDescription->generationDateAndTime    = (char*)xmlGetProp(root, (xmlChar*)"generationDateAndTime");
    modelDescription->variableNamingConvention = getVariableNamingConvention(root);

    xmlXPathContextPtr xpathCtx = xmlXPathNewContext(root->doc);

    xmlXPathObjectPtr xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/CoSimulation", xpathCtx);
    if (xpathObj->nodesetval->nodeNr == 1) {
        CALL(FMICalloc((void**)&modelDescription->coSimulation, 1, sizeof(FMICoSimulationInterface)));
        const xmlNodePtr node = xpathObj->nodesetval->nodeTab[0];
        modelDescription->coSimulation->modelIdentifier = (char*)xmlGetProp(node, (xmlChar*)"modelIdentifier");
        modelDescription->coSimulation->hasEventMode = getBooleanAttribute(node, "hasEventMode");
    }
    xmlXPathFreeObject(xpathObj);

    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/ModelExchange", xpathCtx);
    if (xpathObj->nodesetval->nodeNr == 1) {
        const xmlNodePtr node = xpathObj->nodesetval->nodeTab[0];
        CALL(FMICalloc((void**)&modelDescription->modelExchange, 1, sizeof(FMIModelExchangeInterface)));
        modelDescription->modelExchange->modelIdentifier = (char*)xmlGetProp(node, (xmlChar*)"modelIdentifier");
        modelDescription->modelExchange->providesDirectionalDerivatives = getBooleanAttribute(node, "providesDirectionalDerivatives");
        modelDescription->modelExchange->needsCompletedIntegratorStep = getBooleanAttribute(node, "needsCompletedIntegratorStep");
    }
    xmlXPathFreeObject(xpathObj);

    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/DefaultExperiment", xpathCtx);
    if (xpathObj->nodesetval->nodeNr == 1) {
        CALL(FMICalloc((void**)&modelDescription->defaultExperiment, 1, sizeof(FMIDefaultExperiment)));
        const xmlNodePtr node = xpathObj->nodesetval->nodeTab[0];
        modelDescription->defaultExperiment->startTime = (char*)xmlGetProp(node, (xmlChar*)"startTime");
        modelDescription->defaultExperiment->stopTime  = (char*)xmlGetProp(node, (xmlChar*)"stopTime");
        modelDescription->defaultExperiment->tolerance = (char*)xmlGetProp(node, (xmlChar*)"tolerance");
        modelDescription->defaultExperiment->stepSize  = (char*)xmlGetProp(node, (xmlChar*)"stepSize");
    }
    xmlXPathFreeObject(xpathObj);

    // unit definitions
    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/UnitDefinitions/Unit", xpathCtx);

    modelDescription->nUnits = xpathObj->nodesetval->nodeNr;
    CALL(FMICalloc((void**)&modelDescription->units, xpathObj->nodesetval->nodeNr, sizeof(FMIUnit*)));

    for (size_t i = 0; i < xpathObj->nodesetval->nodeNr; i++) {

        FMIUnit* unit = NULL;

        CALL(FMICalloc((void**)&unit, 1, sizeof(FMIUnit)));

        modelDescription->units[i] = unit;

        xmlNodePtr unitNode = xpathObj->nodesetval->nodeTab[i];

        xmlNodePtr childNode = unitNode->children;

        while (childNode) {

            if (childNode->name && !strcmp((char*)childNode->name, "BaseUnit")) {

                FMIBaseUnit* baseUnit = NULL;

                CALL(FMICalloc((void**)&baseUnit, 1, sizeof(FMIBaseUnit)));

                baseUnit->kg  = getIntAttribute(childNode, "kg");
                baseUnit->m   = getIntAttribute(childNode, "m");
                baseUnit->s   = getIntAttribute(childNode, "s");
                baseUnit->A   = getIntAttribute(childNode, "A");
                baseUnit->K   = getIntAttribute(childNode, "K");
                baseUnit->mol = getIntAttribute(childNode, "mol");
                baseUnit->cd  = getIntAttribute(childNode, "cd");
                baseUnit->rad = getIntAttribute(childNode, "rad");

                baseUnit->factor = getDoubleAttribute(childNode, "factor", 1.0);
                baseUnit->offset = getDoubleAttribute(childNode, "offset", 0.0);

                unit->baseUnit = baseUnit;

            } else if (childNode->name && !strcmp((char*)childNode->name, "DisplayUnit")) {

                CALL(FMIRealloc((void**)&unit->displayUnits, (unit->nDisplayUnits + 1) * sizeof(FMIDisplayUnit*)));

                FMIDisplayUnit* displayUnit;

                CALL(FMICalloc((void**)&displayUnit, 1, sizeof(FMIDisplayUnit)));

                displayUnit->name = (char*)xmlGetProp(childNode, (xmlChar*)"name");
                displayUnit->factor = getDoubleAttribute(childNode, "factor", 1.0);
                displayUnit->offset = getDoubleAttribute(childNode, "offset", 0.0);

                unit->displayUnits[unit->nDisplayUnits] = displayUnit;
                unit->nDisplayUnits++;
            }

            childNode = childNode->next;
        }

        unit->name = (char*)xmlGetProp(unitNode, (xmlChar*)"name");
    }

    // type definitions
    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/TypeDefinitions/"
        "*[self::Float32Type"
        " or self::Float64Type"
        " or self::Int8Type"
        " or self::UInt8Type"
        " or self::Int16Type"
        " or self::UInt16Type"
        " or self::Int32Type"
        " or self::UInt32Type"
        " or self::Int64Type"
        " or self::UInt64Type"
        " or self::EnumerationType"
        " or self::BooleanType"
        " or self::StringType"
        " or self::BinaryType"
        " or self::ClockType]",
        xpathCtx);

    if (xpathObj->nodesetval) {

        modelDescription->nTypeDefinitions = xpathObj->nodesetval->nodeNr;
        
        CALL(FMICalloc((void**)&modelDescription->typeDefinitions, xpathObj->nodesetval->nodeNr, sizeof(FMITypeDefinition*)));

        for (size_t i = 0; i < xpathObj->nodesetval->nodeNr; i++) {

            FMITypeDefinition* typeDefinition = NULL;
            
            CALL(FMICalloc((void**)&typeDefinition, 1, sizeof(FMITypeDefinition)));

            modelDescription->typeDefinitions[i] = typeDefinition;

            xmlNodePtr typeDefinitionNode = xpathObj->nodesetval->nodeTab[i];

            // TODO: type
            typeDefinition->name = (char*)xmlGetProp(typeDefinitionNode, (xmlChar*)"name");
            typeDefinition->quantity = (char*)xmlGetProp(typeDefinitionNode, (xmlChar*)"quantity");

            const char* unit = (char*)xmlGetProp(typeDefinitionNode, (xmlChar*)"unit");

            if (unit) {
                typeDefinition->unit = FMIUnitForName(modelDescription, unit);
                if (!typeDefinition->unit) {
                    FMILogError("Unit \"%s\" of type defintion \"%s\" is not defined.", unit, typeDefinition->name);
                    nProblems++;
                }
                xmlFree((void*)unit);
            }

            const char* displayUnit = (char*)xmlGetProp(typeDefinitionNode, (xmlChar*)"displayUnit");

            if (displayUnit) {
                typeDefinition->displayUnit = FMIDisplayUnitForName(typeDefinition->unit, displayUnit);
                if (!typeDefinition->displayUnit) {
                    FMILogError("Display unit \"%s\" of type defintion \"%s\" is not defined.", displayUnit, typeDefinition->name);
                    nProblems++;
                }
                xmlFree((void*)displayUnit);
            }

            typeDefinition->relativeQuantity = getBooleanAttribute(typeDefinitionNode, "relativeQuantity");
            typeDefinition->min              = (char*)xmlGetProp(typeDefinitionNode, (xmlChar*)"min");
            typeDefinition->max              = (char*)xmlGetProp(typeDefinitionNode, (xmlChar*)"max");
            typeDefinition->nominal          = (char*)xmlGetProp(typeDefinitionNode, (xmlChar*)"nominal");
            typeDefinition->unbounded        = getBooleanAttribute(typeDefinitionNode, "unbounded");
        }
    }

    // model variables
    xpathObj = xmlXPathEvalExpression((xmlChar*)"/fmiModelDescription/ModelVariables/"
        "*[self::Float32"
        " or self::Float64"
        " or self::Int8"
        " or self::UInt8"
        " or self::Int16"
        " or self::UInt16"
        " or self::Int32"
        " or self::UInt32"
        " or self::Int64"
        " or self::UInt64"
        " or self::Enumeration"
        " or self::Boolean"
        " or self::String"
        " or self::Binary"
        " or self::Clock]", 
        xpathCtx);

    modelDescription->nModelVariables = xpathObj->nodesetval->nodeNr;

    CALL(FMICalloc((void**)&modelDescription->modelVariables, xpathObj->nodesetval->nodeNr, sizeof(FMIModelVariable*)));

    for (size_t i = 0; i < xpathObj->nodesetval->nodeNr; i++) {

        FMIModelVariable* variable;

        CALL(FMICalloc((void**)&variable, 1, sizeof(FMIModelVariable)));

        modelDescription->modelVariables[i] = variable;

        xmlNodePtr variableNode = xpathObj->nodesetval->nodeTab[i];

        variable->line        = variableNode->line;
        variable->name        = (char*)xmlGetProp(variableNode, (xmlChar*)"name");
        variable->min         = (char*)xmlGetProp(variableNode, (xmlChar*)"min");
        variable->max         = (char*)xmlGetProp(variableNode, (xmlChar*)"max");
        variable->nominal     = (char*)xmlGetProp(variableNode, (xmlChar*)"nominal");
        variable->start       = (char*)xmlGetProp(variableNode, (xmlChar*)"start");
        variable->description = (char*)xmlGetProp(variableNode, (xmlChar*)"description");

        const char* unit = (char*)xmlGetProp(variableNode, (xmlChar*)"unit");        
        if (unit) {
            variable->unit = FMIUnitForName(modelDescription, unit);
            if (!variable->unit) {
                FMILogError("Unit \"%s\" of model variable \"%s\" (line %hu) is not defined.", unit, variable->name, variable->line);
                nProblems++;
            }
            xmlFree((void*)unit);
        }

        const char* declaredType = (char*)xmlGetProp(variableNode, (xmlChar*)"declaredType");

        if (declaredType) {
            variable->declaredType = FMITypeDefintionForName(modelDescription, declaredType);
            if (!variable->declaredType) {
                FMILogError("Declared type \"%s\" of model variable \"%s\" (line %hu) is not defined.", declaredType, variable->name, variable->line);
                nProblems++;
            }
            xmlFree((void*)declaredType);
        }

        FMIVariableType type;
        
        const char* name = (char*)variableNode->name;

        const char* variability = (char*)xmlGetProp(variableNode, (xmlChar*)"variability");

        if (!variability) {
            variable->variability = -1;  // infer from type
        } else if (!strcmp(variability, "constant")) {
            variable->variability = FMIConstant;
        } else if (!strcmp(variability, "fixed")) {
            variable->variability = FMIFixed;
        } else if (!strcmp(variability, "tunable")) {
            variable->variability = FMITunable;
        } else if (!strcmp(variability, "discrete")) {
            variable->variability = FMIDiscrete;
        } else {
            variable->variability = FMIContinuous;
        }

        xmlFree((void*)variability);

        if (!strcmp(name, "Float32")) {
            type = FMIFloat32Type;
        } else if (!strcmp(name, "Float64")) {
            type = FMIFloat64Type;
        } else if (!strcmp(name, "Int8")) {
            type = FMIInt8Type;
        } else if (!strcmp(name, "UInt8")) {
            type = FMIUInt8Type;
        } else if (!strcmp(name, "Int16")) {
            type = FMIInt16Type;
        } else if (!strcmp(name, "UInt16")) {
            type = FMIUInt16Type;
        } else if (!strcmp(name, "Int32")) {
            type = FMIInt32Type;
        } else if (!strcmp(name, "UInt32")) {
            type = FMIUInt32Type;
        } else if (!strcmp(name, "Int64") || !strcmp(name, "Enumeration")) {
            type = FMIInt64Type;
        } else if (!strcmp(name, "UInt64")) {
            type = FMIUInt64Type;
        } else if (!strcmp(name, "Boolean")) {
            type = FMIBooleanType;
        } else if (!strcmp(name, "String")) {
            type = FMIStringType;
        } else if (!strcmp(name, "Binary")) {
            type = FMIBinaryType;
        } else if (!strcmp(name, "Clock")) {
            type = FMIClockType;
        } else {
            return NULL;
        }

        variable->type = type;

        if (variable->variability == -1) {
            switch (variable->type) {
            case FMIFloat32Type:
            case FMIFloat64Type:
                variable->variability = FMIContinuous;
                break;
            default:
                variable->variability = FMIDiscrete;
                break;
            }
        }

        const char* vr = (char*)xmlGetProp(variableNode, (xmlChar*)"valueReference");

        variable->valueReference = FMIValueReferenceForLiteral(vr);

        xmlFree((void*)vr);

        const char* causality = (char*)xmlGetProp(variableNode, (xmlChar*)"causality");

        if (!causality) {
            variable->causality = FMILocal;
        } else if (!strcmp(causality, "parameter")) {
            variable->causality = FMIParameter;
        } else if (!strcmp(causality, "calculatedParameter")) {
            variable->causality = FMICalculatedParameter;
        } else if (!strcmp(causality, "structuralParameter")) {
            variable->causality = FMIStructuralParameter;
        } else if (!strcmp(causality, "input")) {
            variable->causality = FMIInput;
        } else if (!strcmp(causality, "output")) {
            variable->causality = FMIOutput;
        } else if (!strcmp(causality, "independent")) {
            variable->causality = FMIIndependent;
        } else {
            variable->causality = FMILocal;
        }

        xmlFree((void*)causality);

        variable->derivative = (FMIModelVariable*)xmlGetProp(variableNode, (xmlChar*)"derivative");

        xmlXPathObjectPtr xpathObj2 = xmlXPathNodeEval(variableNode, (xmlChar*)".//Dimension", xpathCtx);

        for (size_t j = 0; j < xpathObj2->nodesetval->nodeNr; j++) {

            const xmlNodePtr dimensionNode = xpathObj2->nodesetval->nodeTab[j];

            const char* start = (char*)xmlGetProp(dimensionNode, (xmlChar*)"start");
            const char* valueReference = (char*)xmlGetProp(dimensionNode, (xmlChar*)"valueReference");

            variable->dimensions = realloc(variable->dimensions, (variable->nDimensions + 1) * sizeof(FMIDimension));

            if (!variable->dimensions) {
                return NULL;
            }

            FMIDimension* dimension = &variable->dimensions[variable->nDimensions];

            dimension->start = 0;
            dimension->variable = NULL;

            if (start) {
                dimension->start = atoi(start);
            } else if (valueReference) {
                const FMIValueReference vr = atoi(valueReference);
                dimension->variable = FMIModelVariableForValueReference(modelDescription, vr);
            } else {
                FMILogError("Dimension must have start or valueReference.\n");
                return NULL;
            }

            variable->nDimensions++;
        }

        xmlXPathFreeObject(xpathObj2);
    }

    xmlXPathFreeObject(xpathObj);

    CALL(readUnknownsFMI3(xpathCtx, modelDescription, "/fmiModelDescription/ModelStructure/Output", &modelDescription->nOutputs, &modelDescription->outputs));
    CALL(readUnknownsFMI3(xpathCtx, modelDescription, "/fmiModelDescription/ModelStructure/ContinuousStateDerivative", &modelDescription->nContinuousStates, &modelDescription->derivatives));
    CALL(readUnknownsFMI3(xpathCtx, modelDescription, "/fmiModelDescription/ModelStructure/InitialUnknown", &modelDescription->nInitialUnknowns, &modelDescription->initialUnknowns));
    CALL(readUnknownsFMI3(xpathCtx, modelDescription, "/fmiModelDescription/ModelStructure/EventIndicator", &modelDescription->nEventIndicators, &modelDescription->eventIndicators));

    xmlXPathFreeContext(xpathCtx);

    // check variabilities
    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {

        FMIModelVariable* variable = modelDescription->modelVariables[i];
        
        if (variable->type != FMIFloat32Type && variable->type != FMIFloat64Type && variable->variability == FMIContinuous) {
            FMILogError("Variable \"%s\" is not of type Float{32|64} but has variability = continuous.\n", variable->name);
            nProblems++;
        }
    }

    // resolve derivatives
    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {

        FMIModelVariable* variable = modelDescription->modelVariables[i];
        
        if (variable->derivative) {
            char* literal = (char*)variable->derivative;
            const FMIValueReference vr = FMIValueReferenceForLiteral(literal);
            variable->derivative = FMIModelVariableForValueReference(modelDescription, vr);
            if (!variable->derivative) {
                nProblems++;
                FMILogError("Failed to resolve attribute derivative=\"%s\" for model variable \"%s\".\n", literal, variable->name);
            }
            xmlFree(literal);
        }

    }

    nProblems += FMIValidateModelDescription(modelDescription);

    if (nProblems > 0) {
        FMIFreeModelDescription(modelDescription);
        modelDescription = NULL;
    }

TERMINATE:

    // TODO

    return modelDescription;
}

static void logSchemaValidationError(void* ctx, const char* msg, ...) {
    (void)ctx; // unused
    va_list args;
    va_start(args, msg);
    logErrorMessage(msg, args);
    va_end(args);
}

FMIModelDescription* FMIReadModelDescription(const char* filename) {

    // TODO: add stream interface (see https://gitlab.gnome.org/GNOME/libxml2/-/wikis/Parser-interfaces)

    xmlDocPtr doc = NULL;
    xmlNodePtr root = NULL;
    xmlSchemaPtr schema = NULL;
    xmlSchemaParserCtxtPtr pctxt = NULL;
    xmlSchemaValidCtxtPtr vctxt = NULL;
    FMIModelDescription* modelDescription = NULL;
    const char* fmiVersion = NULL;
    FMIMajorVersion fmiMajorVersion;

    doc = xmlParseFile(filename);

    // xmlKeepBlanksDefault(0);
    // xmlDocDump(stdout, doc);

    if (!doc) {
        FMILogError("Invalid XML.\n");
        goto TERMINATE;
    }

    root = xmlDocGetRootElement(doc);

    if (root == NULL) {
        FMILogError("Empty document\n");
        goto TERMINATE;
    }

    fmiVersion = (char*)xmlGetProp(root, (xmlChar*)"fmiVersion");

    if (!fmiVersion) {
        FMILogError("Attribute fmiVersion is missing.\n");
        goto TERMINATE;
    } else if (!strcmp(fmiVersion, "1.0")) {
        fmiMajorVersion = FMIMajorVersion1;
        pctxt = xmlSchemaNewMemParserCtxt((char*)fmi1Merged_xsd, fmi1Merged_xsd_len);
    } else if (!strcmp(fmiVersion, "2.0")) {
        fmiMajorVersion = FMIMajorVersion2;
        pctxt = xmlSchemaNewMemParserCtxt((char*)fmi2Merged_xsd, fmi2Merged_xsd_len);
    } else if(!strncmp(fmiVersion, "3.", 2)) {
        pctxt = xmlSchemaNewMemParserCtxt((char*)fmi3Merged_xsd, fmi3Merged_xsd_len);
        fmiMajorVersion = FMIMajorVersion3;
    } else {
        FMILogError("Unsupported FMI version: %s.\n", fmiVersion);
        goto TERMINATE;
    }

    schema = xmlSchemaParse(pctxt);

    if (schema == NULL) {
        goto TERMINATE;
    }

    vctxt = xmlSchemaNewValidCtxt(schema);

    if (!vctxt) {
        goto TERMINATE;
    }

    xmlSchemaSetValidErrors(vctxt, (xmlSchemaValidityErrorFunc)logSchemaValidationError, NULL, NULL);

    if (xmlSchemaValidateDoc(vctxt, doc)) {
        goto TERMINATE;
    }

    if (fmiMajorVersion == FMIMajorVersion1) {
        modelDescription = readModelDescriptionFMI1(root);
    } else if (fmiMajorVersion == FMIMajorVersion2) {
        modelDescription = readModelDescriptionFMI2(root);
    } else {
        modelDescription = readModelDescriptionFMI3(root);
    }

    if (modelDescription) {
        modelDescription->fmiVersion = fmiVersion;
    } else {
        xmlFree((void*)fmiVersion);
    }

TERMINATE:

    if (vctxt) {
        xmlSchemaFreeValidCtxt(vctxt);
    }
    
    if (schema) {
        xmlSchemaFree(schema);
    }

    if (pctxt) {
        xmlSchemaFreeParserCtxt(pctxt);
    }

    if (doc) {
        xmlFreeDoc(doc);
    }

    return modelDescription;
}

void FMIFreeModelDescription(FMIModelDescription* modelDescription) {

    if (!modelDescription) {
        return;
    }

    xmlFree((void*)modelDescription->fmiVersion);
    xmlFree((void*)modelDescription->modelName);
    xmlFree((void*)modelDescription->instantiationToken);
    xmlFree((void*)modelDescription->description);
    xmlFree((void*)modelDescription->generationTool);
    xmlFree((void*)modelDescription->generationDateAndTime);

    if (modelDescription->modelExchange) {
        xmlFree((void*)modelDescription->modelExchange->modelIdentifier);
        for (size_t i = 0; i < modelDescription->modelExchange->nSourceFiles; i++) {
            xmlFree((void*)modelDescription->modelExchange->sourceFiles[i]);
        }
        FMIFree((void**)&modelDescription->modelExchange->sourceFiles);
        FMIFree((void**)&modelDescription->modelExchange);
    }

    if (modelDescription->coSimulation) {
        xmlFree((void*)modelDescription->coSimulation->modelIdentifier);
        for (size_t i = 0; i < modelDescription->coSimulation->nSourceFiles; i++) {
            xmlFree((void*)modelDescription->coSimulation->sourceFiles[i]);
        }
        FMIFree((void**)&modelDescription->coSimulation->sourceFiles);
        FMIFree((void**)&modelDescription->coSimulation);
    }

    if (modelDescription->defaultExperiment) {
        xmlFree((void*)modelDescription->defaultExperiment->startTime);
        xmlFree((void*)modelDescription->defaultExperiment->stopTime);
        xmlFree((void*)modelDescription->defaultExperiment->stepSize);
        FMIFree((void**)&modelDescription->defaultExperiment);
    }

    // units
    for (size_t i = 0; i < modelDescription->nUnits; i++) {

        FMIUnit* unit = modelDescription->units[i];
        
        if (unit) {

            xmlFree((void*)unit->name);
            FMIFree((void**)&unit->baseUnit);

            for (size_t j = 0; j < unit->nDisplayUnits; j++) {
                FMIDisplayUnit* displayUnit = unit->displayUnits[j];
                xmlFree((void*)displayUnit->name);
                FMIFree((void**)&displayUnit);
            }

            FMIFree((void**)&unit);
        }
    }

    FMIFree((void**)&modelDescription->units);

    // type defintions
    for (size_t i = 0; i < modelDescription->nTypeDefinitions; i++) {

        FMITypeDefinition* typeDefintion = modelDescription->typeDefinitions[i];

        if (typeDefintion) {

            xmlFree((void*)typeDefintion->name);
            xmlFree((void*)typeDefintion->quantity);
            xmlFree((void*)typeDefintion->min);
            xmlFree((void*)typeDefintion->max);
            xmlFree((void*)typeDefintion->nominal);

            FMIFree((void**)&typeDefintion);
        }
    }

    FMIFree((void**)&modelDescription->typeDefinitions);

    // model variables
    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {

        FMIModelVariable* variable = modelDescription->modelVariables[i];

        if (variable) {
            xmlFree((void*)variable->name);
            xmlFree((void*)variable->min);
            xmlFree((void*)variable->max);
            xmlFree((void*)variable->nominal);
            xmlFree((void*)variable->start);
            xmlFree((void*)variable->description);

            FMIFree(&variable);
        }
    }

    FMIFree((void**)&modelDescription->modelVariables);

    FMIFree((void**)&modelDescription);
}

FMIValueReference FMIValueReferenceForLiteral(const char* literal) {
    return (FMIValueReference)strtoul(literal, NULL, 0);
}

FMIUnit* FMIUnitForName(const FMIModelDescription* modelDescription, const char* name) {

    if (!modelDescription || !name) {
        return NULL;
    }

    for (size_t i = 0; i < modelDescription->nUnits; i++) {

        FMIUnit* unit = modelDescription->units[i];

        if (!strcmp(unit->name, name)) {
            return unit;
        }
    }

    return NULL;
}

FMIDisplayUnit* FMIDisplayUnitForName(const FMIUnit* unit, const char* name) {

    if (!unit || !name) {
        return NULL;
    }

    for (size_t i = 0; i < unit->nDisplayUnits; i++) {

        FMIDisplayUnit* displayUnit = unit->displayUnits[i];

        if (!strcmp(displayUnit->name, name)) {
            return displayUnit;
        }
    }

    return NULL;
}

FMITypeDefinition* FMITypeDefintionForName(const FMIModelDescription* modelDescription, const char* name) {

    if (!modelDescription || !name) {
        return NULL;
    }

    for (size_t i = 0; i < modelDescription->nTypeDefinitions; i++) {

        FMITypeDefinition* typeDefintion = modelDescription->typeDefinitions[i];

        if (!strcmp(typeDefintion->name, name)) {
            return typeDefintion;
        }
    }

    return NULL;
}

FMIModelVariable* FMIModelVariableForName(const FMIModelDescription* modelDescription, const char* name) {

    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {

        FMIModelVariable* variable = modelDescription->modelVariables[i];
        
        if (!strcmp(variable->name, name)) {
            return variable;
        }
    }

    return NULL;
}

FMIModelVariable* FMIModelVariableForValueReference(const FMIModelDescription* modelDescription, FMIValueReference valueReference) {

    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {

        FMIModelVariable* variable = modelDescription->modelVariables[i];

        if (variable->valueReference == valueReference) {
            return variable;
        }
    }

    return NULL;
}

FMIModelVariable* FMIModelVariableForIndexLiteral(const FMIModelDescription* modelDescription, const char* index) {

    const size_t i = strtoul(index, NULL, 0);

    if (i == 0 || i > modelDescription->nModelVariables) {
        return NULL;
    }

    return modelDescription->modelVariables[i - 1];
}

static bool isLegalCombination(FMIModelVariable* variable) {

    if (variable->causality == FMIStructuralParameter || variable->causality == FMIParameter) {
        
        if (variable->variability == FMIFixed || variable->variability == FMITunable) {

            if (variable->initial == FMIUndefined) {
                variable->initial = FMIExact;
            }
        
            if (variable->initial == FMIExact) {
                return true;
            }

        }

    } else if (variable->causality == FMICalculatedParameter) {
        
        if (variable->variability == FMIFixed || variable->variability == FMITunable) {

            if (variable->initial == FMIUndefined) {
                variable->initial = FMICalculated;
            }
        
            if (variable->initial == FMICalculated || variable->initial == FMIApprox) {
                return true;
            }

        }

    } else if (variable->causality == FMIInput) {
        
        if (variable->variability == FMIDiscrete || variable->variability == FMIContinuous) {
        
            if (variable->initial == FMIUndefined) {
                variable->initial = FMIExact;
            }

            if (variable->initial == FMIExact) {
                return true;
            }

        }

    } else if (variable->causality == FMIOutput) {

        if (variable->variability == FMIConstant) {

            if (variable->initial == FMIUndefined) {
                variable->initial = FMIExact;
            }

            if (variable->initial == FMIExact) {
                return true;
            }

        } else if (variable->variability == FMIDiscrete || variable->variability == FMIContinuous) {

            if (variable->initial == FMIUndefined) {
                variable->initial = FMICalculated;
            }

            if (variable->initial == FMICalculated || variable->initial == FMIExact || variable->initial == FMIApprox) {
                return true;
            }

        }

    } else if (variable->causality == FMILocal) {

        if (variable->variability == FMIConstant) {

            if (variable->initial == FMIUndefined) {
                variable->initial = FMIExact;
            }

            if (variable->initial == FMIExact) {
                return true;
            }

        } else if (variable->variability == FMIFixed || variable->variability == FMITunable) {

            if (variable->initial == FMIUndefined) {
                variable->initial = FMICalculated;
            }

            if (variable->initial == FMICalculated || variable->initial == FMIApprox) {
                return true;
            }

        } else if (variable->variability == FMIDiscrete || variable->variability == FMIContinuous) {

            if (variable->initial == FMIUndefined) {
                variable->initial = FMICalculated;
            }

            if (variable->initial == FMICalculated || variable->initial == FMIExact || variable->initial == FMIApprox) {
                return true;
            }

        }

    } else if (variable->causality == FMIIndependent) {

        if (variable->variability == FMIContinuous) {

            if (variable->initial == FMIUndefined) {
                return true;
            }

        }

    }

    return false;
}

void set_input_string(const char* in);

void end_lexical_scan(void);

void yyerror(const FMIModelVariable* variable, const char* s) {
    FMILogError("\"%s\" (line %d) is not a valid variable name for variableNamingConvention=\"structured\".\n", variable->name, variable->line);
}

size_t FMIValidateModelDescription(const FMIModelDescription* modelDescription) {

    size_t nProblems = 0;

    // validate structured variable names
    if (modelDescription->variableNamingConvention == FMIStructured) {

        for (size_t i = 0; i < modelDescription->nModelVariables; i++) {

            const FMIModelVariable* variable = modelDescription->modelVariables[i];

            set_input_string(variable->name);

            if (yyparse((void*)variable)) {
                nProblems++;
            }

            end_lexical_scan();
        }
    }

    // check combinations of causality, variability, and initial
    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {

        FMIModelVariable* variable = modelDescription->modelVariables[i];

        if (!isLegalCombination(variable)) {
            nProblems++;
            FMILogError("The variable \"%s\" has an illegal combination of causality, variability, and initial.\n", variable->name);
        }
    }

    size_t nOutputs = 0;

    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {

        FMIModelVariable* variable = modelDescription->modelVariables[i];

        if (variable->causality == FMIOutput) {
            nOutputs++;
        }
    }

    if (nOutputs != modelDescription->nOutputs) {
        nProblems++;
        FMILogError("The number of model variables with causality=\"output\" (%zu) must match the number of outputs"
            " in the model structure (%zu).\n", nOutputs, modelDescription->nContinuousStates);
    }

    return nProblems;
}

void FMIDumpModelDescription(FMIModelDescription* modelDescription, FILE* file) {

    fprintf(file, "FMI Version        3.0\n");
    fprintf(file, "FMI Type           Co-Simulation\n");
    fprintf(file, "Model Name         %s\n", modelDescription->modelName);
    fprintf(file, "Description        %s\n", modelDescription->description ? modelDescription->description : "n/a");
    fprintf(file, "Continuous States  %zu\n", modelDescription->nContinuousStates);
    fprintf(file, "Event Indicators   %zu\n", modelDescription->nEventIndicators);
    fprintf(file, "Generation Tool    %s\n", modelDescription->generationTool ? modelDescription->generationTool : "n/a");
    fprintf(file, "Generation Date    %s\n", modelDescription->generationDateAndTime ? modelDescription->generationDateAndTime : "n/a");
    fprintf(file, "\n");
    fprintf(file, "Model Variables\n");
    fprintf(file, "\n");
    fprintf(file, "Name                           Description\n");
    fprintf(file, "\n");
    for (size_t i = 0; i < modelDescription->nModelVariables; i++) {
        fprintf(file, "%-30s %s\n", modelDescription->modelVariables[i]->name, modelDescription->modelVariables[i]->description);
    }

}
