/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ScaleTool.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include "ScaleTool.h"
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "GenericModelMaker.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ScaleTool::ScaleTool() :
    _mass(_massProp.getValueDbl()),
    _height(_heightProp.getValueDbl()),
    _age(_ageProp.getValueDbl()),
    _notes(_notesProp.getValueStr()),
   _genericModelMakerProp(PropertyObj("", GenericModelMaker())),
    _genericModelMaker((GenericModelMaker&)_genericModelMakerProp.getValueObj()),
   _modelScalerProp(PropertyObj("", ModelScaler())),
    _modelScaler((ModelScaler&)_modelScalerProp.getValueObj()),
   _markerPlacerProp(PropertyObj("", MarkerPlacer())),
    _markerPlacer((MarkerPlacer&)_markerPlacerProp.getValueObj())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML file
 */
ScaleTool::ScaleTool(const string &aFileName) :
   Object(aFileName, true),
    _mass(_massProp.getValueDbl()),
    _height(_heightProp.getValueDbl()),
    _age(_ageProp.getValueDbl()),
    _notes(_notesProp.getValueStr()),
   _genericModelMakerProp(PropertyObj("", GenericModelMaker())),
    _genericModelMaker((GenericModelMaker&)_genericModelMakerProp.getValueObj()),
   _modelScalerProp(PropertyObj("", ModelScaler())),
    _modelScaler((ModelScaler&)_modelScalerProp.getValueObj()),
   _markerPlacerProp(PropertyObj("", MarkerPlacer())),
    _markerPlacer((MarkerPlacer&)_markerPlacerProp.getValueObj())
{
    setNull();
    setupProperties();

    updateFromXMLDocument();

    _pathToSubject = IO::getParentDirectory(aFileName);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ScaleTool::~ScaleTool()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSubject ScaleTool to be copied.
 */
ScaleTool::ScaleTool(const ScaleTool &aSubject) :
   Object(aSubject),
    _mass(_massProp.getValueDbl()),
    _height(_heightProp.getValueDbl()),
    _age(_ageProp.getValueDbl()),
    _notes(_notesProp.getValueStr()),
   _genericModelMakerProp(PropertyObj("", GenericModelMaker())),
    _genericModelMaker((GenericModelMaker&)_genericModelMakerProp.getValueObj()),
   _modelScalerProp(PropertyObj("", ModelScaler())),
    _modelScaler((ModelScaler&)_modelScalerProp.getValueObj()),
   _markerPlacerProp(PropertyObj("", MarkerPlacer())),
    _markerPlacer((MarkerPlacer&)_markerPlacerProp.getValueObj())
{
    setNull();
    setupProperties();
    copyData(aSubject);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one ScaleTool to another.
 *
 * @param aSubject ScaleTool to be copied.
 */
void ScaleTool::copyData(const ScaleTool &aSubject)
{
    _mass = aSubject._mass;
    _height = aSubject._height;
    _age = aSubject._age;
    _notes = aSubject._notes;
    _genericModelMaker = aSubject._genericModelMaker;
    _modelScaler = aSubject._modelScaler;
    _markerPlacer = aSubject._markerPlacer;
}

//_____________________________________________________________________________
/**
 * Set the data members of this ScaleTool to their null values.
 */
void ScaleTool::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ScaleTool::setupProperties()
{
    _massProp.setComment("Mass of the subject in kg.  Subject-specific model generated by scaling step will have this total mass.");
    _massProp.setName("mass");
    _massProp.setValue(-1.0);
    _propertySet.append(&_massProp);

    _heightProp.setComment("Height of the subject in mm.  For informational purposes only (not used by scaling).");
    _heightProp.setName("height");
    _heightProp.setValue(-1.0);
    _propertySet.append(&_heightProp);

    _ageProp.setComment("Age of the subject in years.  For informational purposes only (not used by scaling).");
    _ageProp.setName("age");
    _ageProp.setValue(-1.0);
    _propertySet.append(&_ageProp);

    _notesProp.setComment("Notes for the subject.");
    _notesProp.setName("notes");
    _propertySet.append(&_notesProp);

    _genericModelMakerProp.setComment("Specifies the name of the unscaled model (.osim) and the marker set.");
    _genericModelMakerProp.setName("GenericModelMaker");
    _propertySet.append(&_genericModelMakerProp);

    _modelScalerProp.setComment("Specifies parameters for scaling the model.");
    _modelScalerProp.setName("ModelScaler");
    _propertySet.append(&_modelScalerProp);

    _markerPlacerProp.setComment("Specifies parameters for placing markers on the model once a model is scaled. ");
    _markerPlacerProp.setName("MarkerPlacer");
    _propertySet.append(&_markerPlacerProp);
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void ScaleTool::registerTypes()
{
    Object::registerType(GenericModelMaker());
    Object::registerType(ModelScaler());
    Object::registerType(MarkerPlacer());
    GenericModelMaker::registerTypes();
    ModelScaler::registerTypes();
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
ScaleTool& ScaleTool::operator=(const ScaleTool &aSubject)
{
    // BASE CLASS
    Object::operator=(aSubject);

    copyData(aSubject);

    return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Create a generic model, using GenericModelMaker::processModel().
 *
 * @return Pointer to the Model that is created.
 */
Model* ScaleTool::createModel() const
{
    log_info("Processing subject {}...", getName());

    /* Make the generic model. */
    if (!_genericModelMakerProp.getValueIsDefault())
    {
        Model *model = getGenericModelMaker().processModel(_pathToSubject);
        if (!model)
        {
            // processModel() attempts to load both the model and market set
            // file. _pathToSubject might be misleading of model path was
            // given as an aboslute path in the setup file, so it was
            // removed from this error message.
            log_error("Unable to load the generic model or marker set file.");
            return 0;
        }
        else {
            model->setName(getName());
            return model;
        }
    } else {
        log_warn("ScaleTool::createModel: Unscaled model not specified ({} "  
            "section missing from setup file).", 
            _genericModelMakerProp.getName());
    }
    return 0;
}

bool ScaleTool::run() const {
    std::unique_ptr<Model> model(createModel());

    if(model == nullptr) { 
        string msg = "ScaleTool: No model specified.";
        log_error(msg);
        throw Exception(msg, __FILE__, __LINE__);
    }

    if (!isDefaultModelScaler() && getModelScaler().getApply())
    {
        const ModelScaler& scaler = getModelScaler();
        if(!scaler.processModel(model.get(), getPathToSubject(), getSubjectMass())) {
            return false;
        }
    }
    else
    {
        log_error("Scaling parameters disabled (apply is false) or not set. "
            "Model is not scaled.");
    }

    if (!isDefaultMarkerPlacer())
    {
        const MarkerPlacer& placer = getMarkerPlacer();
        if(!placer.processModel(model.get(), getPathToSubject())) {
            return false;
        }
    }
    else
    {
        log_error("Marker placement parameters disabled (apply is false) or "
            "not set. No markers have been moved.");
    }
    return true;
}
