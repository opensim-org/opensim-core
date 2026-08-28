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

#include "GenericModelMaker.h"
#include "OpenSim/Common/IO.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Tools/MarkerPlacer.h"
#include "OpenSim/Tools/ModelScaler.h"

#include <memory>

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
ScaleTool::ScaleTool() { constructProperties(); }

//_____________________________________________________________________________
/**
 * Constructor from an XML file
 */
ScaleTool::ScaleTool(const string& aFileName) : Object(aFileName, true) {
    constructProperties();
    updateFromXMLDocument();

    set_path_to_subject(IO::getParentDirectory(aFileName));
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ScaleTool::constructProperties() {
    constructProperty_mass(-1.0);
    constructProperty_height(-1.0);
    constructProperty_age(-1.0);
    constructProperty_notes("");
    constructProperty_path_to_subject("");

    constructProperty_generic_model_maker(GenericModelMaker());
    constructProperty_model_scaler(ModelScaler());
    constructProperty_marker_placer(MarkerPlacer());
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void ScaleTool::registerTypes() {
    Object::registerType(GenericModelMaker());
    Object::registerType(ModelScaler());
    Object::registerType(MarkerPlacer());
    GenericModelMaker::registerTypes();
    ModelScaler::registerTypes();
    MarkerPlacer::registerTypes();
}

void ScaleTool::setPrintResultFiles(bool aToWrite) {
    upd_model_scaler().setPrintResultFiles(aToWrite);
    upd_marker_placer().setPrintResultFiles(aToWrite);
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
Model* ScaleTool::createModel() const {
    log_info("Processing subject {}...", getName());

    if (isDefaultGenericModelMaker()) {
        log_warn("ScaleTool::createModel: Unscaled model not specified ({} "
                 "section missing from setup file).",
                get_generic_model_maker().getName());
        return nullptr;
    }
    auto* model = getGenericModelMaker().processModel(get_path_to_subject());

    if (!model) {
        // processModel() attempts to load both the model and market set
        // file. _pathToSubject might be misleading if model path was
        // given as an aboslute path in the setup file, so it was
        // removed from this error message.
        log_error("Unable to load the generic model or marker set file.");
    } else {
        model->setName(getName());
    }

    return model;
}

bool ScaleTool::run() const {
    auto model = std::unique_ptr<Model>(createModel());

    if (!model) {
        string msg = "ScaleTool: No model specified.";
        log_error(msg);
        throw Exception(msg, __FILE__, __LINE__);
    }

    if (!isDefaultModelScaler() && getModelScaler().getApply()) {
        const auto& scaler = getModelScaler();
        if (!scaler.processModel(
                    model.get(), getPathToSubject(), getSubjectMass())) {
            return false;
        }
    } else {
        log_error("Scaling parameters disabled (apply is false) or not set. "
                  "Model is not scaled.");
    }

    if (!isDefaultMarkerPlacer()) {
        const auto& placer = getMarkerPlacer();
        if (!placer.processModel(model.get(), getPathToSubject())) {
            return false;
        }
    } else {
        log_error("Marker placement parameters disabled (apply is false) or "
                  "not set. No markers have been moved.");
    }
    return true;
}
