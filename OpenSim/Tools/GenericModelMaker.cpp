/* -------------------------------------------------------------------------- *
 *                      OpenSim:  GenericModelMaker.cpp                       *
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
#include "GenericModelMaker.h"

#include <memory>

#include <OpenSim/Simulation/Model/Model.h>

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
GenericModelMaker::GenericModelMaker() { constructProperties(); }

//_____________________________________________________________________________
/**
 * Destructor.
 */
GenericModelMaker::~GenericModelMaker()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void GenericModelMaker::constructProperties() {
    constructProperty_model_file("");
    constructProperty_marker_set_file("");
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void GenericModelMaker::registerTypes()
{
    //Object::registerType(Marker());
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Execute the model making process, which involves reading
 * an XML model file and possible updating its marker set.
 *
 * @return Pointer to the Model that is constructed.
 */
Model* GenericModelMaker::processModel(const string& aPathToSubject) const {
    log_info("Step 1: Loading generic model");
    try
    {
        std::string modelPath = SimTK::Pathname::
                getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                        aPathToSubject, get_model_file());
        auto model = std::make_unique<Model>(get_model_file());
        model->initSystem();

        if (!getProperty_marker_set_file().getValueIsDefault() &&
                get_marker_set_file() != "Unassigned") {
            std::string markerSetPath = SimTK::Pathname::
                    getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                            aPathToSubject, get_marker_set_file());
            log_info("Loading marker set from '{}'.", markerSetPath);
            MarkerSet markerSet = MarkerSet(markerSetPath);
            model->updateMarkerSet(markerSet);
        }
        return model.release();
    }
    catch (const Exception& x)
    {
        log_error(x.what());
        return nullptr;
    }
}
