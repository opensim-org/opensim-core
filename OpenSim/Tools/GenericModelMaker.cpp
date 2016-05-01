/* -------------------------------------------------------------------------- *
 *                      OpenSim:  GenericModelMaker.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Marker.h>

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
GenericModelMaker::GenericModelMaker()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Set the data members of this GenericModelMaker to their null values.
 */
void GenericModelMaker::setNull()
{
}

void GenericModelMaker::constructProperties()
{
    // These default values are to be backwards-compatible with the previous
    // use of PropertyStr classes, whose default value was the same.
    constructProperty_model_file("Unassigned");
    constructProperty_marker_set_file("Unassigned");
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
Model* GenericModelMaker::processModel(const string& aPathToSubject)
{
    Model* model = NULL;

    cout << endl << "Step 1: Loading generic model" << endl;

    try
    {
        set_model_file(aPathToSubject + get_model_file());

        model = new Model(get_model_file());
        model->initSystem();

        if (!getProperty_marker_set_file().getValueIsDefault()
                && get_marker_set_file() != "Unassigned") {
            cout << "Loading marker set from '" <<
                aPathToSubject + get_marker_set_file() +"'" << endl;
            MarkerSet *markerSet = new MarkerSet(*model,
                    aPathToSubject + get_marker_set_file());
            model->updateMarkerSet(*markerSet);
        }
    }
    catch (const Exception& x)
    {
        x.print(cout);
        return NULL;
    }

    return model;
}
