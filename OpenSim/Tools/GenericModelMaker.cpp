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
GenericModelMaker::GenericModelMaker() :
   _fileName(_fileNameProp.getValueStr()),
    _markerSetFileName(_markerSetFileNameProp.getValueStr())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
GenericModelMaker::~GenericModelMaker()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aGenericModelMaker GenericModelMaker to be copied.
 */
GenericModelMaker::GenericModelMaker(const GenericModelMaker &aGenericModelMaker) :
   Object(aGenericModelMaker),
   _fileName(_fileNameProp.getValueStr()),
    _markerSetFileName(_markerSetFileNameProp.getValueStr())
{
    setNull();
    setupProperties();
    copyData(aGenericModelMaker);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one GenericModelMaker to another.
 *
 * @param aGenericModelMaker GenericModelMaker to be copied.
 */
void GenericModelMaker::copyData(const GenericModelMaker &aGenericModelMaker)
{
    _fileName = aGenericModelMaker._fileName;
    _markerSetFileName = aGenericModelMaker._markerSetFileName;
}

//_____________________________________________________________________________
/**
 * Set the data members of this GenericModelMaker to their null values.
 */
void GenericModelMaker::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void GenericModelMaker::setupProperties()
{
    _fileNameProp.setComment("Model file (.osim) for the unscaled model."); 
    _fileNameProp.setName("model_file");
    _propertySet.append(&_fileNameProp);

    _markerSetFileNameProp.setComment("Set of model markers used to scale the model. "
        "Scaling is done based on distances between model markers compared to "
        "the same distances between the corresponding experimental markers.");
    _markerSetFileNameProp.setName("marker_set_file");
    _propertySet.append(&_markerSetFileNameProp);
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
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
GenericModelMaker& GenericModelMaker::operator=(const GenericModelMaker &aGenericModelMaker)
{
    // BASE CLASS
    Object::operator=(aGenericModelMaker);

    copyData(aGenericModelMaker);

    return(*this);
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
Model* GenericModelMaker::processModel(const string& aPathToSubject) const
{
    Model* model = NULL;

    log_info("Step 1: Loading generic model");

    try
    {
        std::string modelPath = 
            SimTK::Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(aPathToSubject, _fileName);
        model = new Model(modelPath);
        model->initSystem();

        if (!_markerSetFileNameProp.getValueIsDefault() && _markerSetFileName !="Unassigned") {
            log_info("Loading marker set from '{}'.", 
                aPathToSubject + _markerSetFileName);
            std::string markerSetPath = 
                SimTK::Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(aPathToSubject, _markerSetFileName);
            MarkerSet *markerSet = new MarkerSet(markerSetPath);
            model->updateMarkerSet(*markerSet);
        }
    }
    catch (const Exception& x)
    {
        log_error(x.what());
        return NULL;
    }

    return model;
}
