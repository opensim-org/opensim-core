/* -------------------------------------------------------------------------- *
 *                            SimmFileWriter.cpp                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, P2C HD065690, U54 EB020405)   *
 * and by DARPA through the Warrior Web program.                              *
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
#include <sstream>
#include "SimmFileWriter.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "SimbodySimmModel.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmFileWriter::SimmFileWriter() :
    _model(NULL),
    _simbodySimmModel(NULL)
{
}

//_____________________________________________________________________________
/**
 * Constructor taking a model pointer
 */
SimmFileWriter::SimmFileWriter(const Model& aModel) :
    _model(NULL),
    _simbodySimmModel(NULL)
{
    _model = &aModel;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmFileWriter::~SimmFileWriter()
{
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Write a SIMM joint file.
 *
 * @param aFileName name of joint file to write.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeJointFile(const string& aFileName)
{
    if (!_model)
        return false;

    if (!_simbodySimmModel)
        _simbodySimmModel = new SimbodySimmModel(_model);

    return _simbodySimmModel->writeJointFile(aFileName);
}

//_____________________________________________________________________________
/**
 * Write a SIMM muscle file.
 *
 * @param aFileName name of muscle file to write.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeMuscleFile(const string& aFileName)
{
    if (!_model)
        return false;

    if (!_simbodySimmModel)
        _simbodySimmModel = new SimbodySimmModel(_model);

    return _simbodySimmModel->writeMuscleFile(aFileName);
}
