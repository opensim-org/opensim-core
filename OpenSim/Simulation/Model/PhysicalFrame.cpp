/* --------------------------------------------------------------------------*
*                         OpenSim:  PhysicalFrame.cpp                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2015 Stanford University and the Authors                *
* Author(s): Matt DeMers, Ayman Habib, Ajay Seth                             *
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
#include "PhysicalFrame.h"
#include "Model.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
PhysicalFrame::PhysicalFrame() : Frame()
{
    setAuthors("Matt DeMers, Ayman Habib, Ajay Seth");
    constructProperties();
}

void PhysicalFrame::constructProperties()
{
    constructProperty_VisibleObject(VisibleObject());
}

const SimTK::MobilizedBody& PhysicalFrame::getMobilizedBody() const
{
    return getModel().getMatterSubsystem().getMobilizedBody(_mbIndex);
}

SimTK::MobilizedBody& PhysicalFrame::updMobilizedBody() 
{
    return updModel().updMatterSubsystem().updMobilizedBody(_mbIndex);
}

/*
* Implementation of Frame interface by PhysicalFrame.
* 
*/
SimTK::Transform PhysicalFrame::
    calcGroundTransform(const SimTK::State& s) const
{
    // return X_GF = X_GB * X_BF;
    return getMobilizedBody().getBodyTransform(s);
}


SimTK::Transform PhysicalFrame::extendFindTransformInBaseFrame() const
{
    return Transform();
}

void PhysicalFrame::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    if (getName() == "ground"){
        setMobilizedBodyIndex(SimTK::GroundIndex);
    }
}

/*
* Add display geometry to a PhysicalFrame.
*
* @param aGeometryFileName Geometry filename.
*/
void PhysicalFrame::addDisplayGeometry(const std::string &aGeometryFileName)
{
    updDisplayer()->setGeometryFileName(updDisplayer()->getNumGeometryFiles(), aGeometryFileName);
}