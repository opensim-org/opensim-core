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
    FrameGeometry frm(0.2);
    frm.setName("frame_geometry");
    frm.set_display_radius(.004);
    frm.setRepresentation(Geometry::Hide);
    append_geometry(frm);

}

void PhysicalFrame::constructProperties()
{
    constructProperty_WrapObjectSet(WrapObjectSet());
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
    return SimTK::Transform();
}

void PhysicalFrame::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    // Better use name search or more robust method
    if (upd_geometry(0).getFrameName() == "")
        upd_geometry(0).setFrameName(getName());

    for (int i = 0; i < get_WrapObjectSet().getSize(); i++)
        get_WrapObjectSet().get(i).connectToModelAndBody(aModel, *this);
}

const WrapObject* PhysicalFrame::getWrapObject(const string& aName) const
{
    int i;

    for (i = 0; i < get_WrapObjectSet().getSize(); i++) {
        if (aName == get_WrapObjectSet()[i].getName())
            return &get_WrapObjectSet()[i];
    }
    return nullptr;
}

void PhysicalFrame::addWrapObject(WrapObject* wrap) {
    upd_WrapObjectSet().adoptAndAppend(wrap);
}

void PhysicalFrame::scale(const SimTK::Vec3& aScaleFactors)
{
    // Base class, to scale wrap objects
    for (int i = 0; i< get_WrapObjectSet().getSize(); i++)
        upd_WrapObjectSet().get(i).scale(aScaleFactors);

    SimTK::Vec3 oldScaleFactors;
    //NewGeometry getDisplayer()->getScaleFactors(oldScaleFactors);

    for (int i = 0; i<3; i++) {
        oldScaleFactors[i] *= aScaleFactors[i];
    }
    // TODO: When we revamp scaling and solve the issue with where scale factors
    // are maintained, we need to fix this or remove it completely -Ayman 5/15
    // Update scale factors for displayer
    //NewGeometry updDisplayer()->setScaleFactors(oldScaleFactors);
}
