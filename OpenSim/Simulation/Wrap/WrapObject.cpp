/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WrapObject.cpp                          *
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
#include "WrapObject.h"
#include "WrapResult.h"
#include <OpenSim/Simulation/Model/PathPoint.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
/*
 * Default constructor.
 */
WrapObject::WrapObject() : ModelComponent()
{
    constructProperties();
}

/*
 * Destructor.
 */
WrapObject::~WrapObject()
{}


void WrapObject::constructProperties()
{
    constructProperty_active(true);
    const Vec3 defaultRotation(0.0);
    constructProperty_xyz_body_rotation(defaultRotation);
    const SimTK::Vec3 defaultTranslations(0.0);
    constructProperty_translation(defaultTranslations);

    constructProperty_quadrant("Unassigned");
    Appearance defaultAppearance;
    defaultAppearance.set_color(SimTK::Cyan);
    defaultAppearance.set_opacity(0.5);
    defaultAppearance.set_representation(VisualRepresentation::DrawSurface);
    constructProperty_Appearance(defaultAppearance);
}

const PhysicalFrame& WrapObject::getFrame() const
{
    return _frame.getRef();
}

void WrapObject::setFrame(const PhysicalFrame& frame)
{
    _frame.reset(&frame);
}

/*
 * Scale the wrap object by aScaleFactors. This base class method scales
 * only the _translation property, which is a local member. The derived classes
 * are expected to scale the object itself, because they contain the object's
 * dimensions.
 *
 * @param aScaleFactors The XYZ scale factors.
 */
void WrapObject::scale(const SimTK::Vec3& aScaleFactors)
{
   for (int i=0; i<3; i++)
      upd_translation()[i] *= aScaleFactors[i];
}

void WrapObject::extendFinalizeFromProperties()
{
    SimTK::Rotation rot;
    rot.setRotationToBodyFixedXYZ(get_xyz_body_rotation());
    _pose.set(rot, get_translation());

    const std::string& _quadrantName = get_quadrant();
    if (_quadrantName == "-x" || _quadrantName == "-X") {
        _quadrant = negativeX;
        _wrapAxis = 0;
        _wrapSign = -1;
    } else if (_quadrantName == "x" || _quadrantName == "+x" || _quadrantName == "X" || _quadrantName == "+X") {
        _quadrant = positiveX;
        _wrapAxis = 0;
        _wrapSign = 1;
    } else if (_quadrantName == "-y" || _quadrantName == "-Y") {
        _quadrant = negativeY;
        _wrapAxis = 1;
        _wrapSign = -1;
    } else if (_quadrantName == "y" || _quadrantName == "+y" || _quadrantName == "Y" || _quadrantName == "+Y") {
        _quadrant = positiveY;
        _wrapAxis = 1;
        _wrapSign = 1;
    } else if (_quadrantName == "-z" || _quadrantName == "-Z") {
        _quadrant = negativeZ;
        _wrapAxis = 2;
        _wrapSign = -1;
    } else if (_quadrantName == "z" || _quadrantName == "+z" || _quadrantName == "Z" || _quadrantName == "+Z") {
        _quadrant = positiveZ;
        _wrapAxis = 2;
        _wrapSign = 1;
    } else if (_quadrantName == "all" || _quadrantName == "ALL" || _quadrantName == "All") {
        _quadrant = allQuadrants;
        _wrapSign = 0;
    } else if (_quadrantName == "Unassigned") {  // quadrant was not specified in wrap object definition; use default
        _quadrant = allQuadrants;
        upd_quadrant() = "all";
        _wrapSign = 0;
    } else {
        // quadrant was specified incorrectly in wrap object definition; 
        string errorMessage = "Error: quadrant '" + _quadrantName + "' for wrap object "
            + getName() + " was specified incorrectly.";
        throw Exception(errorMessage);
    }
}

int WrapObject::wrapPathSegment(const SimTK::State& s, 
                                AbstractPathPoint& aPoint1, AbstractPathPoint& aPoint2,
                                const PathWrap& aPathWrap, 
                                WrapResult& aWrapResult) const
{
   int return_code = noWrap;
    bool p_flag;
    Vec3 pt1(0.0);
    Vec3 pt2(0.0);

    // Convert the path points from the frames of the bodies they are attached
    // to, to the frame of the wrap object's body
    pt1 = aPoint1.getParentFrame()
        .findStationLocationInAnotherFrame(s, aPoint1.getLocation(s), getFrame());
    
    pt2 = aPoint2.getParentFrame()
        .findStationLocationInAnotherFrame(s, aPoint2.getLocation(s), getFrame());

    // Convert the path points from the frame of the wrap object's body
    // into the frame of the wrap object
    pt1 = _pose.shiftBaseStationToFrame(pt1);
    pt2 = _pose.shiftBaseStationToFrame(pt2);

    return_code = wrapLine(s, pt1, pt2, aPathWrap, aWrapResult, p_flag);

   if (p_flag == true && return_code > 0) {
        // Convert the tangent points from the frame of the wrap object to the
        // frame of the wrap object's body
        aWrapResult.r1 = _pose.shiftFrameStationToBase(aWrapResult.r1);
        aWrapResult.r2 = _pose.shiftFrameStationToBase(aWrapResult.r2);

        // Convert the surface points (between the tangent points) from the frame of
        // the wrap object to the frame of the wrap object's body
        for (int i = 0; i < aWrapResult.wrap_pts.getSize(); i++)
            aWrapResult.wrap_pts.updElt(i) = _pose.shiftFrameStationToBase(aWrapResult.wrap_pts.get(i));
   }

   return return_code;
}
