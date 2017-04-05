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

void WrapObject::updateFromXMLNode(SimTK::Xml::Element& node,
        int versionNumber) {
    int documentVersion = versionNumber;
    if (documentVersion < XMLDocument::getLatestVersion()) {
        if (documentVersion < 30515) {
            // Replace 3.3 display_preference, color, and VisibleObject with
            // Appearance's visible and color.
            // We ignore most of VisibleObject's other properties (e.g.,
            // transform), since it would be misleading to draw the wrap object
            // in the wrong place.
            SimTK::Xml::Element appearanceNode("Appearance"); 
            // Use the correct defaults for WrapObject's appearance, which
            // are different from the default Appearance.
            SimTK::Xml::Element color("color");
            color.setValue("0 1 1"); // SimTK::Cyan
            appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                    color);
            SimTK::Xml::Element defaultOpacity("opacity");
            defaultOpacity.setValue("0.5");
            appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                    defaultOpacity);
            SimTK::Xml::Element defaultSurfProp("SurfaceProperties");
            appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                    defaultSurfProp);
            SimTK::Xml::Element rep("representation");
            rep.setValue("3"); // VisualRepresentation::DrawSurface
            defaultSurfProp.insertNodeAfter(defaultSurfProp.element_end(), rep);
            bool appearanceModified = false;

            // color.
            SimTK::Xml::element_iterator colorIter =
                    node.element_begin("color");
            if (colorIter != node.element_end()) {
                color.setValue(colorIter->getValue());
                node.removeNode(colorIter);
                appearanceModified = true;
            }

            // display_preference -> visible, representation
            SimTK::Xml::Element visibleNode("visible");
            SimTK::Xml::element_iterator dispPrefIter =
                    node.element_begin("display_preference");
            if (dispPrefIter != node.element_end()) {
                if (dispPrefIter->getValueAs<int>() == 0) {
                    visibleNode.setValue("false");
                    appearanceModified = true;
                } else if (dispPrefIter->getValueAs<int>() < 4) {
                    // Set `representation`.
                    // If the value is 4, we use 3 instead, which is the
                    // default (above).
                    rep.setValue(dispPrefIter->getValue());
                    appearanceModified = true;
                }
                node.removeNode(dispPrefIter);
            }
            SimTK::Xml::element_iterator visObjIter =
                    node.element_begin("VisibleObject");
            if (visObjIter != node.element_end()) {
                SimTK::Xml::element_iterator voDispPrefIter =
                        visObjIter->element_begin("display_preference");
                if (voDispPrefIter != visObjIter->element_end()) {
                    if (voDispPrefIter->getValueAs<int>() == 0) {
                        visibleNode.setValue("false");
                        appearanceModified = true;
                    } else if (rep.getValue() == "3" && 
                            voDispPrefIter->getValueAs<int>() < 4) {
                        // Set `representation`.
                        // The representation still has its default value,
                        // meaning the user only specified the inner display
                        // preference, therefore we should use this one.
                        rep.setValue(voDispPrefIter->getValue());
                        appearanceModified = true;
                    }
                }
                node.removeNode(visObjIter);
            }
            if (visibleNode.getValue() != "") {
                appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                        visibleNode);
                appearanceModified = true;
            }

            // Add Appearance to the WrapObject.
            if (appearanceModified) {
                node.insertNodeAfter(node.element_end(), appearanceNode);
            }
        }
    }
    Super::updateFromXMLNode(node, versionNumber);
}
