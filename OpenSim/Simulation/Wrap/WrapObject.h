#ifndef OPENSIM_WRAP_OBJECT_H_
#define OPENSIM_WRAP_OBJECT_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  WrapObject.h                           *
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

// INCLUDE
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Appearance.h>
namespace OpenSim {

class PathWrap;
class WrapResult;
class Model;
class PhysicalFrame;
class AbstractPathPoint;

//=============================================================================
//=============================================================================
/**
 * An abstract class that specifies the interface for a wrapping
 * object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapObject : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(WrapObject, Component);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(active, bool,
        "Whether or not the WrapObject is considered active in computing paths");

    OpenSim_DECLARE_PROPERTY(xyz_body_rotation, SimTK::Vec3,
        "Body-fixed Euler angle sequence for the orientation of the WrapObject");

    OpenSim_DECLARE_PROPERTY(translation, SimTK::Vec3,
        "Translation of the WrapObject.");

    // Default display properties e.g. Representation, color, texture, etc.
    OpenSim_DECLARE_UNNAMED_PROPERTY(Appearance,
        "Default appearance for this Geometry");

    OpenSim_DECLARE_PROPERTY(quadrant, std::string,
        "The name of quadrant over which the wrap object is active. "
        "For example, '+x' or '-y' to set the sidedness of the wrapping.");

    enum WrapQuadrant
    {
        allQuadrants,
        negativeX,
        positiveX,
        negativeY,
        positiveY,
        negativeZ,
        positiveZ
    };

    enum WrapAction
    {
        noWrap,          // the path segment did not intersect the wrap object
        insideRadius,    // one or both path points are inside the wrap object
        wrapped,         // successful wrap, but may not be 'best' path
        mandatoryWrap    // successful wrap that must be used (e.g., both tangent
    };                  // points are on the constrained side of the wrap object)



//=============================================================================
// METHODS
//=============================================================================
public:
    WrapObject();
    virtual ~WrapObject();

    // Use default copy and assignment operator

    virtual void scale(const SimTK::Vec3& aScaleFactors);
    virtual void connectToModelAndBody(Model& aModel, PhysicalFrame& aBody) {}

    const PhysicalFrame& getFrame() const;
    void setFrame(const PhysicalFrame& frame);

    bool getActiveUseDefault() const { 
        return getProperty_active().getValueIsDefault();
    }
    bool getQuadrantNameUseDefault() const {
        return getProperty_quadrant().getValueIsDefault();
    }
    const SimTK::Transform& getTransform() const { return _pose; }
    virtual const char* getWrapTypeName() const = 0;

    // TODO: total SIMM hack!
    virtual std::string getDimensionsString() const { return ""; }

//=============================================================================
// WRAPPING
//=============================================================================
/**
* Calculate the wrapping of one path segment over one wrap object.
* @param state   The State of the model
* @param aPoint1 The first path point
* @param aPoint2 The second path point
* @param aPathWrap An object holding the parameters for this path/wrap-object pairing
* @param aWrapResult The result of the wrapping (tangent points, etc.)
* @return The status, as a WrapAction enum
*/
    int wrapPathSegment( const SimTK::State& state, 
                         AbstractPathPoint& aPoint1, AbstractPathPoint& aPoint2,
                         const PathWrap& aPathWrap,
                         WrapResult& aWrapResult) const;

protected:
    virtual int wrapLine(const SimTK::State& state,
                         SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                         const PathWrap& aPathWrap,
                         WrapResult& aWrapResult, bool& aFlag) const = 0;

    virtual void updateGeometry() {};

   /** Determine the appropriate values of _quadrant, _wrapAxis, and _wrapSign,
     * based on the name of the quadrant. finalizeFromProperties() should be
     * called whenever the quadrant property changes. */
    void extendFinalizeFromProperties() override;

    void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber)
        override;

private:
    void constructProperties();

    SimTK::ReferencePtr<const PhysicalFrame> _frame;

protected:

    WrapQuadrant _quadrant{ WrapQuadrant::allQuadrants };
    int _wrapAxis{ 0 };
    int _wrapSign{ 1 };

    SimTK::Transform _pose;
//=============================================================================
};  // END of class WrapObject
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_WRAP_OBJECT_H_


