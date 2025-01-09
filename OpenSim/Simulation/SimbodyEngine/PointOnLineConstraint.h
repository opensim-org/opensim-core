#ifndef OPENSIM_POINT_ON_LINE_CONSTRAINT_H_
#define OPENSIM_POINT_ON_LINE_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  PointOnLineConstraint.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Samuel R. Hamner                                                *
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
#include "Constraint.h"

namespace OpenSim {

class PhysicalFrame;
//=============================================================================
//=============================================================================
/**
 * A class implementing a Point On Line Constraint.  The underlying Constraint 
 * in Simbody is a SimTK::Constraint::PointOnLine.
 *
 * @author Samuel Hamner
 */
class OSIMSIMULATION_API PointOnLineConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(PointOnLineConstraint, Constraint);

public:
    OpenSim_DECLARE_PROPERTY(line_direction_vec, SimTK::Vec3,
        "Direction of the line specified in the line body frame.");
    OpenSim_DECLARE_PROPERTY(point_on_line, SimTK::Vec3,
        "The default point on the line specified in the line body frame.");
    OpenSim_DECLARE_PROPERTY(point_on_follower, SimTK::Vec3,
        "The point on (and specified in) the follower body constrained to the line.");

    OpenSim_DECLARE_SOCKET(line_body, PhysicalFrame,
        "A frame fixed to the body that contains the line along which the "
        "point on the follower body can move.");
    OpenSim_DECLARE_SOCKET(follower_body, PhysicalFrame,
        "A frame fixed to the body that contains the point that is constrained "
        "to move along a line.");

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    PointOnLineConstraint();
    PointOnLineConstraint(const PhysicalFrame& lineBody,
            const SimTK::Vec3& lineDirection, SimTK::Vec3 pointOnLine,
            const PhysicalFrame& followerBody, const SimTK::Vec3& followerPoint);

    virtual ~PointOnLineConstraint();

    //SET 
    void setLineBodyByName(const std::string& aBodyName);
    void setFollowerBodyByName(const std::string& aBodyName);
    void setLineDirection(SimTK::Vec3 direction);
    void setPointOnLine(SimTK::Vec3 point);
    void setPointOnFollower(SimTK::Vec3 point);

protected:
    /**
    * Extend Component Interface.
    */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    /** Updating XML formating to latest revision */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;

private:
    /** Construct PointConstraint's properties */
    void constructProperties();
    void setNull();


//=============================================================================
};  // END of class PointOnLineConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_POINT_ON_LINE_CONSTRAINT_H_


