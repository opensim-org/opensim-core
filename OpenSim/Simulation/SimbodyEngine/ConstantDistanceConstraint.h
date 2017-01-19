#ifndef OPENSIM_CONSTANT_DISTANCE_CONSTRAINT_H_
#define OPENSIM_CONSTANT_DISTANCE_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ConstantDistanceConstraint.h                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers                                                  *
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
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a constraint that maintains a constant distance between
 * between two points on separate PhysicalFrames. 
 * The underlying SimTK::Constraint in Simbody is a SimTK::Constraint::Rod.
 *
 * @author Matt DeMers
 */
class OSIMSIMULATION_API ConstantDistanceConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantDistanceConstraint, Constraint);

//=============================================================================
// DATA
//=============================================================================
public:
    OpenSim_DECLARE_PROPERTY(location_body_1, SimTK::Vec3,
        "Location of the point in first body specified in body1 "
        "reference frame.");
    OpenSim_DECLARE_PROPERTY(location_body_2, SimTK::Vec3,
        "Location of the point in second body specified in body2 "
        "reference frame.");
    OpenSim_DECLARE_PROPERTY(constant_distance, double, "constant distance "
        "to be rigidly maintained between the two points "
        "fixed on each body.");

    OpenSim_DECLARE_SOCKET(body_1, PhysicalFrame,
        "The first body participating in this constraint.");
    OpenSim_DECLARE_SOCKET(body_2, PhysicalFrame,
        "The second body participating in this constraint.");


//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    ConstantDistanceConstraint();
    /**
    * Convenience Constructor.
    *
    * @param body1          first PhysicalFrame connected by the constraint
    * @param locationBody1  point fixed on body1 where the constraint is applied
    * @param body2          second PhysicalFrame connected by the constraint
    * @param locationBody2: point fixed on body2 where the constraint is applied
    * @param distance       nonzero fixed distance between the points
    */
    ConstantDistanceConstraint(
        const PhysicalFrame& body1, const SimTK::Vec3& locationBody1, 
        const PhysicalFrame& body2, const SimTK::Vec3& locationBody2,
        const double& distance);

    virtual ~ConstantDistanceConstraint();

    /** The Physical frames that the constraint is connected to are
        accessible after connectToModel() has been called on the Model. */
    const PhysicalFrame& getBody1() const;
    const PhysicalFrame& getBody2() const;
    //SET 
    void setBody1ByName(const std::string& aBodyName);
    void setBody1PointLocation(SimTK::Vec3 location);
    void setBody2ByName(const std::string& aBodyName);
    void setBody2PointLocation(SimTK::Vec3 location);
    void setConstantDistance(double distance);

protected:
    /**
    * Extend Component Interface.
    */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    // Visual support Constraint drawing in visualizer.
    void generateDecorations(
        bool                                        fixed,
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
        override;

    /** Updating XML formatting to latest revision */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;


private:
    /** Construct ConstantDistanceConstraint's properties */
    void constructProperties();

    void setNull();

//=============================================================================
}; // END of class ConstantDistanceConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CONSTANT_DISTANCE_CONSTRAINT_H_


