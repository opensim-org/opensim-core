#ifndef OPENSIM_WELD_CONSTRAINT_H_
#define OPENSIM_WELD_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  WeldConstraint.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

class PhysicalOffsetFrame;

//=============================================================================
//=============================================================================
/**
 * A class implementing a Weld Constraint. A WeldConstraint eliminates up to
 * 6 dofs of a model by fixing to PhysicalFrames together at their origins
 * aligning their axes.  PhysicalFrames are generally Ground, Body, or
 * PhysicalOffsetFrame attached to a PhysicalFrame.
 * The underlying Constraint in Simbody is a SimTK::Constraint::Weld
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API WeldConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(WeldConstraint, Constraint);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** WeldConstraint defined frames used to connect PhysicalFrames like 
        Bodies but offset from the body origin. */
    OpenSim_DECLARE_LIST_PROPERTY(frames, PhysicalFrame,
        "Physical frames needed to satisfy WeldConstraint connections.");

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    WeldConstraint();
    // Convenience constructors
    WeldConstraint(const std::string &name,
                const std::string& frame1Name,
                const std::string& frame2Name );

    // Deprecated constructors
    WeldConstraint(const std::string &name, 
        const PhysicalFrame& frame1,
        const SimTK::Vec3& locationInFrame1, const SimTK::Vec3& orientationInFrame1,
        const PhysicalFrame& frame2,
        const SimTK::Vec3& locationInFrame2, const SimTK::Vec3& orientationInFrame2);

    WeldConstraint(const std::string &name,
        const PhysicalFrame& frame1, const SimTK::Transform& transformInFrame1,
        const PhysicalFrame& frame2, const SimTK::Transform& transformInFrame2);

    virtual ~WeldConstraint();

    // Method to set point locations for induced acceleration analysis
    virtual void setContactPointForInducedAccelerations(
        const SimTK::State &s, SimTK::Vec3 point);

protected:
    /**
    * Extend Component Interface.
    */
    void extendFinalizeFromProperties() override;
    void extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system)
                                                                  const override;
    /** Updating XML formating to latest revision */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;


private:
    void setNull();
    // Construct WeldConstraint's properties
    void constructProperties() override;
    // Construct WeldConstraint's connectors
    void constructConnectors() override;

    // Some analyses (e.g. Induced Accelerations, update the constraint
    // location (Transform) based on experimental data. The constraint
    // keeps its own internal frames to update.
    SimTK::ReferencePtr<PhysicalOffsetFrame> _internalOffset1{ nullptr };
    SimTK::ReferencePtr<PhysicalOffsetFrame> _internalOffset2{ nullptr };
//=============================================================================
};  // END of class WeldConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_WELD_CONSTRAINT_H_


