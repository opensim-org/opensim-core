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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/TwoFrameLinker.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

class PhysicalOffsetFrame;

//=============================================================================
//=============================================================================
/**
 * A class implementing a Weld Constraint. A WeldConstraint eliminates up to
 * 6 dofs of a model by fixing two PhysicalFrames together at their origins
 * aligning their axes.  PhysicalFrames are generally Ground, Body, or
 * PhysicalOffsetFrame attached to a PhysicalFrame.
 * The underlying Constraint in Simbody is a SimTK::Constraint::Weld.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API WeldConstraint 
    : public TwoFrameLinker<Constraint, PhysicalFrame> {
OpenSim_DECLARE_CONCRETE_OBJECT(WeldConstraint, TwoFrameLinker);
public:
    /** Default Constructor. Create an unnamed WeldConstraint with frame
        sockets that are unsatisfied. */
    WeldConstraint();

    /** Convenience Constructor.
    Create a WeldConstraint between two PhysicalFrames, frame1 and frame2.
    @param[in] name         the name of this WeldConstraint 
    @param[in] frame1Name   the name of the first PhysicalFrame being constrained
    @param[in] frame2Name   the name of the second PhysicalFrame being constrained
    */
    WeldConstraint( const std::string& name,
                    const std::string& frame1Name,
                    const std::string& frame2Name );

    /** Backwards compatible Convenience Constructor 
    Construct a WeldConstraint where the weld frames are specified in terms of their
    location and orientation in their respective PhysicalFrames. 

    @param[in] name             the name of this WeldConstraint
    @param[in] frame1           the first PhysicalFrame that the weld constrains
    @param[in] locationInFrame1    Vec3 of the location of the weld in the first frame
    @param[in] orientationInFrame1 Vec3 of the XYZ body-fixed Euler angles of the
                                   weld frame orientation in frame 1.
    @param[in] frame2               the second PhysicalFrame that the weld constrains
    @param[in] locationInFrame2    Vec3 of the location of the weld in the second frame
    @param[in] orientationInFrame2 Vec3 of the XYZ body-fixed Euler angles
                                   of the weld frame orientation in frame2.
    */
    WeldConstraint(const std::string &name, 
        const PhysicalFrame& frame1,
        const SimTK::Vec3& locationInFrame1, const SimTK::Vec3& orientationInFrame1,
        const PhysicalFrame& frame2,
        const SimTK::Vec3& locationInFrame2, const SimTK::Vec3& orientationInFrame2);

    /** Convenience Constructor
    Construct a WeldConstraint where the weld frames are specified in terms of their
    transforms in their respective PhysicalFrames.

    @param[in] name         the name of this WeldConstraint
    @param[in] frame1       the first PhysicalFrame that the weld constrains
    @param[in] transformInFrame1    Transform of the weld in the first frame
    @param[in] frame2       the second PhysicalFrame that the weld constrains
    @param[in] transformInFrame2    Transform of the weld in the second frame
    */
    WeldConstraint(const std::string &name,
        const PhysicalFrame& frame1, const SimTK::Transform& transformInFrame1,
        const PhysicalFrame& frame2, const SimTK::Transform& transformInFrame2);

    virtual ~WeldConstraint();

    /** Advanced Method for computing induced accelerations given the constraint
        applied at the point of contact specified. */
    virtual void setContactPointForInducedAccelerations(
        const SimTK::State &s, SimTK::Vec3 point) override;

protected:
    /** Extend Component Interface. */
    void extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system)
                                                                  const override;

private:
    void setNull();
    // Construct WeldConstraint's properties
    void constructProperties();

    // Some analyses (e.g. Induced Accelerations, update the constraint
    // location (Transform) based on experimental data. The constraint
    // keeps its own internal frames to update.
    SimTK::ResetOnCopy<std::unique_ptr<PhysicalOffsetFrame>> _internalOffset1{};
    SimTK::ResetOnCopy<std::unique_ptr<PhysicalOffsetFrame>> _internalOffset2{};
//=============================================================================
};  // END of class WeldConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_WELD_CONSTRAINT_H_


