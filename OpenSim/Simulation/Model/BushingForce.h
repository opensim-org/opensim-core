#ifndef OPENSIM_BUSHING_FORCE_H_
#define OPENSIM_BUSHING_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  BushingForce.h                          *
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
#include "Force.h"
#include <OpenSim/Simulation/Model/TwoFrameLinker.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

//==============================================================================
//                             BUSHING FORCE
//==============================================================================
/**
 * A class implementing a Bushing Force.
 * A Bushing Force is the force proportional to the deviation of two frames.
 * One can think of the Bushing as being composed of 3 linear and 3 torsional
 * spring-dampers, which act along or about the bushing frames. Orientations
 * are measured as x-y-z body-fixed Euler rotations, which are treated as
 * though they were uncoupled. Damping is proportional to the deflection rate of 
 * change (e.g. Euler angle derivatives) which is NOT the angular velocity between
 * the two frames. That makes this bushing model suitable only for relatively
 * small relative orientation deviations between the frames.
 * The underlying Force in Simbody is a SimtK::Force::LinearBushing.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API BushingForce 
    : public TwoFrameLinker<Force, PhysicalFrame> {
OpenSim_DECLARE_CONCRETE_OBJECT(BushingForce, TwoFrameLinker);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    // Physical properties of the bushing force
    OpenSim_DECLARE_PROPERTY(rotational_stiffness, SimTK::Vec3,
        "Stiffness parameters resisting rotational deviation (Nm/rad).");
    OpenSim_DECLARE_PROPERTY(translational_stiffness, SimTK::Vec3,
        "Stiffness parameters resisting relative translation (N/m).");
    OpenSim_DECLARE_PROPERTY(rotational_damping, SimTK::Vec3,
        "Damping parameters resisting angular deviation rate. (Nm/(rad/s))");
    OpenSim_DECLARE_PROPERTY(translational_damping, SimTK::Vec3,
        "Damping parameters resisting relative translational velocity. (N/(m/s))");
//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor leaves frames unspecified and sets all bushing 
        stiffness and damping properties to zero. **/
    BushingForce();

    /** Convenience Constructor.
    Create a BushingForce between two PhysicalFrames, frame1 and frame2.
    @param[in] name         the name of this BushingForce
    @param[in] frame1       the bushing's first PhysicalFrame
    @param[in] frame2       the bushing's second PhysicalFrame
    */
    BushingForce(const std::string& name,
                 const PhysicalFrame& frame1,
                 const PhysicalFrame& frame2);

    /** Convenience Constructor.
    Create a BushingForce between two PhysicalFrames, frame1 and frame2.
    @param[in] name         the name of this BushingForce
    @param[in] frame1Name   the name of the bushing's first PhysicalFrame
    @param[in] frame2Name   the name of the bushing's second PhysicalFrame
    */
    BushingForce(const std::string& name,
                 const std::string& frame1Name,
                 const std::string& frame2Name);

    /** Construct a BushingForce given physical frames that it
     * tries to keep aligned by generating a passive force according to the
     * physical properties of the bushing. See property declarations for more
     * information. */
    BushingForce(const std::string& name,
                 const PhysicalFrame& frame1,
                 const PhysicalFrame& frame2,
                 const SimTK::Vec3& transStiffness,
                 const SimTK::Vec3& rotStiffness,
                 const SimTK::Vec3& transDamping,
                 const SimTK::Vec3& rotDamping);

    /** Construct a BushingForce given the names of physical frames that it
     * tries to keep aligned by generating a passive force according to the
     * physical properties of the bushing. See property declarations for more
     * information. */
    BushingForce(const std::string& name,
                 const std::string& frame1Name,
                 const std::string& frame2Name,
                 const SimTK::Vec3& transStiffness,
                 const SimTK::Vec3& rotStiffness,
                 const SimTK::Vec3& transDamping,
                 const SimTK::Vec3& rotDamping);

    /** Convenience Constructor
    Construct a BushingForce between two frames with offset transforms on the
    respective frames.

    @param[in] name              the name of this BushingForce
    @param[in] frame1            first PhysicalFrame that the bushing connects
    @param[in] transformInFrame1 offset Transform on the first frame
    @param[in] frame2            second PhysicalFrame that the bushing connects
    @param[in] transformInFrame2 offset Transform on the second frame
    @param[in] transStiffness    translational (dx, dy, dz) stiffnesses
    @param[in] rotStiffness      rotational (dq_x, dq_y, dq_z) stiffnesses
    @param[in] transDamping      translational (dx/dt, dy/dt, dz/dt) damping
    @param[in] rotDamping        rotational (dq_x/dt, dq_y/dt, dq_z/dt) damping
    */
    BushingForce(const std::string &name,
                 const PhysicalFrame& frame1, const SimTK::Transform& transformInFrame1,
                 const PhysicalFrame& frame2, const SimTK::Transform& transformInFrame2,
                 const SimTK::Vec3& transStiffness,
                 const SimTK::Vec3& rotStiffness,
                 const SimTK::Vec3& transDamping,
                 const SimTK::Vec3& rotDamping);

    /** Convenience Constructor
    Construct a BushingForce where the two frames are specified by the name
    and offset transforms on the respective frames.

    @param[in] name              the name of this BushingForce 
    @param[in] frame1Name        first PhysicalFrame that the bushing connects
    @param[in] transformInFrame1 offset Transform on the first frame
    @param[in] frame2Name        second PhysicalFrame that the bushing connects
    @param[in] transformInFrame2 offset Transform on the second frame
    @param[in] transStiffness    translational (dx, dy, dz) stiffnesses
    @param[in] rotStiffness      rotational (dq_x, dq_y, dq_z) stiffnesses
    @param[in] transDamping      translational (dx/dt, dy/dt, dz/dt) damping
    @param[in] rotDamping        rotational (dq_x/dt, dq_y/dt, dq_z/dt) damping
    */
    BushingForce(const std::string &name,
        const std::string& frame1Name, const SimTK::Transform& transformInFrame1,
        const std::string& frame2Name, const SimTK::Transform& transformInFrame2,
        const SimTK::Vec3& transStiffness,
        const SimTK::Vec3& rotStiffness,
        const SimTK::Vec3& transDamping,
        const SimTK::Vec3& rotDamping);


    /** Backwards compatible Convenience Constructor
    Construct a BushingForce where the bushing frames are specified in terms of their
    location and orientation in their respective PhysicalFrames and passive force
    defined by the physical properties of the bushing. See property declarations
    for more information. 
    @param[in] name                the name of this BushingForce
    @param[in] frame1Name          name of the first PhysicalFrame of the bushing
    @param[in] locationInFrame1    Vec3 location of the bushing in the first frame
    @param[in] orientationInFrame1 Vec3 of the XYZ body-fixed Euler angles of the
                                   bushing frame orientation in frame 1.
    @param[in] frame2Name          name of the second PhysicalFrame of the bushing
    @param[in] locationInFrame2    Vec3 location of the weld in the second frame
    @param[in] orientationInFrame2 Vec3 of the XYZ body-fixed Euler angles of the
                                   bushing frame orientation in frame2.
    @param[in] transStiffness      translational (dx, dy, dz) stiffnesses
    @param[in] rotStiffness        rotational (dq_x, dq_y, dq_z) stiffnesses
    @param[in] transDamping        translational (dx/dt, dy/dt, dz/dt) damping
    @param[in] rotDamping          rotational (dq_x/dt, dq_y/dt, dq_z/dt) damping
    */
    BushingForce(const std::string &name,
        const std::string& frame1Name,
        const SimTK::Vec3& locationInFrame1, const SimTK::Vec3& orientationInFrame1,
        const std::string& frame2Name,
        const SimTK::Vec3& locationInFrame2, const SimTK::Vec3& orientationInFrame2,
        const SimTK::Vec3& transStiffness,
        const SimTK::Vec3& rotStiffness,
        const SimTK::Vec3& transDamping,
        const SimTK::Vec3& rotDamping);

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

    /** Potential energy is the elastic energy stored in the bushing. */
    double computePotentialEnergy(const SimTK::State& s) const final override;

    //--------------------------------------------------------------------------
    // Reporting
    //--------------------------------------------------------------------------
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) 
     * to be reported. */
    OpenSim::Array<std::string> getRecordLabels() const override;
    /**
    *  Provide the value(s) to be reported that correspond to the labels */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override;

private:
    //--------------------------------------------------------------------------
    // Implement ModelComponent interface.
    //--------------------------------------------------------------------------
    // Create a SimTK::Force::LinearBushing which implements this BushingForce.
    void extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system) 
                                                                    const override;

    void setNull();
    void constructProperties();
//==============================================================================
};  // END of class BushingForce
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_BUSHING_FORCE_H_


