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
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include "Force.h"

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
 * though they were uncoupled. That makes this bushing model suitable only for
 * relatively small relative orientation between the frames.
 * The underlying Force in Simbody is a SimtK::Force::LinearBushing.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API BushingForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(BushingForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    // Physical properties of the bushing force
    OpenSim_DECLARE_PROPERTY(rotational_stiffness, SimTK::Vec3,
        "Stiffness parameters resisting relative rotation (Nm/rad).");
    OpenSim_DECLARE_PROPERTY(translational_stiffness, SimTK::Vec3,
        "Stiffness parameters resisting relative translation (N/m).");
    OpenSim_DECLARE_PROPERTY(rotational_damping, SimTK::Vec3,
        "Damping parameters resisting relative angular velocity. (Nm/(rad/s))");
    OpenSim_DECLARE_PROPERTY(translational_damping, SimTK::Vec3,
        "Damping parameters resisting relative translational velocity. (N/(m/s)");

    /// BushingForce defined frames that are connected by this bushing
    OpenSim_DECLARE_LIST_PROPERTY(frames, PhysicalFrame,
        "Physical frames needed to satisfy the BushingForce's connections.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor leaves frames unspecified and sets all bushing 
        stiffness and damping properties to zero. **/
    BushingForce();

    /** Construct a BushingForce given the names of physical frames that it
    tries to keep aligned by generating a passive force according to the physical
    properties of the bushing.
    See property declarations for more information. */
    BushingForce(const std::string& frame1Name,
                 const std::string& frame2Name,
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
    /** Update the XML format of the BushingForce from older versions */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;

    //--------------------------------------------------------------------------
    // Implement ModelComponent interface.
    //--------------------------------------------------------------------------
    void constructConnectors() override;
    void extendFinalizeFromProperties() override;
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


