#ifndef OPENSIM_COUPLED_BUSHING_FORCE_H_
#define OPENSIM_COUPLED_BUSHING_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  CoupledBushingForce.h                       *
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
#include "osimPluginDLL.h"
#include <OpenSim/Common/Property.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>

namespace OpenSim {

//=================================================================================
//=================================================================================
/**
 * A class implementing a Coupled Bushing Force.
 * A Couple Bushing Force is the force proportional to the deviation of two frames, 
 * where components of the resulting BodyForce are coupled to any or all 
 * deviations, such that the general stiffness and damping matrices are 6x6.
 *
 * Deviations of an offset_frame (1) (on a PhysicalFrame e.g. a Body) and another 
 * offset_frame2 (on another PhysicalFrame) are expressed in terms of and x-y-z 
 * Euler angle sequence (for rotational deviation in radians) and x,y,z (distance
 * in m) of the bushing's offset_frame2 w.r.t. offset_frame1.
 * Damping is applied to the relative angular velocity (rad/s) and linear velocity 
 * (m/s) of offset_frame2 in offset_frame1.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMPLUGIN_API CoupledBushingForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(CoupledBushingForce, Force);
public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with the CoupleBushingForce. **/
    /**@{**/
    /// BushingForce defined frames that are connected by this bushing
    OpenSim_DECLARE_LIST_PROPERTY(frames, PhysicalFrame,
        "Physical frames needed to satisfy the BushingForce's connections.");

    /** Stiffness of the bushing related to Euler XYZ body-fixed angular and
        translational deviations that express frame2 in frame1. Force is zero
        when frames are coincident and aligned. */
    OpenSim_DECLARE_PROPERTY(stiffness_row1, SimTK::Vec6,
        "1st row of stiffness matrix: rotation (Nm/rad) & translation (N/m).");
    OpenSim_DECLARE_PROPERTY(stiffness_row2, SimTK::Vec6,
        "2nd row of stiffness matrix: rotation (Nm/rad) & translation (N/m).");
    OpenSim_DECLARE_PROPERTY(stiffness_row3, SimTK::Vec6,
        "3rd row of stiffness matrix: rotation (Nm/rad) & translation (N/m).");
    OpenSim_DECLARE_PROPERTY(stiffness_row4, SimTK::Vec6,
        "4th row of stiffness matrix: rotation (Nm/rad) & translation (N/m).");
    OpenSim_DECLARE_PROPERTY(stiffness_row5, SimTK::Vec6,
        "5th row of stiffness matrix: rotation (Nm/rad) & translation (N/m).");
    OpenSim_DECLARE_PROPERTY(stiffness_row6, SimTK::Vec6,
        "6th row of stiffness matrix: rotation (Nm/rad) & translation (N/m).");

    /** Damping of the bushing related to XYZ angular and
        translational speeds that express frame2 in frame1.  */
    OpenSim_DECLARE_PROPERTY(damping_row1, SimTK::Vec6,
        "1st row of damping matrix 3 rotation (Nm/(rad/s)) & 3 translation (N/(m/s)).");
    OpenSim_DECLARE_PROPERTY(damping_row2, SimTK::Vec6,
        "2nd row of damping matrix 3 rotation (Nm/(rad/s)) & 3 translation (N/(m/s)).");
    OpenSim_DECLARE_PROPERTY(damping_row3, SimTK::Vec6,
        "3rd row of damping matrix 3 rotation (Nm/(rad/s)) & 3 translation (N/(m/s)).");
    OpenSim_DECLARE_PROPERTY(damping_row4, SimTK::Vec6,
        "4th row of damping matrix 3 rotation (Nm/(rad/s)) & 3 translation (N/(m/s)).");
    OpenSim_DECLARE_PROPERTY(damping_row5, SimTK::Vec6,
        "5th row of damping matrix 3 rotation (Nm/(rad/s)) & 3 translation (N/(m/s)).");
    OpenSim_DECLARE_PROPERTY(damping_row6, SimTK::Vec6,
        "6th row of damping matrix 3 rotation (Nm/(rad/s)) & 3 translation (N/(m/s)).");

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    CoupledBushingForce();
    /** Construct the CoupleBushingForce given the two offset frames to which
    it attaches by name. Stiffness and damping properties are supplied by 6x6
    matrices. Rotational stiffness (damping) in N/rad(/s) and translational in
    N/m(/s). Off-diagonals represent the coupling terms.
    See property declarations for more information. **/
    CoupledBushingForce(const PhysicalFrame& frame1,
                        const PhysicalFrame& frame2,
                        SimTK::Mat66 stiffnessMat,
                        SimTK::Mat66 dampingMat);

    virtual ~CoupledBushingForce();

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.


    //--------------------------------------------------------------------------
    // COMPUTATION
    //--------------------------------------------------------------------------
    /** Compute the deflection (spatial separation) of the two frames connected
        by the bushing force. Angular displacement expressed in Euler angles.
        The force and potential energy are determined by the deflection.  */
    SimTK::Vec6 computeDeflection(const SimTK::State& s) const;

    /** Compute the bushing force contribution to the system and add in to appropriate
     bodyForce and/or system generalizedForce. The bushing force is [K]*dq + [D]*dqdot
     where, [K] is the spatial 6dof stiffness matrix between the two frames 
         dq is the deflection in body spatial coordinates with rotations in Euler angles
         [D] is the spatial 6dof damping matrix opposing the velocity between the frames
         dqdot is the relative spatial velocity of the two frames
     CoupledBushingForce implementation based SimTK::Force::LinearBushing
     developed and implemented by Michael Sherman. */
    void computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const override;

    /** Potential energy is determined by the elastic energy storage of the bushing.
        In spatial terms, U = ~dq*[K]*dq, with K and dq defined above. */
    double computePotentialEnergy(const SimTK::State& s) const override;

    //-----------------------------------------------------------------------------
    // Reporting
    //-----------------------------------------------------------------------------
    /** 
     Provide name(s) of the quantities (column labels) of the force value(s) to be reported
     */
    OpenSim::Array<std::string> getRecordLabels() const override;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override;



private:
    void setNull();

    //--------------------------------------------------------------------------
    // Implement ModelComponent interface.
    //--------------------------------------------------------------------------
    void constructProperties();
    void constructConnectors() override;
    void extendFinalizeFromProperties() override;

    void finalizeMatricesFromProperties();
    void updatePropertiesFromMatrices();

    // internal matrices
    SimTK::Mat66 _stiffnessMatrix;
    SimTK::Mat66 _dampingMatrix;

//=============================================================================
};  // END of class CoupledBushingForce
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_COUPLED_BUSHING_FORCE_H_


