#ifndef OPENSIM_COORDINATE_LIMIT_FORCE_H_
#define OPENSIM_COORDINATE_LIMIT_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  CoordinateLimitForce.h                      *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Force.h>


//=============================================================================
//=============================================================================
namespace OpenSim {

/**
 * Generate a force that acts to limit the range of motion of a coordinate.
 * Force is experienced at upper and lower limits of the coordinate value
 * according to a constant stiffnesses K_upper and K_lower, with a C2 continuous
 * transition from 0 to K. The transition parameter defines how far beyond the
 * limit the stiffness becomes constant. The integrator will like smoother
 * (i.e. larger transition regions).
 *
 * Damping factor is also phased in through the transition region from 0 to the
 * value provided.
 *
 * Limiting force is guaranteed to be zero within the upper and lower limits.
 *
 * The potential energy stored in the spring component of the force is
 * accessible as well as the power (nd optionally energy) dissipated.
  * The function has the following shape:
 * 
 * \image html coordinate_limit_force.png
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API CoordinateLimitForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateLimitForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(coordinate, std::string,
        "Coordinate (name) to be limited.");
    OpenSim_DECLARE_PROPERTY(upper_stiffness, double,
        "Stiffness of the passive limit force when coordinate exceeds upper "
        "limit. Note, rotational stiffness expected in N*m/degree.");
    OpenSim_DECLARE_PROPERTY(upper_limit, double,
        "The upper limit of the coordinate range of motion (rotations in "
        "degrees).");
    OpenSim_DECLARE_PROPERTY(lower_stiffness, double,
        "Stiffness of the passive limit force when coordinate exceeds lower "
        "limit. Note, rotational stiffness expected in N*m/degree.");
    OpenSim_DECLARE_PROPERTY(lower_limit, double,
        "The lower limit of the coordinate range of motion (rotations in "
        "degrees).");
    OpenSim_DECLARE_PROPERTY(damping, double,
        "Damping factor on the coordinate's speed applied only when limit "
        "is exceeded. For translational has units N/(m/s) and rotational has "
        "Nm/(degree/s)");
    OpenSim_DECLARE_PROPERTY(transition, double,
        "Transition region width in the units of the coordinate (rotations "
        "in degrees). Dictates the transition from zero to constant stiffness "
        "as coordinate exceeds its limit.");
    OpenSim_DECLARE_PROPERTY(compute_dissipation_energy, bool, 
        "Option to compute the dissipation energy due to damping in the "
        "CoordinateLimitForce. If true the dissipation power is automatically "
        "integrated to provide energy. Default is false.");

//=============================================================================
// PUBLIC METHODS
//=============================================================================
    /** Default constructor */
    CoordinateLimitForce();

    /** Convenience constructor.
    Generate a force that acts to limit the range of motion of a coordinate
    Force experienced at upper and lower limits of the coordinate (q) value
    is according to a linear stiffnesses K_upper and K_lower, with a C2 continuous
    transition from 0 to K. The transition parameter (dq) defines how far
    beyond the limit the stiffness becomes purely linear. The integrator will
    like smoother (i.e. larger transition regions).
    @param[in]  coordName   Coordinate whose range is to be limited.
    @param[in]  q_upper     Coordinate's upper limit value.
    @param[in]  K_upper     Upper limit stiffness when coordinate > q_upper
    @param[in]  q_lower     Coordinate's lower limit value.
    @param[in]  K_lower     Lower limit stiffness when coordinate < q_lower
    @param[in]  damping     Damping factor when coordinate is beyond the limits
    @param[in]  dq          Transition region (displacement) for force to be
                            engaged.
    @param[in]  computeDissipationEnergy  
                            Whether to compute dissipated energy (false).
    **/
    CoordinateLimitForce(const std::string& coordName, double q_upper,
        double K_upper, double q_lower, double K_lower, double damping,
        double dq, bool computeDissipationEnergy=false);

    //use compiler default copy constructor and assignment operator

    /** Destructor */
    ~CoordinateLimitForce() override = default;


    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // Properties
    /** Stiffness of the passive limit force when coordinate exceeds upper 
    limit. Note, rotational stiffness expected in N*m/degree. */
    void setUpperStiffness(double aUpperStiffness);
    double getUpperStiffness() const;
    
    /** Upper limit of the coordinate range of motion (rotations in degrees).*/
    void setUpperLimit(double aUpperLimit);
    double getUpperLimit() const;
    
    /** Stiffness of the passive limit force when coordinate exceeds lower 
    limit. Note, rotational stiffness expected in N*m/degree. */
    void setLowerStiffness(double aLowerStiffness);
    double getLowerStiffness() const;
    
    /** Lower limit of the coordinate range of motion (rotations in degrees).*/
    void setLowerLimit(double aLowerLimit);
    double getLowerLimit() const;
    
    /** Damping factor on the coordinate's speed applied only when limit is 
    exceeded. For translational has units N/(m/s) and rotational has 
    Nm/(degree/s). */
    void setDamping(double aDamping);
    double getDamping() const;

    /** Transition region width with lengths is m and angles in degrees).
    Specifies the transition from zero to a constant stiffness as 
    coordinate exceeds its limit.*/
    void setTransition(double aTransition);
    double getTransition() const;

    /** Option to compute the dissipation energy due to damping in the 
    CoordinateLimitForce. If true the dissipation power is automatically 
    integrated to provide energy. Default is false. */
    void setComputeDissipationEnergy(bool flag);
    bool isComputingDissipationEnergy() const;

    /** Obtain the rate at which energy is being dissipated by this 
    CoordinateLimit, that is, the power being lost. This is in units of 
    energy/time which is watts in J/s. 
    @param[in]          s    
        The State from which to obtain the current value of the power 
        dissipation.
    @return
        The dissipated power (a nonnegative scalar).
    @see getDissipatedEnergy() for the time-integrated power loss **/
    double getPowerDissipation(const SimTK::State& s) const;

    /** Obtain energy dissipated by this CoordinateLimitForce over time
    in units of energy in J. 
    @param[in]          s    
        The State from which to obtain the current value of 
        dissipated energy 
    @return
        The dissipated energy (a nonnegative scalar). **/
    double getDissipatedEnergy(const SimTK::State& s) const;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    /** Force calculation operator. **/
    double calcLimitForce( const SimTK::State& s) const;

    /** Contribute this Force component's potential energy to the accounting
    of the total system energy. **/
    double computePotentialEnergy(const SimTK::State& s) const override;


    //--------------------------------------------------------------------------
    // REPORTING
    //--------------------------------------------------------------------------
    /** 
     * Methods to query a Force for the value actually applied during simulation
     * The names of quantities (column labels) are  returned by getRecordLabels()
     */
    Array<std::string> getRecordLabels() const override ;
    /**
     * Given SimTK::State object extract all the values necessary to report forces, application location
     * frame, etc. used in conjunction with getRecordLabels and should return same size Array
     */
    Array<double> getRecordValues(const SimTK::State& state) const override ;

protected:
    //--------------------------------------------------------------------------
    // Model Component Interface
    //--------------------------------------------------------------------------
    /** Setup this CoordinateLimitForce as part of the model.
        This were the existence of the coordinate to limit is checked. */ 
    void extendConnectToModel(Model& aModel) override;
    /** Create the underlying Force that is part of the multibody system. */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    //--------------------------------------------------------------------------
    // Force Interface
    //--------------------------------------------------------------------------
    void computeForce(const SimTK::State& s, 
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                      SimTK::Vector& mobilityForces) const override;

private:
    // Object helpers
    void setNull();
    void constructProperties();

    // Model Component Interface when computing energy
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

    // Smooth step functions for continuous transition from no stiffness and 
    // damping to constant values beyond the limits. These are heap allocated
    // and owned here, but we don't want these functions to be copied if a
    // CoordinateLimitForce is copied.
    SimTK::ResetOnCopy<std::unique_ptr<SimTK::Function::Step> > _upStep;
    SimTK::ResetOnCopy<std::unique_ptr<SimTK::Function::Step> > _loStep;

    // Scaling for coordinate values in m or degrees (rotational) 
    double _w;

    // Coordinate limits in internal (SI) units (m or rad)
    double _qup;
    double _qlow;
    // Constant stiffnesses in internal (SI) N/m or Nm/rad
    double _Kup;
    double _Klow;

    // Damping in internal (SI) units of N/(m/s) or Nm/(rad/s)
    double _damp;

    // Corresponding generalized coordinate to which the coordinate actuator
    // is applied.
    SimTK::ReferencePtr<Coordinate> _coord;

    mutable CacheVariable<double> _dissipationPowerCV;

//=============================================================================
};  // END of class CoordinateLimitForce

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef OPENSIM_COORDINATE_LIMIT_FORCE_H_
