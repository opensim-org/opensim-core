#ifndef OPENSIM_COORDINATE_LIMIT_FORCE_H_
#define OPENSIM_COORDINATE_LIMIT_FORCE_H_
// CoordinateLimitForce.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2011, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimActuatorsDLL.h"
#include <OpenSim/Simulation/Model/Force.h>


//=============================================================================
//=============================================================================
/**
 * Generate a force that acts to limit the range of motion of a coordinate.
 * Force is experienced at upper and lower limits of the coordinate value 
 * according to a constant siffneses K_upper and K_lower, with a C2 continuous
 * transition from 0 to K. The transition parameter defines how far beyond the 
 * limit the stiffness becomes constant. The integrator will like smoother
 * (i.e. larger transition regions).
 *
 * Damping factor is also phased in through the transiton region from 0 to the
 * to the value provided.
 *
 * Limiting force is guaranteed to be zero within the upper and lower limits.
 *
 * The potential energy stored in the spring component of the force is
 * accessible as well as the power (nd optionally energy) dissipated. 
 *
 * @author Ajay Seth
 * @version 2.0
 */
namespace OpenSim { 

class OSIMACTUATORS_API CoordinateLimitForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateLimitForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
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
    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================
    /** Default constructor */
    CoordinateLimitForce();

    /** Convenience constructor. 
    Generate a force that acts to limit the range of motion of a coordinate
    Force experienced at upper and lower limits of the coordinate (q) value 
    is according to a linear siffneses K_upper and K_lower, with a C2 continuos
    transition from 0 to K. The transition parameter (dq) defines how far 
    beyond the limit the stiffness becomes purely linear. The integrator will 
    like smoother (i.e. larger transition regions). 
    @param[in]	coordName   Coordinate whose range is to be limited.   
    @param[in]	q_upper	    Coordinate's upper limit value.   
    @param[in]	q_lower	    Coordinate's lower limit value.   
    @param[in]	K_upper	    Upper limit stiffness when coordinate > q_upper
    @param[in]	K_lower	    Lower limit stiffness when coordinate < q_lower
    @param[in]	damping	    Damping factor when coordinate is beyond the limits
    @param[in]	dq		    Transition region (displacement) for force to be 
                            engaged.
    @param[in]	computeDissipationEnergy  
                            Whether to compute dissipated energy (false).
    **/
    CoordinateLimitForce(const std::string& coordName, double q_upper, 
        double K_upper, double q_lower, double K_lower, double damping, 
        double dq, bool computeDissipationEnergy=false); 
    
    // Copy constructor
    CoordinateLimitForce(const CoordinateLimitForce &aForce);
    virtual ~CoordinateLimitForce();
    CoordinateLimitForce& operator=(const CoordinateLimitForce &aForce);

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
    @param[in]          state    
        The State from which to obtain the current value of the power 
        dissipation.
    @return
        The dissipated power (a nonnegative scalar).
    @see getDissipatedEnergy() for the time-integrated power loss **/
    double getPowerDissipation(const SimTK::State& s) const;

    /** Obtain energy dissipated by this CoordinateLimitForce over time
    in units of energy in J. 
    @param[in]          state    
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
    double computePotentialEnergy(const SimTK::State& s) const;


    //--------------------------------------------------------------------------
    // REPORTING
    //--------------------------------------------------------------------------
    /** 
     * Methods to query a Force for the value actually applied during simulation
     * The names of quantities (column labels) are  returned by getRecordLabels()
     */
    Array<std::string> getRecordLabels() const ;
    /**
     * Given SimTK::State object extract all the values necessary to report forces, application location
     * frame, etc. used in conjunction with getRecordLabels and should return same size Array
     */
    Array<double> getRecordValues(const SimTK::State& state) const ;

protected:
    //--------------------------------------------------------------------------
    // Model Component Interface
    //--------------------------------------------------------------------------
    /** Setup this CoordinateLimitForce as part of the model.
        This were the existence of the coordinate to limit is checked. */ 
    void connectToModel(Model& aModel) OVERRIDE_11;
    /** Create the underlying Force that is part of the multibodysystem. */
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    //--------------------------------------------------------------------------
    // Force Interface
    //--------------------------------------------------------------------------
    void computeForce(const SimTK::State& s, 
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                      SimTK::Vector& mobilityForces) const OVERRIDE_11;

private:
    // Object helpers
    void setNull();
    void constructProperties();
    void copyData(const CoordinateLimitForce &aForce);

    // Model Component Interface when computing energy
    SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const;

    // Smooth step functions for continuous transition from no stiffness and 
    // damping to constant values beyond the limits. These are heap allocated
    // and owned here so must be deleted in destructor.
    SimTK::Function::Step *upStep;
    SimTK::Function::Step *loStep;

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

//=============================================================================
};	// END of class CoordinateLimitForce

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef OPENSIM_COORDINATE_LIMIT_FORCE_H_
