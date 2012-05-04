#ifndef OPENSIM_THELEN_2003_MUSCLE_H_
#define OPENSIM_THELEN_2003_MUSCLE_H_

// Thelen2003Muscle.h
/*    Author: Matthew Millard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
 * -------------------------------------------------------------------------- */


// INCLUDE
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
#include <OpenSim/Simulation/Model/MuscleFirstOrderActivationDynamicModel.h>
#include <OpenSim/Simulation/Model/MuscleFixedWidthPennationModel.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {
//==============================================================================
//                          THELEN 2003 MUSCLE
//==============================================================================
/**
 Implementation of a two state (activation and fiber-length) Muscle model by:
 DG Thelen, Adjustment of muscle mechanics model parameters to simulate dynamic 
 contractions in older adults. Journal of biomechanical engineering, 2003.
 This a complete rewrite of a previous implementation by Peter Loan. 

 The Thelen2003Muscle model uses a standard equilibrium muscle equation

 \f[ (a(t) f_{AL}(l_{CE}) f_{V}(\dot{l}_{CE}) 
 - f_{PE}(l_{CE}))\cos \phi - f_{SE}(l_{T}) = 0  \f]

 Rearranging the above equation and solving for \f$ f_{V}(\dot{l}_{CE}) \f$ 
 yields

 \f[ f_{V}(\dot{l}_{CE}) = 
 \frac{ \frac{f_{SE}(l_{T})}{\cos\phi} - f_{PE}(l_{CE}) }{ a(t) f_{AL}(l_{CE}) }
 \f]

 The force velocity curve is usually inverted to compute the fiber velocity,
 
 \f[ \dot{l}_{CE} = f_{V}^{-1}( 
 \frac{ \frac{f_{SE}(l_{T})}{\cos\phi} - f_{PE}(l_{CE}) }{ a(t) f_{AL}(l_{CE}) }
 )
 \f]

 which is then integrated to simualate the musculotendon dynamics. In general, 
 the previous equation has 4 singularity conditions:

 -# \f$ a(t) \rightarrow 0 \f$
 -# \f$ f_{AL}(l_{CE}) \rightarrow 0 \f$
 -# \f$ \phi \rightarrow \frac{\pi}{2} \f$
 -# \f$ f_{V}(\dot{l}_{CE}) \le 0 \f$ or \f$ f_{V}(\dot{l}_{CE}) \ge F^M_{len}\f$

 This implementation has been modified from the model presented in the journal
 paper (marked with a *) to prevent some of these singularities:
 
 -# *\f$ a(t) \rightarrow a_{min} > 0 \f$ : A modified activation dynamic 
 equation is used - MuscleFirstOrderActivationDynamicModel - which smoothly 
 approaches some minimum value that is greater than zero.
 -# \f$ f_{AL}(l_{CE}) > 0 \f$ . The active force length curve of the Thelen 
    muscle is a Gaussian, which is always greater than 0.
 -# \f$ \phi \rightarrow \frac{\pi}{2} \f$ . This singularity cannot be removed 
    without changing Eqn. 1, and still exists in the present Thelen2003Muscle
    formulation.
 -# *\f$ f_{V}(\dot{l}_{CE}) \le 0 \f$ or 
 \f$ f_{V}(\dot{l}_{CE}) \ge F^M_{len}\f$: Equation 6 in Thelen 2003 has been modified so that \f$ V^M \f$ is linearly
  extrapolated when \f$ F^M < 0\f$ (during a concentric contraction), and when
  \f$ F^M > 0.95 F^M_{len}\f$ (during an eccentric contraction). These two 
  modifications make the force velocity curve invertible. The original force
  velocity curve as published by Thelen was not invertible.


 @author Matt Millard
 @author Ajay Seth
 @author Peter Loan
 */
class OSIMACTUATORS_API Thelen2003Muscle : public ActivationFiberLengthMuscle {
OpenSim_DECLARE_CONCRETE_OBJECT(Thelen2003Muscle, ActivationFiberLengthMuscle);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "time constant for ramping up muscle activation");

    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
        "time constant for ramping down of muscle activation");

    OpenSim_DECLARE_PROPERTY(FmaxTendonStrain, double,
        "tendon strain at maximum isometric muscle force");

    OpenSim_DECLARE_PROPERTY(FmaxMuscleStrain, double,
        "passive muscle strain at maximum isometric muscle force");

    OpenSim_DECLARE_PROPERTY(KshapeActive, double,
        "shape factor for Gaussian active muscle force-length relationship");   

    OpenSim_DECLARE_PROPERTY(KshapePassive, double,
        "exponential shape factor for passive force-length relationship");   

    OpenSim_DECLARE_PROPERTY(Af, double,
        "force-velocity shape factor"); 

    OpenSim_DECLARE_PROPERTY(Flen, double,
        "maximum normalized lengthening force");

    OpenSim_DECLARE_PROPERTY(activation_minimum_value, double,
        "minimum activation value permitted");
    
    OpenSim_DECLARE_PROPERTY(fv_linear_extrap_threshold, double,
        "fv threshold where linear extrapolation is used");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    Thelen2003Muscle();
    Thelen2003Muscle(const std::string &aName,double aMaxIsometricForce,
                    double aOptimalFiberLength,double aTendonSlackLength,
                    double aPennationAngle);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

//==============================================================================
// Get and Set Properties
//==============================================================================
    // Properties
    double getActivationTimeConstant() const;
    double getActivationMinimumValue() const;
    double getDeactivationTimeConstant() const;
    double getFmaxTendonStrain() const;
    double getFmaxMuscleStrain() const;
    double getKshapeActive() const;
    double getKshapePassive() const;
    double getAf() const;
    double getFlen() const;
    double getForceVelocityExtrapolationThreshold() const;

    bool setActivationTimeConstant(double aActivationTimeConstant);
    bool setActivationMinimumValue(double aActivationMinValue);
    bool setDeactivationTimeConstant(double aDeactivationTimeConstant);
    bool setFmaxTendonStrain(double aFmaxTendonStrain);
    bool setFmaxFiberStrain(double aFmaxMuscleStrain);
    bool setKshapeActive(double aKShapeActive);
    bool setKshapePassive(double aKshapePassive);
    bool setAf(double aAf);
    bool setFlen(double aFlen);;
    bool setForceVelocityExtrapolationThreshold(double aFvThresh);

//==============================================================================
// Public Computations
//==============================================================================
    //Ajay: this is old. Can I stop calling it?
    virtual double computeActuation(const SimTK::State& s) const;


    /** Compute initial fiber length (velocity) such that muscle fiber and 
        tendon are in static equilibrium and update the state
        
        Part of the Muscle.h interface
    */
    void computeInitialFiberEquilibrium(SimTK::State& s) const; /*virtual*/
    double computeIsometricForce(   SimTK::State& s, 
                                    double activation) const;/*virtual*/

protected:
//==============================================================================
// Protected Computations
//==============================================================================

    /**calculate muscle's position related values such fiber and tendon lengths,
    normalized lengths, pennation angle, etc... */
    void calcMuscleLengthInfo(const SimTK::State& s, 
                              MuscleLengthInfo& mli) const OVERRIDE_11;


    /** calculate muscle's velocity related values such fiber and tendon 
        velocities,normalized velocities, pennation angular velocity, etc... */
    void  calcFiberVelocityInfo(const SimTK::State& s, 
                                      FiberVelocityInfo& fvi) const OVERRIDE_11; 

    /** calculate muscle's active and passive force-length, force-velocity, 
        tendon force, relationships and their related values */
    void  calcMuscleDynamicsInfo(const SimTK::State& s, 
                                    MuscleDynamicsInfo& mdi) const OVERRIDE_11;

    /** Calculate activation rate */
    double calcActivationRate(const SimTK::State& s) const OVERRIDE_11; 

    void createSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;


private:
    void setNull();
    void constructProperties();

    //=====================================================================
    // Private Utility Class Members
    //      -Computes activation dynamics and fiber kinematics
    //=====================================================================
    //bool initializedModel;

    //Activation Dynamics
    MuscleFirstOrderActivationDynamicModel actMdl;

    //Fiber and Tendon Kinematics
    MuscleFixedWidthPennationModel penMdl;

    //=====================================================================
    // Private Accessor names
    //=====================================================================
    //This is so we can get some compiler checking on these string names


    //=====================================================================
    // Private Computation
    //      -Computes curve values, derivatives and integrals
    //=====================================================================

    //Initialization
    SimTK::Vector initMuscleState(SimTK::State& s, double aActivation,
                             double aSolTolerance, int aMaxIterations) const;

    //Stiffness related functions
    double calcFm(double ma, double fal, double fv, double fpe, double fiso) const;


    double calcDFmDlce(double lce, double a,  double fv, 
                      double fiso, double ofl) const;

    double calcDFmATDlce(double lce, double phi, double cosphi, 
    double Fm, double d_Fm_d_lce, double penHeight) const;

    double calcDFseDlce(double tl, double lce, double phi, double cosphi, 
                                    double fiso, double tsl, double vol) const;

    double calcDFseDtl(double tl, double fiso, double tsl) const;
    
    
    //Tendon related helper functions
    double calcfse(double tlN) const;
    double calcDfseDtlN(double tlN) const;
    double calcfsefisoPE(double tlN) const;

    //Active force length functions
    double calcfal( double lceN) const;
    double calcDfalDlceN( double lceN) const;

    //Parallel element functions    
    double calcfpe(double lceN) const;
    double calcDfpeDlceN(double lceN) const;
    double calcfpefisoPE(double lceN) const;    

    //Force velocity functions      
    double calcdlceN(double act,double fal, double actFalFv) const;
    double calcfv(double aFse, double aFpe, double aFal,
                  double aCosPhi, double aAct) const;
    SimTK::Vector calcfvInv(double aAct,  double aFal, double dlceN, 
                            double tolerance, int maxIterations) const;
    double calcDdlceDaFalFv(double aAct, double fal, 
                            double aFalFv) const;


};    
} // end of namespace OpenSim

#endif // OPENSIM_THELEN_2003_MUSCLE_H_
