#ifndef OPENSIM_THELEN_2003_MUSCLE_H_
#define OPENSIM_THELEN_2003_MUSCLE_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  Thelen2003Muscle.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard, Ajay Seth, Peter Loan                          *
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
#include <OpenSim/Common/Component.h>
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
#include <OpenSim/Actuators/MuscleFirstOrderActivationDynamicModel.h>
#include <OpenSim/Actuators/MuscleFixedWidthPennationModel.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

//==============================================================================
//                               Thelen2003Muscle
//==============================================================================
/**
 Implementation of a two state (activation and fiber-length) Muscle model by 
 Thelen 2003.\ This a complete rewrite of a previous implementation (present in
 OpenSim 2.4 and earlier) contained numerous errors.

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

 which is then integrated to simulate the musculotendon dynamics. In general, 
 the previous equation has 4 singularity conditions:

 -# \f$ a(t) \rightarrow 0 \f$
 -# \f$ f_{AL}(l_{CE}) \rightarrow 0 \f$
 -# \f$ \phi \rightarrow \frac{\pi}{2} \f$
 -# \f$ f_{V}(\dot{l}_{CE}) \le 0 \f$ or 
    \f$ f_{V}(\dot{l}_{CE}) \ge F^M_{len}\f$

 This implementation has been slightly modified from the model presented in the 
 journal paper (marked with a *) to prevent some of these singularities:
 
 -# *\f$ a(t) \rightarrow a_{min} > 0 \f$ : A modified activation dynamic 
    equation is used - MuscleFirstOrderActivationDynamicModel - which smoothly 
    approaches some minimum value that is greater than zero.
 -# \f$ f_{AL}(l_{CE}) > 0 \f$ . The active force length curve of the Thelen 
    muscle is a Gaussian, which is always greater than 0.
 -# \f$ \phi \rightarrow \frac{\pi}{2} \f$ . This singularity cannot be removed 
    without changing the first equation, and still exists in the present 
    Thelen2003Muscle formulation.
 -# *\f$ f_{V}(\dot{l}_{CE}) \le 0 \f$ or 
    \f$ f_{V}(\dot{l}_{CE}) \ge F^M_{len}\f$: Equation 6 in Thelen 2003 has been 
    modified so that \f$ V^M \f$ is linearly extrapolated when \f$ F^M < 0\f$ 
    (during a concentric contraction), and when \f$ F^M > 0.95 F^M_{len}\f$ 
    (during an eccentric contraction). These two modifications make the force 
    velocity curve invertible. The original force velocity curve as published 
    by Thelen was not invertible.   
 -# A unilateral constraint has been implemented to prevent the fiber from 
    approaching a fiber length that is smaller than 0.01*optimal fiber length,
    or a fiber length that creates a pennation angle greater than the maximum 
    pennation angle specified by the pennation model. Note that this unilateral 
    constraint does not prevent the muscle fiber from becoming shorter than is 
    physiologically possible (that is shorter than approximately half a 
    normalized fiber length).

  <B> References </B>

   DG Thelen, Adjustment of muscle mechanics model parameters to simulate dynamic 
 contractions in older adults. Journal of biomechanical engineering, 2003.

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

    OpenSim_DECLARE_PROPERTY(fv_linear_extrap_threshold, double,
        "fv threshold where linear extrapolation is used");

    OpenSim_DECLARE_PROPERTY(maximum_pennation_angle, double,
        "Maximum pennation angle, in radians");

    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Activation time constant, in seconds");

    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
        "Deactivation time constant, in seconds");

    OpenSim_DECLARE_PROPERTY(minimum_activation, double,
        "Lower bound on activation");

    enum CurveType{FiberActiveForceLength,
                    FiberPassiveForceLength,
                    FiberForceVelocity,
                    TendonForceLength};

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

    /** @name Convenience methods
    These are convenience methods that get and set properties of the activation
    and pennation models. **/
    /**@{**/
    double getActivationTimeConstant() const;
    void setActivationTimeConstant(double actTimeConstant);
    double getDeactivationTimeConstant() const;
    void setDeactivationTimeConstant(double deactTimeConstant);
    double getMinimumActivation() const;
    void setMinimumActivation(double minimumActivation);
    double getMaximumPennationAngle() const;
    void setMaximumPennationAngle(double maximumPennationAngle);
    /**@}**/

    /**
     @returns the minimum fiber length, which is the maximum of two values:
        the smallest fiber length allowed by the pennation model, and the 
        minimum fiber length in the active force length curve. When the fiber
        length reaches this value, it is constrained to this value until the 
        fiber velocity goes positive.
    */
    double getMinimumFiberLength() const;

    /**
   @returns the MuscleFirstOrderActivationDynamicModel 
            that this muscle model uses
   */
    const MuscleFirstOrderActivationDynamicModel& getActivationModel() const;

    /**
   @returns the MuscleFixedWidthPennationModel 
            that this muscle model uses
   */
    const MuscleFixedWidthPennationModel& getPennationModel() const;

//==============================================================================
// Public Convenience Methods
//==============================================================================
    void printCurveToCSVFile(const CurveType ctype, 
                            const std::string& path) const;

//==============================================================================
// Public Computations
//==============================================================================
    //Ajay: this is old. Can I stop calling it?
    double computeActuation(const SimTK::State& s) const override;


    /** Compute initial fiber length (velocity) such that muscle fiber and 
        tendon are in static equilibrium and update the state
        
        Part of the Muscle.h interface

        @throws MuscleCannotEquilibrate
    */
    void computeInitialFiberEquilibrium(SimTK::State& s) const override;
       
    ///@cond DEPRECATED
    /*  Once the ignore_tendon_compliance flag is implemented correctly get rid 
        of this method as it duplicates code in calcMuscleLengthInfo,
        calcFiberVelocityInfo, and calcMuscleDynamicsInfo
    */
    /*
    @param activation of the muscle [0-1]
    @param fiberLength in (m)
    @param fiberVelocity in (m/s)
    @returns the force component generated by the fiber that is associated only
             with activation (the parallel element is not included)   
    */
    double calcActiveFiberForceAlongTendon( double activation, 
                                            double fiberLength, 
                                            double fiberVelocity) const; 

    double calcInextensibleTendonActiveFiberForce(SimTK::State& s, 
                                             double aActivation)
        const override final;
    ///@endcond

protected:
//==============================================================================
// Protected Computations
//==============================================================================

    /**calculate muscle's position related values such fiber and tendon lengths,
    normalized lengths, pennation angle, etc... */
    void calcMuscleLengthInfo(const SimTK::State& s, 
                              MuscleLengthInfo& mli) const override;

    /** calculate muscle's velocity related values such fiber and tendon 
        velocities,normalized velocities, pennation angular velocity, etc... */
    void  calcFiberVelocityInfo(const SimTK::State& s, 
                                      FiberVelocityInfo& fvi) const override; 

    /** calculate muscle's active and passive force-length, force-velocity, 
        tendon force, relationships and their related values */
    void  calcMuscleDynamicsInfo(const SimTK::State& s, 
                                    MuscleDynamicsInfo& mdi) const override;

    /** calculate muscle's fiber and tendon potential energy */
    void calcMusclePotentialEnergyInfo(const SimTK::State& s,
        MusclePotentialEnergyInfo& mpei) const override;

    /** Calculate activation rate */
    double calcActivationRate(const SimTK::State& s) const override; 

    /** Component interface. */
    void extendFinalizeFromProperties() override;

    /** Implement the ModelComponent interface */
    void extendConnectToModel(Model& aModel) override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

private:
    void setNull();
    void constructProperties();

    // Subcomponents owned by the muscle. The properties of these subcomponents
    // are set (in extendFinalizeFromProperties()) from the properties of the
    // muscle.
    MemberSubcomponentIndex pennMdlIdx{
      constructSubcomponent<MuscleFixedWidthPennationModel>("pennMdl") };
    MemberSubcomponentIndex actMdlIdx{
      constructSubcomponent<MuscleFirstOrderActivationDynamicModel>("actMdl") };

    //=====================================================================
    // Private Computation
    //      -Computes curve values, derivatives and integrals
    //=====================================================================

    // Status flag returned by initMuscleState().
    enum StatusFromInitMuscleState {
        Success_Converged,
        Warning_FiberAtLowerBound,
        Failure_MaxIterationsReached
    };

    // Associative array of values returned by initMuscleState():
    // solution_error, iterations, fiber_length, passive_force, and
    // tendon_force.
    typedef std::map<std::string, double> ValuesFromInitMuscleState;

    /* Calculate the muscle state such that the fiber and tendon are developing
    the same force.

    @param s the system state
    @param aActivation the initial activation of the muscle
    @param aSolTolerance the desired relative tolerance of the equilibrium 
           solution
    @param aMaxIterations the maximum number of Newton steps allowed before we
           give up attempting to initialize the model
    */
    std::pair<StatusFromInitMuscleState, ValuesFromInitMuscleState>
        initMuscleState(const SimTK::State& s,
                        const double aActivation,
                        const double aSolTolerance,
                        const int aMaxIterations) const;

    double calcFm(double ma, double fal, double fv, 
                 double fpe, double fiso) const;

    double calcActiveFm(double ma, double fal, 
                        double fv, double fiso) const;

    //Stiffness related functions
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
    double calcfvInv(double aAct,  double aFal, double dlceN, 
                            double tolerance, int maxIterations) const;
    double calcDdlceDaFalFv(double aAct, double fal, 
                            double aFalFv) const;

    //Returns true if the fiber state is currently clamped to prevent the 
    //fiber from attaining a length that is too short.
    bool isFiberStateClamped(const SimTK::State& s, 
                            double dlceN) const;

    void printMatrixToFile(SimTK::Matrix& data, SimTK::Array_<std::string>& colNames,
    const std::string& path, const std::string& filename) const;

};    
} // end of namespace OpenSim

#endif // OPENSIM_THELEN_2003_MUSCLE_H_
