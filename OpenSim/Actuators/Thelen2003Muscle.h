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
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <simbody/internal/common.h>

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
//                          THELEN 2003 MUSCLE
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

 which is then integrated to simualate the musculotendon dynamics. In general, 
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

     <B> Usage </B>
    This object should be updated through the set methods provided. 
    These set methods will take care of rebuilding the object correctly. If you
    modify the properties directly, the object will not be rebuilt, and upon
    calling any functions an exception will be thrown because the object is out 
    of date with its properties.

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

    //OpenSim_DECLARE_PROPERTY(activation_minimum_value, double,
    //    "minimum activation value permitted");
    
    OpenSim_DECLARE_PROPERTY(fv_linear_extrap_threshold, double,
        "fv threshold where linear extrapolation is used");


    /**@}**/

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
    double getActivationTimeConstant() const;
    double getMinimumActivation() const;
    double getDeactivationTimeConstant() const;
    double getFmaxTendonStrain() const;
    double getFmaxMuscleStrain() const;
    double getKshapeActive() const;
    double getKshapePassive() const;
    double getAf() const;
    double getFlen() const;
    double getForceVelocityExtrapolationThreshold() const;

    /**
     @returns the minimum fiber length, which is the maximum of two values:
        the smallest fiber length allowed by the pennation model, and the 
        minimum fiber length in the active force length curve. When the fiber
        length reaches this value, it is constrained to this value until the 
        fiber velocity goes positive.
    */
    double getMinimumFiberLength() const;

    /**
    @return the maximum pennation angle allowed by this muscle model (radians)
    */
    double getMaximumPennationAngle() const;

    bool setActivationTimeConstant(double aActivationTimeConstant);   
    bool setDeactivationTimeConstant(double aDeactivationTimeConstant);
    bool setMinimumActivation(double aActivationMinValue);
    bool setFmaxTendonStrain(double aFmaxTendonStrain);
    bool setFmaxFiberStrain(double aFmaxMuscleStrain);
    bool setKshapeActive(double aKShapeActive);
    bool setKshapePassive(double aKshapePassive);
    bool setAf(double aAf);
    bool setFlen(double aFlen);
    bool setForceVelocityExtrapolationThreshold(double aFvThresh);

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
                            const std::string& path);

//==============================================================================
// Public Computations
//==============================================================================
    //Ajay: this is old. Can I stop calling it?
    virtual double computeActuation(const SimTK::State& s) const OVERRIDE_11;


    /** Compute initial fiber length (velocity) such that muscle fiber and 
        tendon are in static equilibrium and update the state
        
        Part of the Muscle.h interface
    */
    void computeInitialFiberEquilibrium(SimTK::State& s) const OVERRIDE_11;
       
    ///@cond TO BE DEPRECATED. 
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

    virtual double calcInextensibleTendonActiveFiberForce(SimTK::State& s, 
                                             double aActivation) const FINAL_11;
    ///@endcond

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

	/** calculate muscle's fiber and tendon potential energy */
	void calcMusclePotentialEnergyInfo(const SimTK::State& s,
		MusclePotentialEnergyInfo& mpei) const;

    /** Calculate activation rate */
    double calcActivationRate(const SimTK::State& s) const OVERRIDE_11; 

    virtual void addToSystem(SimTK::MultibodySystem& system) const;
	virtual void initStateFromProperties(SimTK::State& s) const;
    virtual void setPropertiesFromState(const SimTK::State& state);
    virtual void connectToModel(Model& aModel);


private:
    void setNull();
    void constructProperties();
    void buildMuscle();
    void ensureMuscleUpToDate();
    //=====================================================================
    // Private Utility Class Members
    //      -Computes activation dynamics and fiber kinematics
    //=====================================================================
    //bool initializedModel;

    double activation_minimum_value;
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
    SimTK::Vector calcfvInv(double aAct,  double aFal, double dlceN, 
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
