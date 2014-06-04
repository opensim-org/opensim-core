#ifndef OPENSIM_Millard2012AccelerationMuscle_h__
#define OPENSIM_Millard2012AccelerationMuscle_h__
/* -------------------------------------------------------------------------- *
 *                 OpenSim:  Millard2012AccelerationMuscle.h                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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
#include <OpenSim/Actuators/osimActuatorsDLL.h>

#include <simbody/internal/common.h>
#include <OpenSim/Simulation/Model/Muscle.h>
/*
The parent class, Muscle.h, provides 
    1. max_isometric_force
    2. optimal_fiber_length
    3. tendon_slack_length
    4. pennation_angle_at_optimal
    5. max_contraction_velocity
*/
#include <OpenSim/Simulation/Model/Muscle.h>

//All of the sub-models required for a muscle model:
#include <OpenSim/Actuators/MuscleFirstOrderActivationDynamicModel.h>
#include <OpenSim/Actuators/MuscleFixedWidthPennationModel.h>

#include <OpenSim/Actuators/ActiveForceLengthCurve.h>
#include <OpenSim/Actuators/ForceVelocityCurve.h>
#include <OpenSim/Actuators/ForceVelocityInverseCurve.h>
#include <OpenSim/Actuators/FiberForceLengthCurve.h>
#include <OpenSim/Actuators/FiberCompressiveForceLengthCurve.h>
#include <OpenSim/Actuators/FiberCompressiveForceCosPennationCurve.h>
#include <OpenSim/Actuators/TendonForceLengthCurve.h>


namespace OpenSim {


/**
This class implements a 3 state (activation,fiber length and fiber velocity) 
acceleration musculo-tendon model that has several advantages over 
equilibrium musculo-tendon models: it is possible to simulate 0 activation, it 
requires fewer integrator steps to simulate, and physiological active 
force-length (with a minimum value of 0) and force velocity (with true 
asymptotes at the maximum shortening and lengthening velocites) 
curves can be employed.

\image html fig_Millard2012AccelerationMuscle.png

The dynamic equation of the mass, constrained to move in direction 
\f$ \hat{i} \f$ is given by the scalar equation:

\f[
m \ddot{x} = F_{SE} - F_{CE} \cdot \hat{i}  
\f]


The kinematic expression for the acceleration of the mass, \f$ \ddot{s} \f$, 
expressed in terms of the fiber length,\f$l_{CE}\f$, and pennation angle 
\f$\phi\f$ is

\f[
\ddot{x} = \Big(\ddot{l}_{CE} \cos \phi - 2 \dot{l}_{CE}\dot{\phi}\sin\phi 
- \dot{\phi}^2 l_{CE} \cos \phi - \ddot{\phi} l_{CE} \sin \phi\Big)
\f]

The kinematic expression for the angular acceleration of the pennation angle 
can be found by taking the second derivative of the pennation constraint 
equation

\f[
l_{CE} \sin \phi = h
\f]

which yields

\f[
\ddot{\phi} = -\Big( \ddot{l}_{CE}\sin\phi + 
                    2 \dot{l}_{CE} \dot{\phi} \cos\phi
                    - \dot{\phi}^2 l_{CE} \sin \phi \Big) 
                    / \Big( l_{CE} \cos \phi \Big)
\f]

An expression for \f$ \ddot{l}_{CE}\f$ can be obtained by substituting in
the equations \f$ \ddot{\phi} \f$ 
into the equation for \f$\ddot{x}\f$ and simplifying:
\f[
\ddot{l}_{CE} = \frac{1}{m} \Big(  F_{SE} - F_{CE} \cdot \hat{i} \Big) \cos \phi
                + l_{CE} \dot{\phi}^2
\f]

Notice that the above equation for \f$\ddot{l}_{CE}\f$ has no singularities,
provided that there are no singularities in \f$ F_{SE}\f$ and 
\f$ F_{M}\cdot\hat{i}\f$. The force the fiber applies to the tendon (in N), 
\f$F_{CE}\cdot\hat{i}\f$, is given by (+'ve is tension)

\f[
F_{CE} \cdot \hat{i} =  
f_{ISO}\Big(\mathbf{a} \mathbf{f}_L(\hat{l}_{CE}) 
\mathbf{f}_V(\frac{\hat{v}_{CE}}{v_{MAX}}) 
+ \beta_{CE}\hat{v}_{CE}
+ \mathbf{f}_{PE}(\hat{l}_{CE})(1+\beta_{PE}\hat{v}_{CE}) 
- \mathbf{f}_K(\hat{l}_{CE})(1-\beta_{K}\hat{v}_{CE})  \Big) \cos \phi
-  f_{ISO} \Big( \mathbf{f}_{c\phi}(\cos \phi)
(1- \beta_{c \phi}
\frac{d}{dt}(\frac{l_{CE}\cos\phi}{l_{CE,OPT}\cos\phi_{OPT}})) \Big)
\f]

The force the tendon generates (in N) is given by (+'ve is tension)

\f[
F_{SE} =  
f_{ISO} \mathbf{f}_{SE}(\hat{l}_{SE})(1+\beta_{SE}\hat{v}_{SE}) 
\f]

Every elastic element (\f$\mathbf{f}_{PE}\f$,\f$\mathbf{f}_{K}\f$,
\f$\mathbf{f}_{c\phi}\f$, and \f$\mathbf{f}_{SE}\f$) is accompanied by a 
non-linear damping element of a form that is identical to the damping found 
in a Hunt-Crossley contact model. Additionally a linear damping element,
\f$\beta_{CE}\hat{v}_{CE}\f$, is
located in the fiber as in J.He et al. Damping is necessary to include in this 
model to prevent the mass from oscillating in a non-physiologic manner. 
Nonlinear damping Hunt-Crossley damping (where the damping force is scaled 
by the elastic force) has been chosen because this form of damping doesn't 
increase the stiffness of the system equations 
(because it is gradually turned on). 

As with the Hunt-Crossley contact model, the force generated by the nonlinear 
spring and damper saturated so that it is greater than or equal to zero. This
saturation is necessary to ensure that tension elements can only generate 
tensile forces, and that compressive elements only generate compressive forces. 
Note that the sign conventions have been chosen so that damping forces are 
generated in the correct direction for each element.

\f{eqnarray*}{
(1+\beta_{PE}\hat{v}_{CE}) > 0 \\
(1-\beta_{K}\hat{v}_{CE}) > 0 \\
(1- \beta_{c \phi}
\frac{d}{dt}(\frac{l_{CE}\cos\phi}{l_{CE,OPT}\cos\phi_{OPT}})) > 0 \\
(1+\beta_{SE}\hat{v}_{SE}) > 0
\f}


For more information on these new terms please see the
doxygen for FiberCompressiveForceLengthCurve, 
FiberCompressiveForceCosPennationCurve, and 
MuscleFirstOrderActivationDynamicModel.

<B>Units</B>

\li m: meters
\li rad: radians
\li N: Newtons
\li kg: kilograms
\li s: seconds

<B>Usage</B>

 Note that this object should be updated through the set methods provided. 
 These set methods will take care of rebuilding the muscle correctly. If you
 modify the properties directly, the curve will not be rebuilt, and upon
 calling a function that requires a state an exception will be thrown because 
 the muscle is out of date with its properties.

 Note that this muscle does not currently implement the ignore_tendon_compliance
 flag, nor the ignore_activation_dynamics flag.

<B>Nomenclature</B>

Note that dot notation is used to denote time derivatives (units of 
\f$m/s\f$ and \f$m/s^2\f$ in this case), where as the hat symbol 
(as in \f$\hat{l}\f$,\f$\hat{v}\f$) is used to denote time derivatives that have
been scaled by a characteristic dimension (appear in units of 
\f$1/s\f$ and \f$1/s^2\f$ in this case)

\li \f$m\f$: is the mass located at the junction between the fiber and the 
    tendon. This mass should be thought of as a time constant that indicates
    how quickly this model will converge to the force an equilibrium 
    muscle-tendon model would produce (\f$kg\f$)
\li \f$\ddot{x}\f$: is the acceleration of the mass, 
     in the \f$\hat{i}\f$ direction (\f$m/s^2\f$)
\li \f$F_{SE}\f$: is the force developed by the tendon (\f$N\f$)
\li \f$F_{CE}\cdot\hat{i}\f$: is the force developed by the fiber along the
    tendon (\f$N\f$)

\li \f$l_{CE}\f$: Length of the fiber(m)
\li \f$l_{CE,OPT}\f$: Length the fiber generates maximal isometric force (m)
\li \f$\hat{l}_{CE}=l_{CE}/l_{CE,OPT}\f$: 
        Normalized length of the fiber (dimensionless)
\li \f$\hat{v}_{CE}=\dot{l}_{CE}/l_{CE,OPT}\f$: 
        Fiber velocity divided by (\f$1/s\f$)
\li \f$\hat{v}_{MAX}\f$: 
        Maximum normalized fiber velocity (\f$l_{CE,OPT}/s\f$). This
                   quantity typically ranges between 10 and 15 lengths 
                   per second  (1/s)


\li \f$\phi\f$: Pennation angle(rad)
\li \f$\phi_{OPT}\f$: Pennation angle when the fiber is at its optimal 
                      length (rad)

\li \f$l_{SE}\f$: Length of the series element (tendon) (m)
\li \f$l_{SE,R}\f$: Resting length of the series element(m)
\li \f$\hat{l}_{SE} = l_{SE}/l_{SE,R}\f$: 
                Normalized length of the series element (dimensionless)
\li \f$ \hat{v}_{SE} = \dot{l}_{SE}/l_{SE,R}\f$: 
                    Normalized velocity of the tendon (1/s)

\li \f$f_{ISO}\f$: maximum force the muscle can develop statically 
(\f$\hat{v}_{CE}=0\f$) at its optimal length (\f$l_{CE,OPT}\f$) and 
pennation angle (\f$\phi_{OPT}\f$)

\li \f$\mathbf{a}\f$: activation (unitless)
\li \f$\mathbf{f}_L(\hat{l}_{CE})\f$: 
        Active force length multiplier (dimensionless)
\li \f$\mathbf{f}_V(\frac{\hat{v}_{CE}}{v_{MAX}})\f$: 
        Force velocity multiplier (dimensionless)
\li \f$\mathbf{f}_{PE}(\hat{l}_{CE})\f$: 
        Passive force length multiplier (dimensionless)
\li \f$\mathbf{f}_{K}(\hat{l}_{CE})\f$:
        Fiber compressive force length multiplier (dimensionless)
\li \f$\mathbf{f}_{c\phi}(\cos \phi)\f$:
        Fiber compressive cosine pennation multiplier (dimensionless)
\li \f$\mathbf{f}_{SE}(\hat{l}_{SE})\f$:
        Series element (tendon) force-length multiplier (dimensionless)

\li \f$\beta_{CE}\f$: Fiber damping (s)
\li \f$\beta_{PE}\f$: Fiber parallel element damping coefficient (s)
\li \f$\beta_{K}\f$: Fiber compressive force length damping coefficient (s)
\li \f$\beta_{c \phi}\f$: Fiber compressive cosine pennation damping 
                          coefficient (s)
\li \f$\beta_{SE}\f$: Series element (tendon) force length damping coefficient 
                      (s)




<B> References </B>

Hunt,K., and Crossley,F. Coefficient of restitution interpreted as damping in 
v
ibroimpact. Transactions of the ASME Journal of Applied Mechanics, 
42(E):440445, 1975.

J.He, W.S. Levine, and G.E. Leob."The Modelling of the 
Neuro-musculo-skeletal Control System of A Cat Hindlimb", 
Proceedings of the IEEE International Symposium on Intelligent Control, 1988.

@author Matt Millard
*/
class OSIMACTUATORS_API Millard2012AccelerationMuscle : public Muscle {
OpenSim_DECLARE_CONCRETE_OBJECT(Millard2012AccelerationMuscle, Muscle);

//class OSIMACTUATORS_API Millard2012AccelerationMuscle : public Muscle {

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY( default_activation, double,
                       "assumed initial activation level if none is assigned.");

    OpenSim_DECLARE_PROPERTY( default_fiber_length, double,
                       "assumed initial fiber length if none is assigned.");

    OpenSim_DECLARE_PROPERTY( default_fiber_velocity, double,
                       "assumed initial fiber velocity if none is assigned.");

    OpenSim_DECLARE_UNNAMED_PROPERTY( 
                                MuscleFirstOrderActivationDynamicModel,
                                "activation dynamics model with a lower bound");   
    
    OpenSim_DECLARE_UNNAMED_PROPERTY(
                                ActiveForceLengthCurve,
                                "active force length curve");

    OpenSim_DECLARE_UNNAMED_PROPERTY(
                                ForceVelocityCurve,
                                "force velocity curve");

    OpenSim_DECLARE_UNNAMED_PROPERTY(
                                FiberForceLengthCurve,
                                "fiber force length curve");

    OpenSim_DECLARE_UNNAMED_PROPERTY(
                                TendonForceLengthCurve,
                                "Tendon force length curve");

    //Should this be serialized at all? Normally an update to the left hand
    //point of the active force length curve should be accompanied by an update
    //to the engagment point of this curve. I bet many users might miss this
    //subtlety 
    OpenSim_DECLARE_UNNAMED_PROPERTY(
                                FiberCompressiveForceLengthCurve,
                                "fiber compressive force length curve");
    

    OpenSim_DECLARE_UNNAMED_PROPERTY(
                           FiberCompressiveForceCosPennationCurve,
                           "fiber compressive force cos(pennationAngle) curve");


    OpenSim_DECLARE_PROPERTY( fiber_damping, 
                            double,
                            "fiber damping coefficient");

    OpenSim_DECLARE_PROPERTY( fiber_force_length_damping, 
                            double,
                            "fiber force length damping coefficient");

    OpenSim_DECLARE_PROPERTY( fiber_compressive_force_length_damping, 
                         double,
                         "fiber compressive force length damping coefficient");

    OpenSim_DECLARE_PROPERTY( fiber_compressive_force_cos_pennation_damping, 
             double,
             "fiber compressive force cos(pennationAngle) damping coefficient");

    OpenSim_DECLARE_PROPERTY( tendon_force_length_damping, 
             double,
             "tendon force length damping coefficient");


    OpenSim_DECLARE_PROPERTY(   mass,
                                double,
                                "lumped mass");

    /**@}**/



//=============================================================================
// Construction
//=============================================================================
    /**Default constructor: produces a non-functional empty muscle*/
    Millard2012AccelerationMuscle();    

    /**Constructs a functional muscle using all of the default curves and
       activation model.

       @param aName The name of the muscle.

       @param aMaxIsometricForce 
        The force generated by the muscle when it at its optimal resting length,
        has a contraction velocity of zero, and is fully activated 
        (Newtons).
       
       @param aOptimalFiberLength
        The optimal length of the muscle fiber (meters).
                                
       @param aTendonSlackLength
        The resting length of the tendon (meters).

       @param aPennationAngle
        The angle of the fiber relative to the tendon when the fiber is at its
        optimal resting length (radians).
    */
    Millard2012AccelerationMuscle(const std::string &aName,
                                double aMaxIsometricForce,
                                double aOptimalFiberLength,
                                double aTendonSlackLength,
                                double aPennationAngle);

    


//==============================================================================
// Get Properties
//==============================================================================
   /**
   @param s the state of the system
   @return the normalized force term associated with the compressive force length
          element, \f$\mathbf{f}_K(\hat{l}_{CE})\f$, in the equilibrium equation
   */
   double getFiberCompressiveForceLengthMultiplier(SimTK::State& s) const;

   /**
   @param s the state of the system
   @return the normalized force term associated with the compressive force 
           cosine pennation element, \f$\mathbf{f}_{c\phi}(\cos \phi)\f$, in the 
           equilibrium equation
   */
   double getFiberCompressiveForceCosPennationMultiplier(SimTK::State& s) const;


   /**
   @param s the state of the system
   @return the normalized force term associated with tendon element,
            \f$\mathbf{f}_{SE}(\hat{l}_{T})\f$, in the equilibrium equation 
   */
   double getTendonForceMultiplier(SimTK::State& s) const;

   /**
   @return the size of the mass between the tendon and fiber
   */
   double getMass() const;

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

    /**
    @returns the ActiveForceLengthCurve that this muscle model uses
    */
    const ActiveForceLengthCurve& getActiveForceLengthCurve() const;

    /**
    @returns the ForceVelocityInverseCurve that this muscle model uses
    */
    const ForceVelocityCurve& getForceVelocityCurve() const;

    /**
    @returns the FiberForceLengthCurve that this muscle model uses
    */
    const FiberForceLengthCurve& getFiberForceLengthCurve() const;

    /**
    @returns the TendonForceLengthCurve that this muscle model uses
    */
    const TendonForceLengthCurve& getTendonForceLengthCurve() const;

    /**
    @returns the FiberCompressiveForceLengthCurve that this muscle model uses
    */
    const FiberCompressiveForceLengthCurve&
        getFiberCompressiveForceLengthCurve() const;

    /**
    @returns the FiberCompressiveForceCosPennationCurve that this muscle 
             model uses.
    */
    const FiberCompressiveForceCosPennationCurve& 
        getFiberCompressiveForceCosPennationCurve() const;

    /**
    @returns the stiffness of the muscle fibers along the tendon (N/m)
    */
    double getFiberStiffnessAlongTendon(const SimTK::State& s) const;

//==============================================================================
// Set Properties
//==============================================================================

    /**
    @param aActivationMdl the MuscleFirstOrderActivationDynamicModel that this
                          muscle model uses to simulate activation dynamics
    */
    void setActivationModel(
            MuscleFirstOrderActivationDynamicModel& aActivationMdl);

    /**
    @param aActiveForceLengthCurve the ActiveForceLengthCurve that this muscle
                            model uses to scale active fiber force as a function
                            of length
    */
    void setActiveForceLengthCurve(
            ActiveForceLengthCurve& aActiveForceLengthCurve);

    /**
    @param aForceVelocityCurve the ForceVelocityCurve that this
                            muscle model uses to calculate the derivative of
                            fiber length.
    */
    void setForceVelocityCurve(ForceVelocityCurve& aForceVelocityCurve);

    /**
    @param aFiberForceLengthCurve the FiberForceLengthCurve that this muscle 
                            model uses to calculate the passive force the muscle
                            fiber generates as the length of the fiber changes
    */
    void setFiberForceLengthCurve(
            FiberForceLengthCurve& aFiberForceLengthCurve);
    
    /**
    @param aTendonForceLengthCurve the TendonForceLengthCurve that this muscle
                            model uses to define the tendon force length curve
    */
    void setTendonForceLengthCurve(
            TendonForceLengthCurve& aTendonForceLengthCurve);

    /**
    @param aFiberCompressiveForceLengthCurve the 
            FiberCompressiveForceLengthCurve that this muscle model uses to 
            ensure the length of the fiber is always greater than a physically
            realistic lower bound.
    */
    void setFiberCompressiveForceLengthCurve(
            FiberCompressiveForceLengthCurve& 
            aFiberCompressiveForceLengthCurve);

    /**
    @param aFiberCompressiveForceCosPennationCurve the
            FiberCompressiveForceCosPennationCurve that this muscle model uses
            to prevent pennation angles from approaching 90 degrees, which is
            associated with a singularity in this model.
    */
    void setFiberCompressiveForceCosPennationCurve(
            FiberCompressiveForceCosPennationCurve& 
            aFiberCompressiveForceCosPennationCurve);

    /**
    @param mass
            The size of the mass parameter between the fiber and the tendon. 
            Making this parameter small will make the muscle model more rapidly
            converge to the results an equilibrium model would produce.
    
    <B>Conditions</B>
    \verbatim
        mass >= 0.001
    \endverbatim
    */
    void setMass(double mass);


//==============================================================================
// State Variable Related Functions
//==============================================================================

    /**
    @returns the default activation level that is used as an initial condition
             if none is provided by the user.
    */
    double getDefaultActivation() const;

    /**
    @returns the default fiber length that is used as an initial condition
             if none is provided by the user.
    */
    double getDefaultFiberLength() const;

    /**
    @returns the default fiber velocity that is used as an initial condition
             if none is provided by the user.
    */
    double getDefaultFiberVelocity() const;

    /**
    @param s The state of the system
    @returns the time derivative of activation
    */
    double getActivationRate(const SimTK::State& s) const;

    /**
    @param s The state of the system
    @returns the velocity of the fiber (m/s)
    */
    double getFiberVelocity(const SimTK::State& s) const;

    /**
    @param s The state of the system
    @returns the acceleration of the fiber (m/s)
    */
    double getFiberAcceleration(const SimTK::State& s) const;


    /**
    @param activation the default activation level that is used to initialize
           the muscle
    */
    void setDefaultActivation(double activation);

    /**
    @param fiberLength the default fiber length that is used to initialize
           the muscle
    */
    void setDefaultFiberLength(double fiberLength);

    /**
    @param fiberVelocity the default fiber velocity that is used to initialize
           the muscle
    */
    void setDefaultFiberVelocity(double fiberVelocity);

    /**
    @param s the state of the system
    @param activation the desired activation level
    */
    void setActivation(SimTK::State& s, double activation) const;

    /**
    @param s the state of the system
    @param fiberLength the desired fiber length (m)
    */
    void setFiberLength(SimTK::State& s, double fiberLength) const;

    /**
    @param s the state of the system
    @param fiberVelocity the desired fiber velocity (m/s)
    */
    void setFiberVelocity(SimTK::State& s, double fiberVelocity) const;

    /**
    @returns A string arraw of the state variable names
    */
    Array<std::string> getStateVariableNames() const FINAL_11;

    /**
    @param stateVariableName the name of the state varaible in question
    @returns The system index of the state variable in question
    */
    SimTK::SystemYIndex getStateVariableSystemIndex(
        const std::string &stateVariableName) const FINAL_11;

//==============================================================================
// Public Computations Muscle.h
//==============================================================================

    /**
    @param s the state of the system
    @returns the tensile force the muscle is generating in N
    */
    double computeActuation(const SimTK::State& s) const FINAL_11;


    /** This function computes the fiber length such that muscle fiber and 
        tendon are developing the same force, and so that the velocity of
        the entire muscle-tendon is spread between the fiber and the tendon
        according to their relative compliances.
        
        @param s the state of the system
    */
    void computeInitialFiberEquilibrium(SimTK::State& s) const FINAL_11;
        
///@cond TO BE DEPRECATED. 
    /*  Once the ignore_tendon_compliance flag is implemented correctly get rid 
        of this method as it duplicates code in calcMuscleLengthInfo,
        calcFiberVelocityInfo, and calcMuscleDynamicsInfo
    */
    virtual double calcInextensibleTendonActiveFiberForce(SimTK::State& s, 
                                       double aActivation) const FINAL_11;
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
///@endcond
    
protected:

    /**Related to scaling, soon to be removed.
    @param s the state of the model
    @param aScaleSet the scale set
    */
    void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);
    
    /**
    @param s the state of the model
    @return the time derivative of activation
    */
    double calcActivationRate(const SimTK::State& s) const; 


//==============================================================================
//Muscle Inferface requirements
//==============================================================================

    /**calculate muscle's position related values such fiber and tendon lengths,
    normalized lengths, pennation angle, etc... 
    @param s the state of the model
    @param mli the muscle length info struct that will hold updated information
               about the muscle that is available at the position stage
    */
    void calcMuscleLengthInfo(const SimTK::State& s, 
                              MuscleLengthInfo& mli) const FINAL_11;  
    

    /** calculate muscle's velocity related values such fiber and tendon 
        velocities,normalized velocities, pennation angular velocity, etc... 
    @param s the state of the model
    @param fvi the fiber velocity info struct that will hold updated information
               about the muscle that is available at the velocity stage

    */
    virtual void  calcFiberVelocityInfo(const SimTK::State& s, 
                                      FiberVelocityInfo& fvi) const FINAL_11;

    /** calculate muscle's active and passive force-length, force-velocity, 
        tendon force, relationships and their related values 
    @param s the state of the model
    @param mdi the muscle dynamics info struct that will hold updated 
            information about the muscle that is available at the dynamics stage    
    */
	void  calcMuscleDynamicsInfo(const SimTK::State& s, 
                                    MuscleDynamicsInfo& mdi) const FINAL_11;


	void calcMusclePotentialEnergyInfo(const SimTK::State& s,
		MusclePotentialEnergyInfo& mpei) const FINAL_11;
 
//==============================================================================
//ModelComponent Interface requirements
//==============================================================================

    /**Sets up the ModelComponent from the model, if necessary
    @param model the dynamic model
    */
    void connectToModel(Model& model) FINAL_11;

    /**Creates the ModelComponent so that it can be used in simulation
    @param system the multibody system
    */
	void addToSystem(SimTK::MultibodySystem& system) const FINAL_11;

    /**Initializes the state of the ModelComponent
    @param s the state of the model
    */
	void initStateFromProperties(SimTK::State& s) const FINAL_11;
    
    /**Sets the default state for ModelComponent
    @param s the state of the model
    */
    void setPropertiesFromState(const SimTK::State& s) FINAL_11;
	
    /**computes state variable derivatives
    @param s the state of the model
    */
    SimTK::Vector computeStateVariableDerivatives(
        const SimTK::State& s) const FINAL_11;
 
//==============================================================================
//State derivative helper methods
//==============================================================================

    /**
    Set the derivative of an actuator state, specified by name
    @param s the state    
    @param aStateName The name of the state to set.
    @param aValue The value to set the state to.
    */
    void setStateVariableDeriv( const SimTK::State& s, 
                                const std::string &aStateName, 
                                double aValue) const;
   
    /**
     Get the derivative of an actuator state, by index.
     @param s the state 
     @param aStateName the name of the state to get.
     @return The value of the state derivative
    */
	double getStateVariableDeriv(   const SimTK::State& s, 
                                    const std::string &aStateName) const;


private:
    //The name used to access the activation state
    static const std::string STATE_ACTIVATION_NAME;
    //The name used to access the fiber length state
	static const std::string STATE_FIBER_LENGTH_NAME;
    //The name used to access the fiber velocity state
	static const std::string STATE_FIBER_VELOCITY_NAME;

    //A struct that holds all of the necessary quantities to compute
    //the fiber and tendon force, acceleration, and stiffness
    struct AccelerationMuscleInfo;

    //This object defines the pennation model used for this muscle model
    MuscleFixedWidthPennationModel m_penMdl;

    //sets this class to null
    void setNull();

    //constructs all of the properties required to use this calss
    void constructProperties();
    
    /*Builds all of the components that are necessary to use this 
    muscle model in simulation*/
    void buildMuscle();

    /*Checks to make sure that none of the muscle model's properties have
    changed. If they have changed, then the muscle is rebuilt.*/
    void ensureMuscleUpToDate();

    /*       
    @param ami A struct that holds all of the necessary quantities to compute
                the fiber and tendon force, acceleration, and stiffness
    @return the force generated by the tendon (N)
    */
    double calcTendonForce(const AccelerationMuscleInfo& ami) const;  

    /**
    @param ami A struct that holds all of the necessary quantities to compute
                the fiber and tendon force, acceleration, and stiffness
    @return the stiffness of the tendon, that is the partial derivative of the
            tendon force with respect to length
    */
    double calcTendonStiffness(const AccelerationMuscleInfo& ami) const;

    /*
    @param a activation, the dimensionless parameter that ranges between 0 and 1
           that indicates how turned on the muscle is

    @param ami A struct that holds all of the necessary quantities to compute
                the fiber and tendon force, acceleration, and stiffness

    @return the force generated by the fiber (N), in the direction of the fiber
     in i, j components, where i is parallel to the tendon direction, and j
     is perpendicular to the tendon direction.
    */
    SimTK::Vec2 calcFiberForceIJ(double a, 
                              const AccelerationMuscleInfo& ami) const;    

    /*
    @param fiberForceIJ the force generated by the fiber (N), in the direction 
            of the fiber in i, j components, where i is parallel to the tendon 
            direction, and j is perpendicular to the tendon direction.

    @param ami A struct that holds all of the necessary quantities to compute
                the fiber and tendon force, acceleration, and stiffness

        
    */
    double calcFiberForce(  SimTK::Vec2 fiberForceIJ, 
                            const AccelerationMuscleInfo& ami) const;

   /*
    @param fiberForceIJ the force generated by the fiber (N), in the direction 
            of the fiber in i, j components, where i is parallel to the tendon 
            direction, and j is perpendicular to the tendon direction.
    @returns the fiber force projected along the direction of the tendon
   */
    double calcFiberForceAlongTendon(SimTK::Vec2 fiberForceIJ) const;  


    /*
    @param a activation, the dimensionless parameter that ranges between 0 and 1
           that indicates how turned on the muscle is

    @param ami A struct that holds all of the necessary quantities to compute
                the fiber and tendon force, acceleration, and stiffness

    @return the fiber stiffness (N/m), in the direction of the fiber
     in i, j components, where i is parallel to the tendon direction, and j
     is perpendicular to the tendon direction.

    */
    SimTK::Vec2 calcFiberStiffnessIJ(  double a, 
        const AccelerationMuscleInfo& ami) const;

    /*
    @param fiberForceIJ the force generated by the fiber (N), in the direction 
            of the fiber in i, j components, where i is parallel to the tendon 
            direction, and j is perpendicular to the tendon direction.

    @param fiberStiffnessIJ the fiber stiffness (N/m), in the direction of the fiber
     in i, j components, where i is parallel to the tendon direction, and j
     is perpendicular to the tendon direction.

    @param ami A struct that holds all of the necessary quantities to compute
                the fiber and tendon force, acceleration, and stiffness

    @return the stiffness of the fiber in the direction of the fiber
    */
    double calcFiberStiffness(  SimTK::Vec2 fiberForceIJ,
                                SimTK::Vec2 fiberStiffnessIJ, 
                                const AccelerationMuscleInfo& ami) const;

    /*
    @param dFmAT_dlce the partial derivative of fiber force along the tendon
                      for a small change in fiber length
    @param ami A struct that holds all of the necessary quantities to compute
                the fiber and tendon force, acceleration, and stiffness
    @return the stiffness (d_FmAT/d_lceAT) of the fiber in the direction of 
            the tendon.
    */
    double calc_DFiberForceAT_DFiberLengthAT(double dFmAT_dlce,                                               
                                      const AccelerationMuscleInfo& ami) const;

    /*
    @param fiberStiffnessIJ the fiber stiffness (N/m), in the direction of the fiber
     in i, j components, where i is parallel to the tendon direction, and j
     is perpendicular to the tendon direction.

    @return the partial derivative of fiber force along the tendon with respect
            to small changes in fiber length (in the direction of the fiber)
    */
    double calc_DFiberForceAT_DFiberLength( SimTK::Vec2 fiberStiffnessIJ) const;



    /*
    @param dFt_dtl the partial derivative of tendon force with respect to small
                    changes in tendon length (tendon stiffness) in (N/m)   
    @param ami A struct that holds all of the necessary quantities to compute
                the fiber and tendon force, acceleration, and stiffness
    @return the partial derivative of tendon force with respect to small changes 
            in fiber length
    */
    double calc_DTendonForce_DFiberLength(double dFse_dtl,
                                          const AccelerationMuscleInfo& ami,
                                          std::string& caller) const;

    //=====================================================================
    // Private Utility Class Members
    //      -Computes activation dynamics and fiber kinematics
    //=====================================================================

    /*
    @param s the system state
    @param aActivation the initial activation of the muscle
    @param aSolTolerance the desired relative tolerance of the equilibrium 
           solution
    @param aMaxIterations the maximum number of Newton steps allowed before
           attempts to initialize the model are given up, and an exception is 
           thrown.
    @param aNewtonStepFraction the fraction of a Newton step to take at each
           update
    */
    SimTK::Vector initMuscleState(SimTK::State& s, double aActivation,
                             double aSolTolerance, int aMaxIterations,
                             double aNewtonStepFraction) const;
    


    /*
    This can only be called at the velocity stage at least.
    @param ami      An empty AccelerationMuscleInfo struct
    @param lce      fiber length (m)
    @param dlce_dt  fiber lengthening velocity (m/s)
    @param phi      fiber pennation angle (rad)
    @param dphi_dt  fiber pennation angular velocity (rad/s)
    @param tl       tendon length (m)
    @param dtl_dt   tendon lengthening velocity (m/s)
    @param fal      fiber active force length multiplier (dimensionless)
    @param fv       fiber force velocity multiplier (dimensionless)
    @param fpe      fiber passive force length multiplier (dimensionless)
    @param fk       fiber compressive force length multiplier (dimensionless)
    @param fcphi    fiber compressive force cos pennation multiplier 
                    (dimensionless)
    @param fse      tendon force length multiplier (dimensionless)
    */
   void calcAccelerationMuscleInfo(
                  AccelerationMuscleInfo& ami,
                                             double lce, 
                                             double dlce_dt,
                                             double phi,
                                             double dphi_dt,
                                             double tl,
                                             double dtl_dt,
                                             double fal,
                                             double fv,
                                             double fpe,
                                             double fk,
                                             double fcphi,
                                             double fse) const;

    struct AccelerationMuscleInfo {   
        //Kinematic Quantities
        double lce;             //fiber length (m)
        double dlce_dt;         //fiber lengtening velocity (m/s)
        double lceAT;           //fiber length along the tendon (m)
        double dlceAT_dlce;     //change in fiber length along the tendon with
                                //a small change in fiber length (m/m)
        double dlceAT_dt;       //velocity of the fiber along the tendon (m/s)
        double d_dlceATdt_dlce; //change of the velocity of the fiber along
                                //the tendon with a small change in fiber length
                                //(m/s)/m
        double phi;         //pennation angle (rad)
        double cosphi;      //cosine of the pennation angle
        double sinphi;      //sine of the pennation angle
        double dphi_dt;     //pennation angular velocity (rad/s)
        double dphi_dlce;   //the partial derivative of pennation angle w.r.t
                            // fiber length (rad/m)
        double d_dphidt_dlce;//The partial derivative of the pennation angular
                            //velocity w.r.t small changes in fiber length
        double tl;      //tendon length (m)
        double dtl_dt;  //tendon lengthening velocity (m/s)
        double dtl_dlce;//the change in tendon length for a small change in
                        //fiber length (m/m)

        //Elastic Force Multipliers
        double fse;  //tendon force length elastic multiplier 
        double fal;  //fiber active force length elastic multiplier
        double fv;   //fiber force velocity multiplier
        double fpe;  //fiber force length elastic multiplier
        double fk;   //fiber compressive force length elastic multiplier
        double fcphi;//fiber compressive force cos pennation elastic multiplier

        //Visco force multipliers
        double fseV;  //tendon force length damping multiplier 
        double fpeV;  //fiber force length damping multiplier
        double fkV;   //fiber compressive force length damping multiplier
        double fcphiV;//fiber compressive force cos pennation damping multiplier
        double fibV;  //fiber damping

        //Visco elastic force multiplier
        double fseVEM;  // ' ' visco elastic multiplier   
        double fpeVEM;
        double fkVEM;
        double fcphiVEM;

        //Partial derivative of the elastic multipliers 
        // w.r.t. their input variable
        double dfse_dtl;    //d(tendon force length multipler) d(tendonLength) 

        double dfal_dlce;   //d(fiber active force length multiplier)
                            //d(fiberLength)

        //dfv_dlce = 0 for the force velocity curve being used
        double dfpe_dlce;   //d(fiber force length multiplier)d(fiberLength)

        double dfk_dlce;    //d(fiber compressive force length multiplier)
                            //d(fiberLength)

        double dfcphi_dlce; //d(fiber compressive force cos pennation multiplier)
                            //d(fiberLength

        //Partial derivative of the damping multipliers 
        // w.r.t. their input variable
        double dfseV_dtl;     //d(tendon force length damping multiplier)
                              //d(tendon length)

        double dfpeV_dlce;    //d(fiber force length damping multiplier)
                              //d(fiberLength)

        double dfkV_dlce;     //d(fiber compressive force 
                              //  length damping  multiplier)
                              //d(fiberLength)

        double dfcphiV_dlce;  //d(fiber compressive force 
                              //    cos pennation multiplier)
                              //d(fiberLength)

        double dfibV_dlce;

        //Partial derivative of the visco elastic multiplier 
        //w.r.t. the input variable
        double dfseVEM_dtl;   //d(tendon force length visco elastic multiplier)
                              //d(tendon length)

        double dfpeVEM_dlce; //d(fiber force length viscoelastic multiplier)
                             //d(fiber length)

        double dfkVEM_dlce; //d(fiber compressive force length 
                            //        viscoelastic multiplier)
                            //d(fiber length)

        double dfcphiVEM_dlce;  //d(fiber compressive force cos pennation
                                //                viscoelastic multiplier)
                                //d(fiber length)

		AccelerationMuscleInfo(): 
            lce(SimTK::NaN),
            dlce_dt(SimTK::NaN),
            lceAT(SimTK::NaN),
            dlceAT_dlce(SimTK::NaN),
            dlceAT_dt(SimTK::NaN),            
            d_dlceATdt_dlce(SimTK::NaN),
            phi(SimTK::NaN),
            cosphi(SimTK::NaN),
            sinphi(SimTK::NaN),
            dphi_dlce(SimTK::NaN),
            dphi_dt(SimTK::NaN),
            d_dphidt_dlce(SimTK::NaN),
            tl(SimTK::NaN),
            dtl_dt(SimTK::NaN),
            dtl_dlce(SimTK::NaN),
                fse(SimTK::NaN),   
                fal(SimTK::NaN),
                fv(SimTK::NaN),
                fpe(SimTK::NaN),     
                fk(SimTK::NaN),      
                fcphi(SimTK::NaN),
            fseV(SimTK::NaN),                 
            fpeV(SimTK::NaN),     
            fkV(SimTK::NaN),      
            fcphiV(SimTK::NaN), 
            fibV(SimTK::NaN),
                fseVEM(SimTK::NaN),              
                fpeVEM(SimTK::NaN),
                fkVEM(SimTK::NaN),
                fcphiVEM(SimTK::NaN),
            dfse_dtl(SimTK::NaN),    
            dfal_dlce(SimTK::NaN),   
            dfpe_dlce(SimTK::NaN),   
            dfk_dlce(SimTK::NaN),    
            dfcphi_dlce(SimTK::NaN), 
                dfseV_dtl(SimTK::NaN),                 
                dfpeV_dlce(SimTK::NaN),   
                dfkV_dlce(SimTK::NaN),    
                dfcphiV_dlce(SimTK::NaN),
                dfibV_dlce(SimTK::NaN),
            dfseVEM_dtl(SimTK::NaN),                
            dfpeVEM_dlce(SimTK::NaN),
            dfkVEM_dlce(SimTK::NaN),
            dfcphiVEM_dlce(SimTK::NaN){};
		friend std::ostream& operator<<(std::ostream& o, 
            const AccelerationMuscleInfo& ami) {
			o << "Millard2012AccelerationMuscle::"
                "AccelerationMuscleInfo should not be serialized!" 
              << std::endl;
			return o;
		}
    };


};    

} // end of namespace OpenSim

#endif // __Millard2012AccelerationMuscle_h__
