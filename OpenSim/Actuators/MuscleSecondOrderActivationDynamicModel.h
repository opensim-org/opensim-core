#ifndef OPENSIM_MUSCLESECONDORDERACTIVATIONDYNAMICMODEL_H_
#define OPENSIM_MUSCLESECONDORDERACTIVATIONDYNAMICMODEL_H_
/* -------------------------------------------------------------------------- *
 *             OpenSim:  MuscleSecondOrderActivationDynamicModel.h             *
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
//=============================================================================
// INCLUDES
//=============================================================================

//#include <string>
//#include <SimTKcommon/internal/Function.h>
#include "Simbody.h"


#include <OpenSim/Common/Object.h>
#include <OpenSim/Actuators/osimActuatorsDLL.h>

namespace OpenSim {

    /**
    This function is a muscle modeling utility class that simulates
    dynamics of muscle force development as a function of input excitation.
    Though it has been long been assumed that a second-order, critically damped 
    filter models the twitch response of muscle very accurately (Bellmare et al)
    first order models have persisted in the literature. Zajac originally chose
    to use first order, rather than second order equations in an effort to 
    reduce simulation times. First order activation dynamics were later 
    popularized in two influential muscle modeling papers by Winters in 1995, 
    and later by Thelen in 2003. Computing power is far greater today than it 
    was at the time Zajac wrote his influential muscle modeling paper, and so it
    makes sense to include activation dynamics that more closely match the
    experimental literature.
    
    The figure shows that
    there are very large differences in the impulse response between a first 
    order system (filled grey area), and Bellmare et al.'s experimental data
    (thick grey line). The first order system has an instantaneous rise time,
    reaches a peak value when the impulse terminates (in 0 time). 
    Additionally the impulse response of the second order system is 
    orders of magnitude higher than the second order system (peaking at a 
    value of just under 0.01), and so, its peak is not shown in the figure.
    The second order system peaks at the same time as Bellmare et al.'s 
    experimental data, and then slowly decays. These plots suggest that a 
    second order system decays far slower than the experiments indicate.

    \image html fig_MuscleSecondOrderActivationDynamicModel.png
        
    The quality of match between the second order model and the experimental data
    can be improved by having separate time constants for rising 
    (\f$\dot{a} > 0\f$) and falling (\f$\dot{a} < 0\f$). This small change can 
    be easily encorporated into a canonical second order ODE of a damped system 
    with a forced input where \f$a\f$ is activation, \f$\zeta\f$ is damping, 
    \f$\omega\f$ is the natural frequency of the system, \f$\mu\f$ is excitation 
    (bounded by 0 and 1).

    \f[
    \ddot{a} + 2 \zeta \omega \dot{a} + \omega^2 a = \omega^2\mu
    \f]

    For a critically damped system \f$\zeta = 1\f$. To obtain a quicker rate of
    decay a the value of \f$\omega\f$ changes depending on the sign of 
    \f$\dot{a}\f$:

    if \f$\dot{a}>0\f$
    \f[
        \omega = \frac{1}{T} 
    \f]

    else if \f$\dot{a}<0\f$

    \f[
        \omega = \frac{2}{T}                                          
    \f]
    
    where \f$ T \f$ is the contraction time of the muscle. Values of \f$ T \f$ vary
    from muscle to muscle, depending mainly on their fiber composition. 

    Although the above second order system approximates the twitch response of
    muscle very well, it has a lower bound of 0. The conventional muscle model
    formulation requires that activation smoothly approach a lower bound that is
    above zero. This can be achieved by rescaling activation, and the lower
    bound:

    \f[
        \hat{a} = \frac{a}{1-a_{min}} 
    \f]
    \f[
        \hat{a}_{min} = \frac{a_{min}}{1-a_{min}}         
    \f]

    Subsituting in the rescaled activation and lower bound into the
    second order system yields

    \f[
       \ddot{a}+2\zeta\omega\dot{a}+\omega^2(\hat{a}-\hat{a}_{min})=\omega^2\mu
    \f]

    This is the activation dynamic equation that this class implements as it
    matches Bellmare et al.'s experimental data quite well, and has a 
    user-settable lower bound.

    <B>Default</B>


    The default time to peak (or twitchTimeConstant) is 0.050 seconds. This is
    on the faster end of skeletal muscles. Bellare et al. reported values of 
    0.0657, 0.071 and 0.116 for the biceps brachaii, adductor pollicus, and the
    solues respectively.


    <B>References</B>

    Bellmare, F., Woods, JJ., Johansson,R., and Bigland-Ritchie,B (1983). 
    Motor-unit discharge rates in maximal voluntary contractions of three human 
    muscles. J. Neurophysiology(50), pp. 1380-1392.

    Thelen, DG.(2003), Adjustment of Muscle Mechanics Model
    Parameters to Simulate Dynamic Contractions in Older Adults. 
    ASME Journal of Biomechanical Engineering (125).

    Winters, JM (1995). An Improved Muscle-Reflex Actuator for Use in 
    Large-Scale Neuromusculoskeletal Models. Annals of Biomedical Engineering
    (25), pp. 359-374.

    Zajac, FE (1989). Muscle and Tendon: Properties, Models, Scaling and 
    Application to Biomechanics and Motor Control. Critical Reviews in 
    Bimedical Engineering (17), pp. 359-410.

    <B>Computational Cost Details</B>
    All computational costs assume the following operation costs:

    \verbatim
    Operation Type   : #flops
    +,-,=,Boolean Op : 1 
                   / : 10
                 sqrt: 20
                 trig: 40
    \endverbatim

    @author Matt Millard
    @version 0.0
    */      
//class MuscleSecondOrderActivationDynamicModel : public Object{
class OSIMACTUATORS_API MuscleSecondOrderActivationDynamicModel:public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleSecondOrderActivationDynamicModel, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(twitch_time_constant, double,
        "time-to-peak after an impulse (s)");    
    OpenSim_DECLARE_PROPERTY(minimum_activation, double,
        "activation lower bound");
    //OpenSim_DECLARE_PROPERTY(minimum_activation, double,
    //    "minimum activation allowed");                       
    /**@}**/
    
//==============================================================================
// PUBLIC METHODS
//==============================================================================

    /**
    @param twitchTimeConstant   The time-to-peak, in seconds, in response to an 
                                impulse.

    @param minActivation    The minimum activation allowed. Equilibrium 
                            muscle models might set this value to be between
                            0.01-0.1, as they have a singularity when 
                            a = 0. Muscle models that don't have a 
                            singularity at a=0 will set minActivation to be
                            0. (Unitless).

    @param muscleName       The name of the muscle that this activation 
                            object belongs to. This string is used to
                            create useful exception messages.

    <B>Conditions</B>
    \verbatim
        0 < twitchTime 
        0 <= minActivation < 1
    \endverbatim

    <B>Computational Cost</B>
    \verbatim
        ~15 flops
    \endverbatim

    */
    MuscleSecondOrderActivationDynamicModel(double twitchTimeConstant,                                             
                                            double minActivation,
                                            const std::string& muscleName);
    
    ///Default constructor. Sets data members to NAN and other error
    ///causing values
    MuscleSecondOrderActivationDynamicModel();


    /**
    @param dactivation_dt   The derivative of activation

    @param excitation   The excitation signal being sent to the muscle 
                        (Unitless, [0,1]). This value will be clamped between
                        0 and 1-minActivation.

   

    @param activation       The current activation of the muscle(Unitless [0,1])
                            This value will be clamped to its permitted range
                            between minActivation and 1.
    @returns the time derivative of activation

    <B>Conditions</B>
    \verbatim
        0 <= excitation <= 1
    \endverbatim

    <B>Computational Cost</B>
    \verbatim
    \endverbatim
    */
    double calcDerivative(  double dactivation_dt, 
                            double activation,                           
                            double excitation) const;


    /**        
    @returns The time to peak of the impulse response of the muscle (sec).
        
    <B>Computational Cost</B>
    \verbatim
        ~1 flops
    \endverbatim
    */
    double getTwitchTimeConstant() const;
        

    /**
    @returns The minimum activation level
        
    <B>Computational Cost</B>
    \verbatim
        ~1 flops
    \endverbatim
    */
    double getMinimumActivation() const;


    /**
    @returns The maximum activation level
        
    <B>Computational Cost</B>
    \verbatim
        ~1 flops
    \endverbatim
    */
    double getMaximumActivation() const;

    /**
    @returns activation that has been clamped to a legal range, that is between
             minActivation specified in the constructor and 1.0
    <B>Computational Cost</B>
    \verbatim
        ~2 flops
    \endverbatim
    */
    double clampActivation(double activation) const;
    

    /**        
    @param activationTimeConstant The activation time constant in 
                                  units of seconds
    @returns a bool that indicates if the value was set or not        

    <B>Computational Cost</B>
    \verbatim
        ~1 flops
    \endverbatim
    */
    bool setTwitchTimeConstant(double activationTimeConstant);
        
           
    /**
    @returns The minimum activation level
    @returns a bool that indicates if the value was set or not        
        
    <B>Computational Cost</B>
    \verbatim
        ~1 flops
    \endverbatim
    */
    bool setMinimumActivation(double minimumActivation);

    ///@cond
    /*This is useful for testing purposes only. Don't even think
        about using these functions!*/
    double calcValue(const SimTK::Vector& x) const; /*virtual*/
    double calcDerivative(const SimTK::Array_<int>& derivComponents, 
                            const SimTK::Vector& x) const; /*virtual*/ 
    int getArgumentSize() const;  /*virtual*/ 
    int getMaxDerivativeOrder() const;  /*virtual*/ 
    ///@endcond

    void ensureModelUpToDate();
    private:      
        void buildModel();
           
        void setNull();
        void constructProperties();
};

}
#endif //OPENSIM_MUSCLESECONDORDERACTIVATIONDYNAMICMODEL
