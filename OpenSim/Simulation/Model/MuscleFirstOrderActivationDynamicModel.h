#ifndef OPENSIM_MUSCLEFIRSTORDERACTIVATIONDYNAMICMODEL_H_
#define OPENSIM_MUSCLEFIRSTORDERACTIVATIONDYNAMICMODEL_H_
//MuscleFirstOrderActivationDynamicModel.h
// Author: Matthew Millard
/*
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
//=============================================================================
// INCLUDES
//=============================================================================

//#include <string>
//#include <SimTKcommon/internal/Function.h>
#include "Simbody.h"

#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace OpenSim {

    /**
    This function is a muscle modeling utility class that computes the 
    derivative of activation with respect to time time using a modification 
    of the activation model presented in Thelen 2003. The activation model 
    presented by Thelen in Eqns. 1 and 2 in the Appendix closely follows 
    the activation dynamic model found in Winters 1995 (Eqn. 2, line 2, Eqn 3) 
    where the time derivative of activation (da/dt) is equal to the difference 
    between excitation (u) and activation (a) scaled by a variable time-constant 
    (tau(a,u)).
    
    \verbatim
        da/dt    = (u-a)/tau(a,u)                                           (1)
    \endverbatim

    The primary difference between the activation model presented by Thelen and
    the one presented by Winters lies in the expression for tau(a,u), which 
    differ only in the values for the constant terms within the braces:
    
    \verbatim
                     t_act *(0.5+1.5*a) : u > a                             (2)
        tau(a,u) = {                                         
                     t_dact/(0.5+1.5*a) : u <= a                            (3)
    \endverbatim

    This model for activation dynamics presented in Eqn. 1 notably does not 
    respect a lower bound for activation. Equilibrium muscle models (used 
    ubiquituously to model muscle in lumped parameter musculoskeletal 
    simulations) have a singularity in their state equations when activation 
    goes to zero, making the above activation dynamic model unsuitable for 
    simulating using equilibrium muscle models.

    Equation 1 can be made to respect a lower bond on activation by introducing
    a lower bound, a_low, scaling activation to range from amin to 1

    \verbatim
        aS       = a/(1-amin)                                               (4)
        aminS    = amin/(1-amin)                                            (5)
    \endverbatim

    , adjusting the calculation for the time constant to use the scaled 
    activation

    \verbatim
                           t_act *(0.5+1.5*(aS-aminS)) : u > aS-asminS      (6)
        tau(a,u,asmin) = {                                         
                           t_dact/(0.5+1.5*(aS-aminS)) : u <= aS-asminS     (7)
    \endverbatim

    and finally updating the final expression for the derivative of activation

    \verbatim
        da/dt    = (u-(aS-aminS))/tau(a,u,amin)                             (8)
    \endverbatim
    

    The phase portraits of this system with an aMin>0 is very similar to that of 
    the activation dynamics equations that Thelen presented (which result when
    aMin=0), though there are differences: the left two panels of the figure 
    below shows a phase portrait of the Thelen activation model (dotted lines) 
    against one with a lower bound of 0.05, which are slightly different.
    The step response of this model is also very similar, however, there are 
    differences between the Thelen activation model (shown in the figure below 
    on the right panel in balc) and one with a lower bound of 0.05 (shown in 
    green), particularly when the activation level approaches the minimum value.
    All plots were generated with an activation time constant of 0.010s and a 
    deactivation time constant of 0.040s. 
    
    The step response of the Thelen activation function and the modified version
    are also shown below. Note that due to the nonlinear nature of the state
    equations, the time activation and deactivation time constants do not relate 
    directly to the rise and fall time of the step response. The 10%-90% rise 
    time of both models is virtually identical: 30ms from 10%-90%, and then 86ms
    from 90%-10%. Dividing these times by the rising and falling time constants 
    respectively yields a rise time of 3*t_act, and 2.17*t_dact.

    \image html activationDynamics.png

    <B>References</B>

    Thelen, DG.(2003), Adjustment of Muscle Mechanics Model
    Parameters to Simulate Dynamic Contractions in Older Adults. 
    ASME Journal of Biomechanical Engineering (125).

    Winters, JM (1995). An Improved Muscle-Reflex Actuator for Use in 
    Large-Scale Neuromusculoskeletal Models. Annals of Biomedical Engineering
    (25), pp. 359-374.

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
    class OSIMSIMULATION_API MuscleFirstOrderActivationDynamicModel 
        : public SimTK::Function_<double>
    {
        //It occurs to me that it doesn't make sense to have this in a
            //function object, because the derivative I'm computing is not
            //the derivative of the function w.r.t. the 0th parameter,
            //activation, but time. Hmm. This interface is convenient for 
            //other things though, particularly for testing (I have written
            //lots of test code that works with the SimTK::Function interface.)
            //
            //Given that the guts of the interface are private, that is good
            //enough justification for me, for now.

        private:
            const double m_ta; //activation time constant
            const double m_td; //deactivation time constant
            const double m_minA; //minimum activation level
            double m_minAS; //scaled version of m_minA
            std::string m_name;

        public:
        /**
        @param tauActivation The first order time constant associated with
                             a muscle that is being activated (units of seconds) 
                             A typical value is 0.010, or 10 ms.

        @param tauDeactivation  The first order time constant associated with a
                                muscle that is turning off, or being 
                                deactivated (units of seconds). A typical value
                                is 0.040 or 40 ms.

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
            0 < tauActivation 
            0 < tauDeactivation 
            0 <= minActivation < 1
        \endverbatim

        <B>Computational Cost</B>
        \verbatim
            ~15 flops
        \endverbatim

        */
        MuscleFirstOrderActivationDynamicModel(double tauActivation, 
                                               double tauDeactivation, 
                                               double minActivation,
                                               const std::string& muscleName);
        
        /**
        @param excitation The excitation signal being sent to the muscle 
                          (Unitless, [0,1])
        @param activation   The current activation of the muscle(Unitless [0,1])
        @returns the time derivative of activation

        <B>Conditions</B>
        \verbatim
            0 <= excitation <= 1
        \endverbatim

        <B>Computational Cost</B>
        \verbatim
            ~40 flops
        \endverbatim
        */
        double calcDerivative(double activation, double excitation) const;


        /**        
        @returns The activation time constant in (units of seconds)
        
        <B>Computational Cost</B>
        \verbatim
            ~1 flops
        \endverbatim
        */
        double getActivationTimeConstant() const;
        
        /**        
        @returns The deactivation time constant in (units of seconds)
        
        <B>Computational Cost</B>
        \verbatim
            ~1 flops
        \endverbatim
        */
        double getDeactivationTimeConstant() const;
        
        /**
        @returns The minimum activation level
        
        <B>Computational Cost</B>
        \verbatim
            ~1 flops
        \endverbatim
        */
        double getMinActivation() const;

        /**
        @returns the name of this activation object
        
        <B>Computational Cost</B>
        \verbatim
            ~1 flops
        \endverbatim
        */
        std::string getName() const;

        private:
           /**
           
           */
            double calcValue(const SimTK::Vector& x) const; /*virtual*/

           /** 
           
           */
           double calcDerivative(const SimTK::Array_<int>& derivComponents, 
                                   const SimTK::Vector& x) const; /*virtual*/ 

           /**
           @returns 2, activation and excitation in that order
           */
           int getArgumentSize() const;  /*virtual*/ 

           /**
           @returns 1
           */ 
           int getMaxDerivativeOrder() const;  /*virtual*/ 


    };

}
#endif //OPENSIM_MUSCLEFIRSTORDERACTIVATIONDYNAMICMODEL