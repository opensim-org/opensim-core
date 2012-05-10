#ifndef OPENSIM_TENDON_FORCE_LENGTH_CURVE_H_
#define OPENSIM_TENDON_FORCE_LENGTH_CURVE_H_

/* Author: Matthew Millard
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

// INCLUDE
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/MuscleCurveFunctionFactory.h>
#include <OpenSim/Common/MuscleCurveFunction.h>
#include <simbody/internal/common.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

//==============================================================================
//                       TENDON FORCE LENGTH CURVE
//==============================================================================

/**
 This class serves as a serializable TendonForceLengthCurve, for use in 
 muscle models. The user has control over the strain the tendon undergoes at 
 1 unit load (e0), its stiffness at a strain of e0, and the shape of the tendon 
 curve (its `curviness') using the following parameters: 
 
  <B>Manditory Properties</B>
 \li strainAtOneNormForce

 <B>Optional Properties</B>
 \li stiffnessAtOneNormForce
 \li curviness

  All parameters but the strain of the tendon at 1 unit load (e0) are optional. 
 Note that both optional parameters must either be left blank, or filled in. 
 Filling in one optional parameter but not the other will throw an exception
 when the curve is built.

  The shape of the curve can be varied from a close approximation of a line to a
  sharply bent curve using the optional parameters. 

 \image html fig_TendonForceLengthCurve.png
 
 If the optional parameters are not provided, the tendon force length curve is
 fitted to a linearly extrapolated exponential curve (Thelen 2003) so that the
 following mathematical properties match between the two curves:
 
 \li The strain (e0) of the two curves under 1 unit force of tension
 \li The slope of the curve at a strain of e0 of the two curves 
 \li The area enclosed by the curves between a strain of 0 and e0 of the two
     curves match with a small relative error.

    An example of a fitted curve can be seen in the figure below.

 \image html fig_TendonForceLengthCurve_Fitted.png

 For more details on the fitting process see functions:

 \li TendonForceLengthCurve(double strainAtOneNormForce,const std::string& muscleName)
 \li getStiffnessAtOneNormForceInUse()
 \li getCurvinessInUse()

 <B> References </B>
    Thelen (2003). Adjustment of Muscle Mechanics Model Paramters to Simulate 
    Dynamic Contractions in Older Adults. ASME J Biomech Eng (125).

 <B>Computational Cost Details</B>
    All computational costs assume the following operation costs:

    \verbatim
    Operation Type     : #flops
    *,+,-,=,Boolean Op : 1 
                     / : 10
                   sqrt: 20
                   trig: 40
    \endverbatim

    These relative weightings will vary processor to processor, and so any of 
    the quoted computational costs are approximate.

  @author Matt Millard

 */
class OSIMACTUATORS_API TendonForceLengthCurve : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(TendonForceLengthCurve, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(strain_at_one_norm_force, double,
        "tendon strain at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_one_norm_force, double, 
        "tendon stiffness at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(curviness, double, 
        "tendon curve bend, from linear to maximum bend (0-1)");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor creates an object with a default name that doesn't
    yet define a curve. Calling this function is equivalent to creating a fitted
    curve using the default strainAtOneNormForce
    
    <B>Default Parameters</B>
    \verbatim
        strainAtOneNormForce    = 0.04             
    \endverbatim

    **/
    TendonForceLengthCurve();

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /**
     Constructs a C2 continuous tendon force length curve. 
     
    
        @param strainAtOneNormForce
                    The tendon strain at which the tendon develops 1 unit of 
                    normalized force. The definition of strain used for this 
                    quantity is consistent with the Cauchy or engineering 
                    definition of strain: strain = (l-l0)/l0, where l is length,
                    and l0 is resting length. In this context 
                    strainAtOneNormForce = 0.04 means that 
                    the tendon will develop a tension of 1 normalized force when 
                    it is strained by 4% of its resting length, or 
                    equivalently is stretched to 1.04 times its resting length.

        @param stiffnessAtOneNormForce     
                        The normalized stiffness (or slope) of the tendon curve 
                        when the tendon is strained by strainAtOneNormForce
                        under a load of 1 normalized unit of force.

        @param curviness    
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.

        @param muscleName
                The name of the muscle this curve belongs to. This name is used
                to create the name of this curve, which is formed simply by 
                appending "_TendonForceLengthCurve" to the string in muscleName.
                This name is used for making intelligible error messages and 
                also for naming the XML version of this curve when it is 
                serialized.

         \image html fig_TendonForceLengthCurve.png

         This curve has the 
     advantage of being C2 continuous which results in faster simulations when
     compared to the popular method of using a linearly extrapolated 
     exponential curve to parameterize the tendon force length curve, which 
     is only C0 continuous. Details to appear in Millard et al 2012.

      <B>Conditions:</B>
        \verbatim
            strainAtOneNormForce > 0
            stiffnessAtOneNormForce > 1/strainAtOneNormForce
            0 <= curviness <= 1
        \endverbatim

        <B>Computational Costs</B>
        \verbatim 
            ~174,100 flops
        \endverbatim

        <B>Example:</B>
        @code
            TendonForceLengthCurve fseCurve3(0.10,50,0.75,"soleus");
            double fseVal  = fseCurve3.calcValue(0.02);
            double dfselVal = fseCurve3.calcDerivative(0.02,1);
        @endcode



    */
    TendonForceLengthCurve( double strainAtOneNormForce, 
                            double stiffnessAtOneNormForce,
                            double curviness,
                            const std::string& muscleName);

    /**
     Constructs a C2 continuous tendon force length curve that is fitted to a
     conventional linearly extrapolated tendon curve. 
        
        @param strainAtOneNormForce
                The tendon strain at which the tendon develops 1 unit of 
                normalized force. The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtOneNormForce = 0.04 means that 
                the tendon will develop a tension of 1 normalized force when 
                it is strained by 4% of its resting length, or 
                equivalently is stretched to 1.04 times its resting length.


        @param muscleName
                The name of the muscle this curve belongs to. This name is used
                to create the name of this curve, which is formed simply by 
                appending "_TendonForceLengthCurve" to the string in muscleName.
                This name is used for making intelligible error messages and 
                also for naming the XML version of this curve when it is 
                serialized.


           \image html fig_TendonForceLengthCurve_Fitted.png

     This curve has the 
     advantage of being C2 continuous which results in faster simulations when
     compared to the popular method of using a linearly extrapolated 
     exponential curve to parameterize the tendon force length curve, which 
     is only C0 continuous. Details to appear in Millard et al 2012. 
     
     This constructor will create a C2 continuous tendon force length curve that
     closely approximates the commonly used linearly extrapolated exponental
     curve as documented in Thelen 2003. The resulting curve is fitted to the
     linearly extrapolated exponential curve so that
     
     
     \li The strain (e0) of the two curves under 1 unit force of tension is identical
     \li The slope of the curve at a strain of e0 of the two curves is identical 
     \li The area enclosed by the curves between a strain of 0 and e0 of the two
         curves match with a small error.
     
     The functions 'getStiffnessAtOneNormForceInUse()', and 'getCurvinessInUse()'
     are used to fit this tendon curve to the linearly extrapolated exponential
     curve. See the documentation of these functions for more details on how
     these parameters are computed.

      <B>Conditions:</B>
        \verbatim
            strainAtOneNormForce > 0
        \endverbatim

        \verbatim
            ~870,080 flops
        \endverbatim

        <B>Default Parameter Values</B>
         \verbatim
             strainAtOneNormForce    = 0.04            
         \endverbatim

        <B>Example:</B>
        @code
            TendonForceLengthCurve fseCurve3(0.10,"soleus");
            double fseVal  = fseCurve3.calcValue(1.02);
            double dfselVal = fseCurve3.calcDerivative(1.02,1);
        @endcode


        <B>References:</B>
        Thelen, DG (2003). Adjustment of Muscle Mechanics Model 
         Parameters to Simulate Dynamic Contractions in Older Adults. 
         ASME J.Biomech. Eng., 125, 75-77)


    */
    TendonForceLengthCurve( double strainAtOneNormForce,                             
                            const std::string& muscleName);

    /**
    @returns    The tendon strain at which the tendon develops 1 unit of 
                normalized force. The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtOneNormForce = 0.04 means that the tendon will develop 
                a tension of 1 normalized force when it is strained by 4% of 
                its resting length, or equivalently is stretched to 1.04 times 
                its resting length.
    */
     double getStrainAtOneNormForce();

     
     /**
     @returns   The normalized stiffness (or slope) of the tendon curve 
                when the tendon is strained by strainAtOneNormForce
                under a load of 1 normalized unit of force.

                This is an optional parameter, and so, if this parameter is 
                empty, a call to this function will throw an exception. If
                you're not sure if its been set, you can make a call to the
                corresponding set function before hand, or call the 
                corresponding '...InUse()' function.
     */
     double getStiffnessAtOneNormForce();
     
     /**
        @returns The normalized stiffness (or slope) of the tendon curve 
                when the tendon is strained by strainAtOneNormForce
                under a load of 1 normalized unit of force.

                If the optional parameter stiffnessAtOneNormForce has beens set,
                its value is returned.
        
        If the optional parameter 'stiffnessAtOneNormForce' has not been set, 
        then this function will calculate the stiffness of the curve so that 
        it matches klin in Eqn. 5 in Thelen 2003. Note that the solution for 
        klin in Thelen 2003 is approximate, and only correct to 3 decimal
        places. This approximation causes noticable error particularly for
        tendons that experience high loads (such as the Achillies tendon). 
        The correct expressions, and the ones that are employed by this function,
        for \f$ k_{lin}\f$ and \f$\epsilon_{toe}\f$ are derived below.
        
        The remainder of this documentation will use the 
        notation that is found in Thelen 2003, where:

        \li \f$ \epsilon_0 \f$  : strainAtOneNormForce
        \li \f$ F_{toe} \f$: 33/100
        \li \f$ k_{toe} \f$: 3 (as in the paper)
               
        The exact solution for \f$ k_{lin}\f$ and 
        \f$\epsilon_{toe}\f$ can be solved by introducing a continuity equation.
        The values for etoe and klin must satisfy a C1 continuity condition, 
        that is the slope of the linear extrapolation must match the slope of 
        the exponental curve at the point where they join. First noting that 
        the slope of the linear extrapolation is given by

         \f[
        k_{lin} = \frac{1-F_{toe}}{\epsilon_0 - \epsilon_{toe}}
        \f]

        the continuity equation can be expressed by taking the derivative of
        the exponential and linear curves and equating them at 
        \f$ \epsilon=\epsilon_{toe}\f$

        \f[
         \frac{F_{toe}(k_{toe} / \epsilon_{toe} )}{ e^{k_{toe}} - 1}
         ( e^{ k_{toe} \epsilon / \epsilon_{toe} }) 
         = \frac{1-F_{toe}}{\epsilon_0-\epsilon_{toe}}          
        \f]

        Evaluating the above equation at the junction point between the two
        curves (\f$ \epsilon = \epsilon_{toe}\f$) yields an equation that
        can be rearranged to express \f$ \epsilon_{toe} \f$ as a function of
        \f$ \epsilon_{0} \f$ 
        (with values of \f$F_{toe}=33.0/100.0, k_{toe}=3\f$):

        \f[
        \epsilon_{toe} = 99 \epsilon_{0} e^3 / (166 e^3 - 67)     
        \f]
        
        which in double precision is
        
        \f[
        \epsilon_{etoe} = 0.6086155378758797 \epsilon_{0}
        \f]
        
        Substituting in the symbolic expression for \f$ \epsilon_{toe}\f$ to
        the above equation for \f$ k_{lin} \f$ yields

        \f[
        k_{lin} = \frac{1-F_{toe}}
                    {\epsilon_0(1 - 99 e^3 / (166 e^3 - 67))}
        \f]

        Which for values of \f$F_{toe}=33.0/100.0, k_{toe}=3\f$ and
        some simplification is

        \f[        
        k_{lin} = \frac{67} { 100(\epsilon_0 - (99\epsilon_0 e^3)/(166e^3-67)) }
        \f]

        In double precision this evaluates to

        \f[
            k_{lin} = \frac{1.711871739526343}{\epsilon_0}
        \f]

        This is the slope of the tendon curve that is used if the paramter
        'stiffnessAtOneNormForce' is not specified. The value for \f$ k_{lin}\f$
        is recomputed using its analytical equation (rather than the double
        floating point value) to ensure accuracy of the solution.

        <B>Computational Costs</B>       
        \verbatim
            ~80 flops for a new strain value
            ~5  flops for subsequent calls
        \endverbatim

        <B> References </B>
        Thelen (2003). Adjustment of Muscle
        Mechanics Model Paramters to Simulate Dynamic Contractions in Older
        Adults. ASME J Biomech Eng (125).
     */
     double getStiffnessAtOneNormForceInUse();

     /**
     @returns   A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure in the class description.

                This is an optional parameter, and so, if this parameter is 
                empty, a call to this function will throw an exception. If
                you're not sure if its been set, you can make a call to the
                corresponding set function before hand, or call the 
                corresponding '...InUse()' function.
     */
     double getCurviness();

     /**
        @returns A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure in the class description.
        
        If the optional parameter 'curviness' has been set, its value is 
        returned.

        If the optional parameter 'curviness' has not been set, 
        then this function will calculate the curviness parameter so that the
        curve has the same area (equivalent to the same normalized strain 
        energy) between a strain of 0, and the strain where
        the tendon develops 1 normalized unit of tension (e0) as a linearly
        extrapolated exponential curve documented in Thelen 2003 
        (with FToe = 0.33, and kToe = 3, as specified in the paper). 
        
        A root finding method (Bisection+Newton's method) is used for this task, 
        and continues until the normalized area of the reference tendon and the
        fit tendon agree with a relative error of 1e-6. If it is not possible
        to fit the tendon to this precision then an exception is thrown.
        
        
        <B> Computational Costs </B>
        \verbatim
            ~0 flops if parameter is already computed
            ~870,000 flops if curve fitting is required
        \endverbatim

     */
     double getCurvinessInUse();

     /**
    @param aStrainAtOneNormForce     
                The tendon strain at which the tendon develops 1 unit of 
                normalized force. The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtOneNormForce = 0.04 means that 
                the tendon will develop a tension of 1 normalized force when 
                it is strained by 4% of its resting length, or 
                equivalently is stretched to 1.04 times its resting length.
    */
     void setStrainAtOneNormForce(double aStrainAtOneNormForce);

     /**
     @param aStiffnessAtOneNormForce
                The normalized stiffness (or slope) of the tendon curve 
                when the tendon is strained by strainAtOneNormForce
                under a load of 1 normalized unit of force.
     */
     void setStiffnessAtOneNormForce(double aStiffnessAtOneNormForce);

     /**
     @param aCurviness  
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.
     */
     void setCurviness(double aCurviness);

    /**
    Calculates the value of the curve evaluated at the desired normalized fiber
    length. Note that if the curve is out of date it is rebuilt (at a cost of 
    ~20,500 flops). 

    @param aNormLength 
                The normalized fiber length used to evaluate the tendon force 
                length curve for the corresponding normalized force. Here 
                aNormLength = l/l0, where l is the length of the fiber and l0 
                is the resting length of the fiber.  Thus normalized length of 
                1.0 means the fiber is at its resting length.
        
    @return the value of the normalized force generated by the tendon

    <B>Computational Costs</B>
    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim

    */
    double calcValue(double aNormLength) const;

    /**
    Calculates the derivative of the tendon force length curve with respect to
    the normalized fiber length. Note that if the curve is out of date it is 
    rebuilt (at a cost of ~20,500 flops).

    @param aNormLength
                The normalized fiber length used to evaluate the tendon force 
                length curve for the corresponding normalized force. Here 
                aNormLength = l/l0, where l is the length of the fiber and l0 
                is the resting length of the fiber.  Thus normalized length of 
                1.0 means the fiber is at its resting length.

    @param order the order of the derivative. Only values of 0,1 and 2 are 
                  acceptable.

    @return the derivative of the normalized tendon force length curve w.r.t. 
            normalized tendon length
    
    <B>Computational Costs</B>       
    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops       
    \endverbatim

    */
    double calcDerivative(double aNormLength, int order) const;

    /**     
    @param aNormLength
                The normalized fiber length used to evaluate the tendon force 
                length curve for the corresponding normalized force. Here 
                aNormLength = l/l0, where l is the length of the fiber and l0 
                is the resting length of the fiber.  Thus normalized length of 
                1.0 means the fiber is at its resting length.
    
    @return Computes the normalized area under the curve. For this curve, 
            this quantity corresponds to the normalized potential energy stored 
            in the tendon - simply multiply this quantity by the number of 
            NormForce*NormLength (where NormForce corresponds to the number of
            Newtons that 1 normalized force corresponds to, and NormLength 
            is the length in meters that a length of 1 corresponds to) to obtain 
            the potental energy stored in the tendon in units of Joules.

    <B>Computational Costs</B>       
    \verbatim
        x in curve domain  : ~13 flops
        x in linear section: ~19 flops
    \endverbatim

    */
    double calcIntegral(double aNormLength) const;


    /**
       This function returns a SimTK::Vec2 that contains in its 0th element
       the lowest value of the curve domain, and in its 1st element the highest
       value in the curve domain of the curve. Outside of this domain the curve
       is approximated using linear extrapolation. Note that  if the curve is 
       out of date is rebuilt (which will cost ~20,500 flops).

       @return The minimum and maximum value of the domain, x, of the curve 
                  y(x). Within this range y(x) is a curve, outside of this range
                  the function y(x) is a C2 (continuous to the second 
                  derivative) linear extrapolation*/

    SimTK::Vec2 getCurveDomain() const;

    /**This function will generate a csv file with a name that matches the 
       curve name (e.g. "bicepfemoris_TendonForceLengthCurve.csv");
       Note that  if the curve is out of date is rebuilt 
       (which will cost ~20,500 flops).
       
       @param path The full path to the location. Note '/' slashes must be used,
            and do not put a '/' after the last folder.

       The file will contain the following columns:
       
       \verbatim
       Col# 1, 2,     3,       4,  
            x, y, dy/dx, d2y/dx2,
       \endverbatim
       
       The curve will be sampled from its linear extrapolation region
       (the region with normalized fiber velocities < -1), through 
       the curve, out to the other linear extrapolation region
       (the region with normalized fiber velocities > 1). The width of 
       each linear extrapolation region is 10% of the entire range of x, or 
       0.1*(x1-x0).

       The curve is sampled quite densely: there are 200+20 rows    

       <B>Computational Costs</B>
       \verbatim
            ~194,800 flops
       \endverbatim

       <B>Example</B>
       To read the csv file with a header in from Matlab, you need to use 
       csvread set so that it will ignore the header row. This is accomplished
       by using the extra two numerical arguments for csvread to tell the 
       function to begin reading from the 1st row, and the 0th index (csvread
       is 0 indexed). This is necessary to skip reading in the text header
       \verbatim
        data=csvread('bicepfemoris_fiberTendonForceLengthCurve.csv',1,0);
       \endverbatim

       */
       void printMuscleCurveToCSVFile(const std::string& path) const;

//==============================================================================
// PRIVATE
//==============================================================================
private:
    /*
    This object extends the ModelComponent interface so that we can make use
    of the 'createSystem' function, which we are using to create the 
    curve (using MuscleCurveFunctionFactory), which is an 
    expensive operation, just once prior to simulation. 
    
    Thus the user is allowed to set the properties of this curve until just 
    before the simulation begins. Just prior to the simulation starts 
    'createSystem' is called, and then this object will build the 
    MuscleCurveFunction that defines the curve the user requested
    */

    ///ModelComponent Interface required function
  	void setup(Model& model) OVERRIDE_11;
    ///ModelComponent Interface required function
	void initState(SimTK::State& s) const OVERRIDE_11;
    /**
    ModelComponent is being used for this one function, which is called just
    prior to a simulation beginning. This is the ideal time to actually
    create the curve because

    \li The curve parameters cannot change anymore
    \li This function is only called just prior to simulation, so the expensive
        task of creating the curve will only be done when it is absolutely 
        necessary

    */
	void createSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    ///ModelComponent Interface required function
    void setDefaultsFromState(const SimTK::State& state) OVERRIDE_11 {};
       

    /**
        @param strainAtOneNormForce 
            The Cauchy or engineering strain of the tendon under 1 unit force
            of tension. Cauchy or engineering strain is defined as 
            strainAtOneNormForce = (l-l0)/l0, where l is the length of the 
            tendon under tension, and l0 is the resting length.

        @returns    A vector that contains the parameters that define
                    the properties of the extrapolated exponental curve that is 
                    used to construct the reference tendon defined in 
                    Thelen 2003. The fields of the vector are as follows:
                    [0]: e0
                    [1]: etoe
                    [2]: Ftoe
                    [3]: ktoe
                    [4]: klin
                    [5]: normalized potental energy from a strain of 0 to e0
                
    <B>Computational Costs</B>       
    \verbatim
        ~211 flops     
    \endverbatim

    <B> References </B>
        Thelen (2003). Adjustment of Muscle
        Mechanics Model Paramters to Simulate Dynamic Contractions in Older
        Adults. ASME J Biomech Eng (125).
    */
    SimTK::Vector 
        calcReferenceTendonCurveProperties(double strainAtOneNormForce);

    void setNull();
    void constructProperties();

  /**
        This function will take all of the current property values, and if they
        have changed since the last time the curve was built, and build a curve.

        <B>Computational Costs</B>
        \verbatim 
            Curve Construction Costs :   ~20,500 flops
        \endverbatim

    */
  void buildCurve();

  /**
    @returns m_TendonReference if calcReferenceTendonCurveProperties has been
                               called, else it calls 
                               calcReferenceTendonCurveProperties
                               and assigns the resulting vector to 
                               m_TendonReference
  */
  SimTK::Vector getReferenceTendon();

  double fitToReferenceTendon(  double strainAtOneNormForce,
                                double stiffnessAtOneNormForce,
                                double areaAtOneNormForce,
                                double relTol,
                                std::string& name) const;

  MuscleCurveFunction   m_curve;

  //[0] the curviness in use
  //[1] the strain for which the curviness was computed
  SimTK::Vec2   m_stiffnessAtOneNormForceInUse; 
  //[0] the curviness in use
  //[1] the strain for which the curviness was computed
  SimTK::Vec2   m_curvinessInUse; 

  SimTK::Vector   m_TendonReference;
};

}

#endif // OPENSIM_TENDON_FORCE_LENGTH_CURVE_H_
