#ifndef OPENSIM_FIBER_FORCE_LENGTH_CURVE_H_
#define OPENSIM_FIBER_FORCE_LENGTH_CURVE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  FiberForceLengthCurve.h                      *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>
#include <OpenSim/Common/SmoothSegmentedFunction.h>
#include <simbody/internal/common.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {
//==============================================================================
//                        FIBER FORCE LENGTH CURVE
//==============================================================================

/**
 This class serves as a serializable FiberForceLengthCurve, commonly used
 to model the parallel elastic element, for use in muscle models. This curve i
 is dimensionless, with force being normalized to maximum isometric force, and
 length normalized to resting muscle length. The user has control over the 
 maximum strain at no load, the strain developed under 1 unit force of 
 load, its stiffness at 1 unit load, and the shape of the 
 curve (its `curviness'). 

 All parameters but the strain of the fiber at 1 unit load are optional. 
 Note that the optional parameters must either be left blank, or all filled in. 
 Filling in one optional parameter but the others will throw an exception
 when the curve is built.

 <B>Manditory Properties</B>

 \li strainAtOneNormForce 

 <B>Optional Properties</B>
 \li strainAtZeroForce
 \li stiffnessAtOneNormForce
 \li curviness

 The shape of the curve can be varied from a close approximation of a line to a
 sharply bent curve using the optional parameters. 
 
  \image html fig_FiberForceLengthCurve.png

 If the optional parameters are not specified, the curve is fit using a 
 hueristic algorithm that appears to agree well with experimentally measured
 fiber force length curves of Winters et al, and additionally, fits Thelen's
 fiber force length curve (matching its area, strain, and stiffness at one 
 norm force).

 \li strainAtZeroForce  is set to 0
 \li strainAtOneNormForce  is set to 0.6
 \li stiffnessAtOneNormForce = 5.03391827453152134
                                /(strainAtOneNormForce-strainAtZeroForce)
 \li curviness = 0.63753341725162227

 The `magic numbers' are a result of fitting the curve to Thelen's 2003 default
 fiber force length curve (which agrees well with Winters et al's experimental
 data) so that the two curves have the same final stiffness, strain, and area.
 The advantage of this curve is that it is continuous to the second derivative 
 (the Thelen 2003 curve is only C0 continuous). The improved smoothness of the 
 curve makes the equations far easier to simulate and saves simulation time, and 
 permit a number of different solving techniques to be used that require 
 continuity. In addition, the extra parameters in this curve formulation can be 
 made to match a wide variety of shapes, should it be desired to fit the curve 
 to experimental data. 
 
 <B> Default Values </B>

 The default value for strainAtOneNormForce is 0.6, which matches the 
 experimental curve reported in Winters et al in Fig. 3a, and Thelen's fiber
 force length curve function described in his 2003 paper.

 <B> References </B>

    Thelen (2003). Adjustment of Muscle Mechanics Model Paramters to 
    Simulate Dynamic Contractions in Older Adults. ASME J Biomech Eng (125).

    Winters, T.M., Takahashi,M., Lieber,R.L., and Ward,S.(2010). Whole Muscle
    Length-Tension Relationships are Accurately Modeled as Scaled Sarcomeres in
    Rabbit Hindlimb Muscles. J.Biomech 44:109-115.

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
class OSIMACTUATORS_API FiberForceLengthCurve : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(FiberForceLengthCurve, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(strain_at_one_norm_force, double, 
        "Fiber strain at a tension of 1 normalized force");
    OpenSim_DECLARE_PROPERTY(strain_at_zero_force, double, 
        "Fiber strain at zero force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_one_norm_force, double, 
        "Fiber stiffness at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(curviness, double, 
        "Fiber curve bend, from linear to maximum bend (0-1)");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
      /**
        Creates a default fitted fiber force length curve using only the strain 
        the fiber undergoes at 1 normalized unit force of tensile load. The 
        curve is given a the name `default_FiberForceLengthCurve'.

     The manditory parameters required by this curve are set to their 
     default values:

     \li strainAtZeroForce  is set to 0
     \li strainAtOneNormForce  is set to 0.6

     The remaining parameters required by this curve are calculated
     using a hueristic algorithm that appears to agree well with experimentally
     measured fiber force length curves of Winters et al, and additionally, fits
     Thelen's fiber force length curve (matching its area, strain, and stiffness
     at one norm force).
     
     \li stiffnessAtOneNormForce = 5.03391827453152134
                                    /(strainAtOneNormForce-strainAtZeroForce)
     \li curviness = 0.63753341725162227

     The `magic numbers' are a result of fitting the curve to Thelen's 2003 
     default fiber force length curve (which agrees well with Winters et al's 
     experimental data) so that the two curves have the same final stiffness, 
     strain, and area. 
 
     <B> Default Values </B>

     The default value for strainAtOneNormForce is 0.6, which matches the 
     experimental curve reported in Winters et al in Fig. 3a, and Thelen's fiber
     force length curve function described in his 2003 paper.

     <B> References </B>
     \verbatim
        Thelen (2003). Adjustment of Muscle Mechanics Model Paramters to 
        Simulate Dynamic Contractions in Older Adults. ASME J Biomech Eng (125).

        Winters, T.M., Takahashi,M., Lieber,R.L., and Ward,S.(2010). Whole Muscle
        Length-Tension Relationships are Accurately Modeled as Scaled Sarcomeres in
        Rabbit Hindlimb Muscles. J.Biomech 44:109-115.
    \endverbatim
    */
    FiberForceLengthCurve();

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /**
     Constructs a C2 continuous fiber force length curve. This curve has the 
     advantage of being C2 continuous which results in slightly faster 
     simulations and is required by a number of useful numerical solving 
     routines.
     
        @param strainAtZeroForce
                    The fiber strain at which the fiber starts to develop 
                    force. The definition of strain used for this 
                    quantity is consistent with the Cauchy or engineering 
                    definition of strain: strain = (l-l0)/l0, where l is length,
                    and l0 is resting length. In this context 
                    strainAtZeroForce = 0.0 means that the fiber will start to
                    develop tension when it is at its resting length.

        @param strainAtOneNormForce
                    The fiber strain at which the fiber develops 1 unit of 
                    normalized force. The definition of strain used for this 
                    quantity is consistent with the Cauchy or engineering 
                    definition of strain: strain = (l-l0)/l0, where l is length,
                    and l0 is resting length. In this context 
                    strainAtOneNormForce = 0.6 means that the fiber will 
                    develop a tension of 1 normalized force when it is strained 
                    by 60% of its resting length, or equivalently is stretched 
                    to 1.60 times its resting length.

        @param stiffnessAtOneNormForce     
                The normalized stiffness (or slope) of the fiber curve when the 
                fiber is strained by strainAtOneNormForce under a load of 1 
                normalized unit of force.

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
                appending "_FiberForceLengthCurve" to the string in muscleName.
                This name is used for making intelligible error messages and 
                also for naming the XML version of this curve when it is 
                serialized.

      <B>Conditions:</B>
        \verbatim
            strainAtOneNormForce > strainAtZeroForce
            stiffnessAtOneNormForce > 1/(strainAtOneNormForce-strainAtZeroForce)
            0 <= curviness <= 1
        \endverbatim

        <B>Computational Costs</B>
        \verbatim 
            ~174,100 flops
        \endverbatim

    <B> Default Parameter Values </B>
         \verbatim
             strainAtOneNormForce    = 0.60,              
         \endverbatim


        <B>Example:</B>
        @code
            FiberForceLengthCurve fpeCurve3(0,0.60,8.4,0.65,"soleus");
            double fpeVal  = fpeCurve3.calcValue(0.02);
            double dfpeVal = fpeCurve3.calcDerivative(0.02,1);
        @endcode
    */
    FiberForceLengthCurve(  double strainAtZeroForce,
                            double strainAtOneNormForce, 
                            double stiffnessAtOneNormForce,
                            double curviness,
                            const std::string& muscleName);



    
    /**
    @returns    The fiber strain at which the fiber just begins to develop
                force. The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtZeroForce = 0.0 means that the fiber will start to
                tension of when it is at its resting length.

                By default this property is set to 0.
    */
     double getStrainAtZeroForce() const;


    /**
    @returns    The fiber strain at which the fiber develops 1 unit of 
                normalized force. The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtOneNormForce = 0.6 means that 
                the fiber will develop a tension of 1 normalized force when 
                it is strained by 60% of its resting length, or 
                equivalently is stretched to 1.6 times its resting length.

                By default this property is set to 0.6.
    */
     double getStrainAtOneNormForce() const;

     

     /**
     @returns   The slope of the curve (or stiffness of the fiber) when the 
                fiber is under 1 normalized tensile force. 
                
                If this optional property has been set, the value of this 
                property is returned.
                
                If the property has not been set, the stiffness is computed 
                using the following hueristic algorithm which produces curves
                that agree well with Winters et al.'s in-vivo data, and 
                Thelen's exponential curve.

                 \li strainAtZeroForce  is set to 0
                 \li strainAtOneNormForce  is set to 0.6
                 \li stiffnessAtOneNormForce = 5.03391827453152134
                                /(strainAtOneNormForce-strainAtZeroForce)

               \verbatim 
                        ~5 flops 
                \endverbatim


        <B> References </B>
        \verbatim
        Thelen (2003). Adjustment of Muscle Mechanics Model Paramters to 
        Simulate Dynamic Contractions in Older Adults. ASME J Biomech Eng (125).

        Winters, T.M., Takahashi,M., Lieber,R.L., and Ward,S.(2010). 
        Whole Muscle Length-Tension Relationships are Accurately Modeled as 
        Scaled Sarcomeres in Rabbit Hindlimb Muscles. J.Biomech 44:109-115.
        \endverbatim                
     */
     double getStiffnessAtOneNormForceInUse() const;

     

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
        then this function will use a value that appears to match Winters et
        al.'s in vivo data, and Thelen's fiber force length curve:

        \li curviness = 0.63753341725162227
      
        
        
        <B> Computational Costs </B>

        \verbatim
            ~1-5 flops            
        \endverbatim

     */
     double getCurvinessInUse() const;

     /**
     @returns true if the optional properties are empty and the fitted curve is
              being used. This function returns false if the optional properties
              are filled and are being used to construct the curve.
     */
     bool isFittedCurveBeingUsed() const;

     /**
    @param aStrainAtOneNormForce     
                The fiber strain at which the fiber develops 1 unit of 
                normalized force. The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtOneNormForce = 0.6 means that 
                the fiber will develop a tension of 1 normalized force when 
                it is strained by 60% of its resting length, or 
                equivalently is stretched to 1.6 times its resting length.
    */
     void setStrainAtOneNormForce(double aStrainAtOneNormForce);

     /**
    @param aStrainAtZeroForce     
                The fiber strain at which the fiber develops tenson. 
                The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtOneNormForce = 0.0 means that 
                the fiber will start to develop tension at its resting length.
    */
     void setStrainAtZeroForce(double aStrainAtZeroForce);

    
     /**
     @param aStiffnessAtOneNormForce
                The normalized stiffness (or slope) of the fiber curve 
                when the fiber is strained by strainAtOneNormForce
                under a load of 1 normalized unit of force.

     @param aCurviness  
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.

    
     */    
     void setOptionalProperties(double aStiffnessAtOneNormForce,
                                double aCurviness);

    /**
    Calculates the value of the curve evaluated at the desired normalized fiber
    length. Note that if the curve is out of date it is rebuilt 
    (at a cost of ~20,500 flops). 

    @param aNormLength
                The normalized fiber length used to evaluate the fiber force 
                length curve for the corresponding normalized force. Here 
                aNormLength = l/l0, where l is the length of the fiber and l0 
                is the resting length of the fiber.  Thus normalized length of 
                1.0 means the fiber is at its resting length.      

    @return the value of the normalized force generated by the fiber

    <B>Computational Costs</B>

    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim

    */
    double calcValue(double aNormLength) const;

    /**
    Calculates the derivative of the fiber force length curve w.r.t. 
    to the normalized fiber length. Note that if the curve is out of date it is 
    rebuilt (at a cost of ~20,500 flops).

    @param aNormLength
                The normalized fiber length used to evaluate the fiber force 
                length curve for the corresponding normalized force. Here 
                aNormLength = l/l0, where l is the length of the fiber and l0 
                is the resting length of the fiber.  Thus normalized length of 
                1.0 means the fiber is at its resting length.

    @param order the order of the derivative. Only values of 0,1 and 2 are 
                  acceptable.

    @return the derivative of the normalized fiber force length curve w.r.t. 
        normalized fiber length

    <B>Computational Costs</B>    

    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops       
    \endverbatim

    */
    double calcDerivative(double aNormLength, int order) const;

    /**     
    @param aNormLength
                The normalized fiber length used to evaluate the fiber force 
                length curve. Here aNormLength = l/l0, where l is the length 
                of the fiber and l0 is the resting length of the fiber.  
                Thus normalized length of 1.0 means the fiber is at its 
                resting length.
    
    @return Computes the normalized area under the curve. For this curve, 
            this quantity corresponds to the normalized potential energy stored 
            in the fiber - simply multiply this quantity by the number of 
            NormForce*NormLength (where NormForce corresponds to the number of
            Newtons that 1 normalized force corresponds to, and NormLength 
            is the length in meters that a length of 1 corresponds to) to obtain 
            the potental energy stored in the fiber in units of Joules.

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
       curve name (e.g. "bicepfemoris_FiberForceLengthCurve.csv");
       Note that  if the curve is out of date it is rebuilt 
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
        data=csvread('bicepfemoris_fiberFiberForceLengthCurve.csv',1,0);
       \endverbatim

       */
       void printMuscleCurveToCSVFile(const std::string& path) const;

//==============================================================================
// PRIVATE
//==============================================================================
private:
    /*
    This object extends the ModelComponent interface so that we can make use
    of the 'addToSystem' function, which we are using to create the 
    curve (using SmoothSegmentedFunctionFactory), which is an 
    expensive operation, just once prior to simulation. 
    
    Thus the user is allowed to set the properties of this curve until just 
    before the simulation begins. Just prior to the simulation starts 
    'addToSystem' is called, and then this object will build the 
    SmoothSegmentedFunction that defines the curve the user requested
    */

    ///ModelComponent Interface required function
    void connectToModel(Model& model) OVERRIDE_11;
    ///ModelComponent Interface required function
    void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;
    /**
    ModelComponent is being used for this one function, which is called just
    prior to a simulation beginning. This is the ideal time to actually
    create the curve because

    \li The curve parameters cannot change anymore
    \li This function is only called just prior to simulation, so the expensive
        task of creating the curve will only be done when it is absolutely 
        necessary

    */
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    ///ModelComponent Interface required function
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11 {};
    
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
        If the curve is upto date nothing is done. If the curves parameters have
        changed then all of the 
    */
    void ensureCurveUpToDate();

    /*<B>NO LONGER USED</B>
     @returns The properties of the passive fiber length curve documented in 
            Thelen 2003. Specifically:

        \li [0]: the strain at which the fiber begins to develop force
        \li [1]: the strainAtOneNormForce these properties were computed for
        \li [2]: kPE
        \li [3]: Normalized slope: dF/depsilon at 
                 strainAtOneNormForce (stiffness)
        \li [4]: Normalized area under the curve from a strain (Cauchy 
                 definition of strain) from 0 to strainAtOneNormForce

        Note that the equation presented in Thelen 2003 has been improved
        upon to fit passive force length curves that begin to develop
        force at a strain value that is greater than 0, by replacing
        the e1 terms with (e1-e0) terms.
    
   <B> Computational Cost: </B>

   \verbatim 
            ~220 flops
    \endverbatim

   <B> References </B>
        \verbatim
        Thelen (2003). Adjustment of Muscle Mechanics Model Paramters to 
        Simulate Dynamic Contractions in Older Adults. ASME J Biomech Eng (125).
        \endverbatim
   */
   SimTK::Vec5 calcReferencePassiveFiber(double strainAtZeroForce,
                                         double strainAtOneNormForce);
      
   //<B>NO LONGER USED</B>
   double calcCurvinessOfBestFit(double e0, double e1,
                                 double k, 
                                 double area, double relTol);

    SmoothSegmentedFunction   m_curve;    
    double m_stiffnessAtOneNormForceInUse;    
    double m_curvinessInUse;
    bool   m_fittedCurveBeingUsed; 

};

}

#endif // OPENSIM_FIBER_FORCE_LENGTH_CURVE_H_
