#ifndef OPENSIM_TENDON_FORCE_LENGTH_CURVE_H_
#define OPENSIM_TENDON_FORCE_LENGTH_CURVE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  TendonForceLengthCurve.h                     *
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
#include <OpenSim/Common/Function.h>
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
    fitted to match the average dimensionless in-vivo tendon curve 
    reported by Maganarius et al and Magnusson et al. In addition,
    the curve produced will have a characteristic toe region that appears
    in the in-vitro tendon testing literature (for example see Lewis et al, 
    figure 4), though does not show up strongly in the in-vivo literature. The
    curve is fitted using only the strainAtOneNormForce and this hueristic
    algorithm:
          
     \li stiffnessAtOneNormForce = 1.375/strainAtOneNormForce
     \li normForceAtToeEnd = 2.0/3.0  
     \li curviness = 0.5
     
     For strains less than 0, or greater than the toe strain (which is computed
     using normForceAtToeEnd and stiffnessAtOneNormForce) the C2 curve
     is linearly extrapolated.

     Note that this curve is not being fitted to the commonly used linearly
     extrapolated exponential curve documented by Thelen, as it makes the 
     toe region about half as stiff as both the in-vitro and in-vivo data
     indicate is reasonable, and additionally, makes the linear section of
     the curve nearly twice as stiff than the data indicates is reasonable.



 For more details on the fitting process see functions:

 \li TendonForceLengthCurve(double strainAtOneNormForce,
                            const std::string& muscleName)
 \li getStiffnessAtOneNormForceInUse()
 \li getNormForceAtToeEndInUse()
 \li getCurvinessInUse()

  <B> Usage </B>
    This object should be updated through the set methods provided. 
    These set methods will take care of rebuilding the object correctly. If you
    modify the properties directly, the object will not be rebuilt, and upon
    calling any functions an exception will be thrown because the object is out 
    of date with its properties.

 <B> References </B>
        \verbatim
        Lewis, G. and Shaw, K.M. (1997). Tensile Properties of Human Tendon
        Achillis: Effect of Donor Age and Strain Rate. The Journal of Foot 
        and Ankle Surgery, 36, 435-445.

        Maganaris, C.N. and Paul, J.P.(2002). Tensile properties of the in
        vivo grastrocnemius tendon. J. Biomechanics 35:1639-1646

        Magnusson S.P., Aagaard, P., Rosager, S., Dyhre-Poulsen, P., 
        and Kjaer,M. (2001). Load-displacement properties of the human triceps
        surae aponeurosis in vivo. Journal of Physiology, 531, 277-288.

        Thelen, DG (2003). Adjustment of Muscle Mechanics Model 
         Parameters to Simulate Dynamic Contractions in Older Adults. 
         ASME J.Biomech. Eng., 125, 75-77.
        \endverbatim

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
class OSIMACTUATORS_API TendonForceLengthCurve : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(TendonForceLengthCurve, Function);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(strain_at_one_norm_force, double,
        "tendon strain at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(norm_force_at_toe_end, double, 
        "normalized force developed at the end of the toe region");
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

        @param normForceAtToeEnd
                        The normalized force developed at the end of the `toe'
                        region. The toe region lies between 0 strain and 
                        some intermediate strain less than the strain required
                        to develop 1 norm force. The toe region is non-linear 
                        and more compliant than the rest of the tendon curve.

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
     is only C0 continuous. Details to appear in Millard et al 2013.

      <B>Conditions:</B>
        \verbatim
            strainAtOneNormForce > 0
            stiffnessAtOneNormForce > 1/strainAtOneNormForce
            0 < normForceAtToeEnd < 1
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
                            double normForceAtToeEnd,
                            double curviness,
                            const std::string& muscleName);

    /**
     Constructs a C2 continuous tendon force length curve that is fitted to 
     match the shape of tendon-load displacement curves reported in in-vito 
     load-displacement experiments, with the strains reported in in-vivo data.
             
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

     This curve has the 
     advantage of being C2 continuous which results in faster simulations when
     compared to the popular method of using a linearly extrapolated 
     exponential curve to parameterize the tendon force length curve, which 
     is only C0 continuous. Details to appear in Millard et al 2013. 
     
     This constructor will create a C2 continuous tendon force length curve that
     is fitted to match the average dimensionless in-vivo tendon curve 
     reported by Maganarius et al and Magnusson et al. In addition,
     the curve produced will have a characteristic toe region fitted to the
     in the in-vivo literature. The curve is fitted using only the 
     strainAtOneNormForce and this hueristic algorithm:
          
     \li stiffnessAtOneNormForce = 1.375/strainAtOneNormForce
     \li normForceAtToeEnd =2.0/3.0 (fitted to data from Maganaris et al., 
                                     and Magnusson et al.)
     \li curviness = 0.5
     
     For strains less than 0, or greater than strainAtOneNormForce the C2 curve
     is linearly extrapolated.

     Note that this curve is not being fitted to the commonly used linearly
     extrapolated exponential curve documented by Thelen, as this curve makes 
     the toe region about half as stiff as both the in-vitro and in-vivo data
     indicate is reasonable, and additionally, makes the linear section of
     the curve nearly twice as stiff as the data indicates is reasonable.

      <B>Conditions:</B>
        \verbatim
            strainAtOneNormForce > 0
        \endverbatim

        \verbatim
            ~174,100 flops
        \endverbatim

        <B>Default Parameter Values</B>
         \verbatim
             strainAtOneNormForce    = 0.049            
         \endverbatim

        <B>Example:</B>
        @code
            TendonForceLengthCurve fseCurve3(0.05,"soleus");
            double fseVal  = fseCurve3.calcValue(1.02);
            double dfselVal = fseCurve3.calcDerivative(1.02,1);
        @endcode

        <B>References:</B>
        \verbatim
        Maganaris, C.N. and Paul, J.P.(2002). Tensile properties of the in
        vivo grastrocnemius tendon. J. Biomechanics 35:1639-1646

        Magnusson S.P., Aagaard, P., Rosager, S., Dyhre-Poulsen, P., 
        and Kjaer,M. (2001). Load-displacement properties of the human triceps
        surae aponeurosis in vivo. Journal of Physiology, 531, 277-288.

        Lewis, G. and Shaw, K.M. (1997). Tensile Properties of Human Tendon
        Achillis: Effect of Donor Age and Strain Rate. The Journal of Foot 
        and Ankle Surgery, 36, 435-445.

        Thelen, DG (2003). Adjustment of Muscle Mechanics Model 
         Parameters to Simulate Dynamic Contractions in Older Adults. 
         ASME J.Biomech. Eng., 125, 75-77.
        \endverbatim  */
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
                its resting length.    */
     double getStrainAtOneNormForce() const;
  
     
     /**
        @returns The normalized stiffness (or slope) of the tendon curve 
                 when the tendon is strained by strainAtOneNormForce
                 under a load of 1 normalized unit of force.

        If the optional parameter stiffnessAtOneNormForce has beens set,
        its value is returned.

        If the optional parameter `stiffnessAtOneNormForce' has not been set 
        then it is assigned a value of 1.375/strainAtOneNormForce. This 
        hueristic value appears to agree well with the in-vivo data of
        Magnaris and Paul, Magnusson et al, and the in-vitro data produced
        by Lewis's for tendon force length curve of the Achilles tendon.

        <B> References </B>
        \verbatim
        Maganaris, C.N. and Paul, J.P.(2002). Tensile properties of the in
        vivo grastrocnemius tendon. J. Biomechanics 35:1639-1646

        Magnusson S.P., Aagaard, P., Rosager, S., Dyhre-Poulsen, P., 
        and Kjaer,M. (2001). Load-displacement properties of the human triceps
        surae aponeurosis in vivo. Journal of Physiology, 531, 277-288.

        Lewis, G. and Shaw, K.M. (1997). Tensile Properties of Human Tendon
        Achillis: Effect of Donor Age and Strain Rate. The Journal of Foot 
        and Ankle Surgery, 36, 435-445.
        \endverbatim
     */
     double getStiffnessAtOneNormForceInUse() const;

     /**
     @returns the norm force developed at the point in the curve where the toe
              region transitions to the linear stiffness region. By default
              a value of 2.0/3.0 is used as it best fits the 
              tendon-force--length curves reported by Magnaris et al. and
              Magnussen et al.

     <B>References</B>
     \verbatim
        Maganaris, C.N. and Paul, J.P.(2002). Tensile properties of the in
        vivo grastrocnemius tendon. J. Biomechanics 35:1639-1646

        Magnusson S.P., Aagaard, P., Rosager, S., Dyhre-Poulsen, P., 
        and Kjaer,M. (2001). Load-displacement properties of the human triceps
        surae aponeurosis in vivo. Journal of Physiology, 531, 277-288.

     \endverbatim
     */
     double getNormForceAtToeEndInUse() const;

     /**
        @returns A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure in the class description.
        
        If the optional parameter 'curviness' has been set, its value is 
        returned.
     
        If the optional parameter `curviness' has not been set 
        then it is assigned a value of 0.5. This produces a toe region 
        that appears to agree well with the in-vitro curves for tendon force
        length of the Achilles tendon as reported by Lewis.

        <B>References</B>
        \verbatim
        Lewis, G. and Shaw, K.M. (1997). Tensile Properties of Human Tendon
        Achillis: Effect of Donor Age and Strain Rate. The Journal of Foot 
        and Ankle Surgery, 36, 435-445.
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
                The tendon strain at which the tendon develops 1 unit of 
                normalized force. The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtOneNormForce = 0.04 means that 
                the tendon will develop a tension of 1 normalized force when 
                it is strained by 4% of its resting length, or 
                equivalently is stretched to 1.04 times its resting length.

    <B>Computational Cost</B>
    The curve is rebuilt at a cost of ~174,100 flops
    */
     void setStrainAtOneNormForce(double aStrainAtOneNormForce);
    
     /**
     @param aStiffnessAtOneNormForce
                The normalized stiffness (or slope) of the tendon curve 
                when the tendon is strained by strainAtOneNormForce
                under a load of 1 normalized unit of force.
    
     @param aNormForceAtToeEnd
                The normalized force developed at the end of the toe region,
                after which the force length curve becomes linear

     @param aCurviness  
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.

        <B>Computational Cost</B>
        The curve is rebuilt at a cost of ~174,100 flops


     */
     void setOptionalProperties( double aStiffnessAtOneNormForce, 
                                 double aNormForceAtToeEnd,
                                 double aCurviness);

    /**
    Calculates the value of the curve evaluated at the desired normalized fiber
    length. 

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
    the normalized tendon length.

    @param aNormLength
                The normalized tendon length used to evaluate the tendon force 
                length curve for the corresponding normalized force. Here 
                aNormLength = l/l0, where l is the length of the tendon and l0 
                is the tendon slack length. Thus, a normalized length of 1.0
                means the tendon is at its resting length.

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
       is approximated using linear extrapolation. 

       @return The minimum and maximum value of the domain, x, of the curve 
                  y(x). Within this range y(x) is a curve, outside of this range
                  the function y(x) is a C2 (continuous to the second 
                  derivative) linear extrapolation*/

    SimTK::Vec2 getCurveDomain() const;

    /**This function will generate a csv file with a name that matches the 
       curve name (e.g. "bicepfemoris_TendonForceLengthCurve.csv"). This 
       function is not const to permit the curve to be rebuilt if it is out of
       date with its properties.
      
       
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
       void printMuscleCurveToCSVFile(const std::string& path);

       void ensureCurveUpToDate();
//==============================================================================
// PRIVATE
//==============================================================================
private:
    /**
	//--------------------------------------------------------------------------
	<B> OpenSim::Function Interface </B>
	//--------------------------------------------------------------------------
    Create the underlying SimTK::Function that implements the calculations
	    necessary for this curve. 
	*/
    SimTK::Function* createSimTKFunction() const OVERRIDE_11;

	/**
        <B> No Longer Used</B>

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
                    [5]: normalized potental energy from a strain of 0 to etoe
                    [6]: normalized potental energy from a strain of 0 to e0
                
    <B>Computational Costs</B>       
    \verbatim
        ~211 flops     
    \endverbatim  

    <B> References </B>
        \verbatim
        Thelen (2003). Adjustment of Muscle
        Mechanics Model Parameters to Simulate Dynamic Contractions in Older
        Adults. ASME J Biomech Eng (125).
        \endverbatim  */
    SimTK::Vector 
        calcReferenceTendon(double strainAtOneNormForce);

    void setNull();
    void constructProperties();

  /**
        This function will take all of the current property values, and if they
        have changed since the last time the curve was built, and build a curve.

        <B>Computational Costs</B>
        \verbatim 
            Curve Construction Costs :   ~20,500 flops
        \endverbatim    */
  void buildCurve(bool computeIntegral = false);


  SmoothSegmentedFunction   m_curve;

  double   m_normForceAtToeEndInUse;
  double   m_stiffnessAtOneNormForceInUse;  
  double   m_curvinessInUse; 
  bool     m_isFittedCurveBeingUsed;

};

}

#endif // OPENSIM_TENDON_FORCE_LENGTH_CURVE_H_
