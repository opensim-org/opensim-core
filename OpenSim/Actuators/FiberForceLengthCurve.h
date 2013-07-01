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
/** This class serves as a serializable FiberForceLengthCurve, commonly used
    to model the parallel elastic element in muscle models. This curve is
    dimensionless, with force being normalized to maximum isometric force and
    length being normalized to resting muscle length. The user can adjust the
    maximum strain at no load and the strain developed under 1 unit force of
    load using the fitted curve. Additionally, if desired, it is possible to
    directly set the low-force stiffness of the fiber, the stiffness of the
    fiber at 1 unit of normalized force, and the shape of the curve (its
    'curviness').

    All parameters but the strain of the fiber at 1 unit load are optional. The
    manditory parameter can be set using either the constructor or the provided
    'setCurveStrains' function. Note that the optional parameters must either be
    left blank or all filled in using the provided 'setOptionalProperties'
    function. Filling in one optional parameter but not the others will throw
    an exception when the curve is built.

 <B>Manditory Properties</B>
    \li strainAtOneNormForce

 <B>Optional Properties</B>
 \li strainAtZeroForce
 \li stiffnessAtLowForce
 \li stiffnessAtOneNormForce
 \li curviness

    The shape of the curve can be varied from a close approximation of a line to
    a sharply bent curve using the optional parameters.

    \image html fig_FiberForceLengthCurve.png

    If the optional parameters are not specified, the curve is fit using a
    hueristic algorithm that appears to agree well with the experimentally
    measured fiber-force-length curves of Winters et al.

 \li strainAtZeroForce  is set to 0
 \li strainAtOneNormForce  is set to 0.6
    \li stiffnessAtOneNormForce = 3.0 / (strainAtOneNormForce-strainAtZeroForce)
                             = 3.0/0.6 = 5
    \li stiffnessAtLowForce = 0.025 * stiffnessAtOneNormForce
                         = 0.125
 \li curviness = 0.75

    The 'magic numbers' are a result of fitting the curve to the experimental
    data reported by Winters et al. The advantage of this curve over the typical
    exponential curve used in the literature is that it is continuous to the
    second derivative; the usual linearly-extrapolated exponential curve is only
    C0-continuous. The improved smoothness of this curve makes the equations
    somewhat easier to simulate and, more importantly, permits the use of
    derivative-based numerical methods on the curve. In addition, the extra
    parameters in this curve formulation can be adjusted to match a wide variety
    of shapes, should it be desired to fit the curve to experimental data.

    <B>Usage</B>
    Note that this object should be updated through the set methods provided.
 These set methods will take care of rebuilding the curve correctly. If you
 modify the properties directly, the curve will not be rebuilt, and upon
    calling a function like calcValue, calcDerivative, or printCurveToCSVFile,
    an exception will be thrown because the curve is out-of-date with its
 properties.

    <B>Example</B>
 @code
    // Make a fitted fiber-force-length curve.
     FiberForceLengthCurve fpeCurve2;
     fpeCurve2.setStrainAtOneNormForce(0.80);
     double fpeVal2 = fpeCurve2.calcValue(0.1);

    // Make a custom fiber-force-length curve by supplying every parameter.
    FiberForceLengthCurve fpeCurve3(-0.1, 0.50, 0.1, 5.0, 0.75, "testMuscle");
            double fpe3Val  = fpeCurve3.calcValue(0.02);
            double dfpe3Val = fpeCurve3.calcDerivative(0.02,1);
 @endcode

    <B>Default Values</B>
    The default value for strainAtOneNormForce is 0.6, which matches the
    experimental curve reported in Winters et al. (Fig. 3a) and the
    fiber-force-length curve described by Thelen (2003).

    <B>References</B>
    \li Thelen, D.G. (2003). Adjustment of muscle mechanics model paramters to
        simulate dynamic contractions in older adults. ASME Journal of
        Biomechanical Engineering 125:70-77.

    \li Winters, T.M., Takahashi, M., Lieber, R.L., and Ward, S. (2010). Whole
        muscle length-tension relationships are accurately modeled as scaled
        sarcomeres in rabbit hindlimb muscles. Journal of Biomechanics
        44:109-115.

 <B>Computational Cost Details</B>
    All computational costs assume the following operation costs:

    \verbatim
    Operation Type     : #flops
    *,+,-,=,Boolean Op : 1
                     / : 10
                  sqrt : 20
                  trig : 40
    \endverbatim

    Since these relative weightings will vary from processor to processor, all
    the quoted computational costs are approximate.

  @author Matt Millard
*/
class OSIMACTUATORS_API FiberForceLengthCurve : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(FiberForceLengthCurve, Function);
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
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_low_force, double,
        "Fiber stiffness at the end of the low-force region");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_one_norm_force, double,
        "Fiber stiffness at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(curviness, double,
        "Fiber curve bend, from linear bend (0) to maximum bend (1)");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Creates a default fitted fiber-force-length curve using only the strain
    the fiber undergoes at 1 normalized unit of tensile load. The curve is given
    the name 'default_FiberForceLengthCurve'. The parameter values used to
    create the default curve are specified in the class description.
    */
    FiberForceLengthCurve();

    /** Creates a fiber-force-length curve using the parameters specified by the
    user.

        @param strainAtZeroForce
        The fiber strain at which the fiber starts to develop force. The
        definition of strain used for this quantity is consistent with the
        Cauchy or engineering definition of strain: strain = (l-l0)/l0, where l
        is the current fiber length and l0 is its resting length. In this
        context, strainAtZeroForce = 0.0 means that the fiber will start to
                    develop tension when it is at its resting length.
        @param strainAtOneNormForce
        The fiber strain at which the fiber develops 1 unit of normalized force.
        The definition of strain used for this quantity is consistent with the
        Cauchy or engineering definition of strain: strain = (l-l0)/l0, where l
        is the current fiber length and l0 is its resting length. In this
        context, strainAtOneNormForce = 0.6 means that the fiber will develop a
        tension of 1 normalized force when it is strained by 60% of its resting
        length or, equivalently, when it is stretched to 1.60 times its resting
        length.
    @param stiffnessAtLowForce
        The normalized stiffness (or slope) of the curve at low force.
    @param stiffnessAtOneNormForce
        The normalized stiffness (or slope) of the curve when the fiber develops
        a tension of 1 normalized unit of force.
    @param curviness
        A dimensionless parameter between 0 and 1 that controls how the curve is
        drawn: a value of 0 will create a curve that is very close to a straight
        line segment while a value of 1 will create a curve that smoothly fills
        the corner formed by the linear extrapolation of
        'stiffnessAtOneNormForce' and the x-axis, as shown in the figure.
        @param muscleName
        The name of the muscle to which this curve belongs. The muscle name is
        used to create the name of this curve simply by appending
        "_FiberForceLengthCurve" to the string in muscleName. The curve name is
        used for reporting meaningful error messages and for naming the XML
        version of this curve when it is serialized.

    <B>Conditions</B>
        \verbatim
            strainAtOneNormForce >  strainAtZeroForce
            stiffnessAtOneNormForce > 1/(strainAtOneNormForce-strainAtZeroForce)
            0 < strainAtLowForce < stiffnessAtOneNormForce
            0 <= curviness <= 1
        \endverbatim

    <B>Computational Cost</B>
    \verbatim
            ~174,100 flops
        \endverbatim

    <B>Example</B>
        @code
        FiberForceLengthCurve fpeCurve3(0, 0.60, 0.16, 6.67, 0.5, "soleus");
            double fpeVal  = fpeCurve3.calcValue(0.02);
            double dfpeVal = fpeCurve3.calcDerivative(0.02,1);
        @endcode
    */
    FiberForceLengthCurve(double strainAtZeroForce,
                          double strainAtOneNormForce,
                            double stiffnessAtLowForce,
                          double stiffnessAtOneNormForce,
                            double curviness,
                            const std::string& muscleName);

    /** @returns The fiber strain at which the fiber just begins to develop
    force. The definition of strain used for this quantity is consistent with
    the Cauchy or engineering definition of strain: strain = (l-l0)/l0, where l
    is the current fiber length and l0 is its resting length. In this context,
    strainAtZeroForce = 0.0 means that the fiber will start to develop force
    when it is at its resting length. This property is set to 0 by default. */
     double getStrainAtZeroForce() const;

    /** @returns The fiber strain at which the fiber develops 1 unit of
    normalized force. The definition of strain used for this quantity is
    consistent with the Cauchy or engineering definition of strain: strain =
    (l-l0)/l0, where l is the current fiber length and l0 is its resting length.
    In this context, strainAtOneNormForce = 0.6 means that the fiber will
    develop a tension of 1 normalized force when it is strained by 60% of its
    resting length or, equivalently, is stretched to 1.6 times its resting
    length. This property is set to 0.6 by default. */
     double getStrainAtOneNormForce() const;

    /** @returns The slope of the curve when the fiber is just beginning to
    develop tensile force. If this optional property has been set, the value of
    the property is returned; otherwise, the stiffness is computed using a
    fitting algorithm that produces curves that closely match the in-vivo data
    reported by Winters et al.
        \verbatim
        ~5 flops
    \endverbatim
     */
     double getStiffnessAtLowForceInUse() const;

    /** @returns The slope of the curve when the fiber is developing 1 unit of
    normalized force. If this optional property has been set, the value of the
    property is returned; otherwise, the stiffness is computed using a hueristic
    that produces curves that closely match the in-vivo data reported by Winters
    et al.
        \verbatim
        ~5 flops
    \endverbatim
     */
     double getStiffnessAtOneNormForceInUse() const;

    /** @returns A dimensionless parameter between 0 and 1 that controls how the
    curve is drawn: a value of 0 will create a curve that is very close to a
    straight line segment, while a value of 1 will create a curve that smoothly
    fills the corner formed by the linear extrapolation of
    'stiffnessAtOneNormForce' and the x-axis, as shown in the figure in the
    class description. If the optional 'curviness' parameter has been set, its
    value is returned; otherwise, this function will use the default value
    mentioned in the class description.

    <B>Computational Cost</B>
        \verbatim
        ~1-5 flops
        \endverbatim
     */
     double getCurvinessInUse() const;

    /** @returns true if the optional properties are empty and the fitted curve
    is being used, false if the optional properties are filled and are being
    used to construct the curve. */
     bool isFittedCurveBeingUsed() const;

     /**
    @param aStrainAtZeroForce The fiber strain at which the fiber develops
        tenson. The definition of strain used for this quantity is consistent
        with the Cauchy or engineering definition of strain: strain = (l-l0)/l0,
        where l is the current fiber length and l0 is its resting length. In
        this context, strainAtOneNormForce = 0.0 means that the fiber will start
        to develop tension at its resting length.
    @param aStrainAtOneNormForce The fiber strain at which the fiber develops 1
        unit of normalized force. The definition of strain used for this
        quantity is consistent with the Cauchy or engineering definition of
        strain: strain = (l-l0)/l0, where l is the current fiber length and l0
        is its resting length. In this context, strainAtOneNormForce = 0.6 means
        that the fiber will develop a tension of 1 normalized force when it is
        strained by 60% of its resting length or, equivalently, is stretched to
        1.6 times its resting length.

    <B>Conditions</B>
        \verbatim
        strainAtOneNormForce > strainAtZeroForce
        \endverbatim

    <B>Computational Cost</B>
         The  curve is rebuilt at a cost of ~174,100 flops
     */
     void setCurveStrains(double aStrainAtZeroForce,
                         double aStrainAtOneNormForce);

     /**
    @param aStiffnessAtLowForce The normalized stiffness (or slope) of the curve
        when it is just beginning to develop force. The value used for
        aStiffnessAtLowForce is typically a small fraction of the value used in
        aStiffnessAtOneNormForce.
    @param aStiffnessAtOneNormForce The normalized stiffness (or slope) of the
        curve when the fiber is strained by strainAtOneNormForce under a load of
        1 normalized unit of force.
    @param aCurviness A dimensionless parameter between 0 and 1 that controls
        how the curve is drawn: a value of 0 will create a curve that is very
        close to a straight line segment; a value of 1 will create a curve that
        smoothly fills the corner formed by the linear extrapolation of
        'stiffnessAtOneNormForce' and the x-axis, as shown in the figure.

    <B>Conditions</B>
        \verbatim
        stiffnessAtOneNormForce > 1/(strainAtOneNormForce-strainAtZeroForce)
        \endverbatim

    <B>Computational Cost</B>
      The  curve is rebuilt at a cost of ~174,100 flops
    */
     void setOptionalProperties(double aStiffnessAtLowForce,
                               double aStiffnessAtOneNormForce,
                                double aCurviness);

    /** Calculates the value of the curve evaluated at the desired normalized
    fiber length.
    @param aNormLength The normalized fiber length used to evaluate the fiber
        force-length curve for the corresponding normalized force. Here,
        aNormLength = l/l0, where l is the current fiber length and l0 is its
        resting length. Thus, a normalized length of 1.0 means that the fiber is
        at its resting length.
    @return The value of the normalized force generated by the fiber.

    <B>Computational Cost</B>
    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim
    */
    double calcValue(double aNormLength) const;

    /** Calculates the derivative of the fiber force-length curve with respect
    to the normalized fiber length.
    @param aNormLength The normalized fiber length used to evaluate the fiber
        force-length curve for the corresponding normalized force. Here,
        aNormLength = l/l0, where l is the current fiber length and l0 is its
        resting length. Thus, a normalized length of 1.0 means that the fiber is
        at its resting length.
    @param order The order of the derivative. Only values of 0, 1, and 2 are
                  acceptable.
    @return The derivative of the normalized fiber force-length curve with
        respect to the normalized fiber length.

    <B>Computational Cost</B>
    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops
    \endverbatim
    */
    double calcDerivative(double aNormLength, int order) const;

    /**
    @param aNormLength The normalized fiber length used to evaluate the fiber
        force-length curve. Here, aNormLength = l/l0, where l is the current
        fiber length and l0 is its resting length. Thus, a normalized length of
        1.0 means that the fiber is at its resting length.
    @return Computes the normalized area under the curve. For this curve, this
        quantity corresponds to the normalized potential energy stored in the
        fiber. To calculate the potental energy stored in the fiber in units of
        Joules, simply multiply the returned quantity by normForce*normLength
        (where normForce is the number of Newtons represented by a normalized
        force of 1.0, and normLength is the number of meters represented by a
        normalized length of 1.0).

    <B>Computational Cost</B>
    \verbatim
        x in curve domain  : ~13 flops
        x in linear section: ~19 flops
    \endverbatim
    */
    double calcIntegral(double aNormLength) const;

    /** This function returns a SimTK::Vec2 containing the lower (0th element)
    and upper (1st element) bounds on the domain of the curve. Outside this
    domain, the curve is approximated using linear extrapolation.
    @return The minimum and maximum value of the domain, x, of the curve y(x).
        Within this range, y(x) is a curve; outside this range, the function
        y(x) is a C2-continuous linear extrapolation.
    */
    SimTK::Vec2 getCurveDomain() const;

    /** This function will generate a .csv file with a name that matches the
    curve name (e.g., "bicepfemoris_FiberForceLengthCurve.csv"). This function
    is not const to permit the curve to be rebuilt if it is out-of-date with its
    properties.
    @param path The full destination path. Note that forward slashes ('/') must
        be used, and there should not be a slash after the last folder.

       The file will contain the following columns:
       \verbatim
        column# 1, 2,     3,       4
                x, y, dy/dx, d2y/dx2
       \endverbatim

    The curve will be sampled from its zero-force region, through the curve, out
    to strains beyond which the fiber generates passive normalized forces
    greater than 1.

    The curve is sampled quite densely: the fiber force-length .csv file will
    have 200+20 rows.

    <B>Computational Cost</B>
       \verbatim
            ~194,800 flops
       \endverbatim

       <B>Example</B>
    To read the .csv file into Matlab, you need to set csvread to ignore the
    header row. Since csvread is 0-indexed, the following example will begin
    reading the .csv file from the first column of the second row:
       \verbatim
        data = csvread('bicepfemoris_FiberForceLengthCurve.csv', 1, 0);
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

    void setNull();
    void constructProperties();

    /** This function will take all of the current property values. If they have
    changed since the last time the curve was built, the curve is rebuilt.

    <B>Computational Cost</B>
    \verbatim
        Curve construction costs ~20,500 flops
        \endverbatim
    */
    void buildCurve(bool computeIntegral=false);

    /** @returns The properties of the passive force-length curve documented in
    Thelen (2003). Specifically:
        \li [0]: the strain at which the fiber begins to develop force
        \li [1]: the strainAtOneNormForce for which these properties were
                 computed
        \li [2]: kPE
        \li [3]: normalized slope: dF/depsilon at strainAtOneNormForce
                 (stiffness)
        \li [4]: normalized area under the curve from a (Cauchy) strain of 0 to
                 strainAtOneNormForce

    Note that the equation presented in Thelen (2003) has been improved to fit
    passive force-length curves that begin to develop force at a strain that is
    greater than 0. This was done by replacing the e1 terms with (e1-e0) terms.

    <B>Computational Cost</B>
    \verbatim
            ~220 flops
    \endverbatim
   */
   SimTK::Vec5 calcReferencePassiveFiber(double strainAtZeroForce,
                                         double strainAtOneNormForce);

   //<B>NO LONGER USED</B>
    double calcCurvinessOfBestFit(double e0, double e1, double k0, double k1,
                                 double area, double relTol);

    SmoothSegmentedFunction m_curve;
    double m_stiffnessAtOneNormForceInUse;
    double m_stiffnessAtLowForceInUse;
    double m_curvinessInUse;
    bool   m_fittedCurveBeingUsed;

};

}

#endif // OPENSIM_FIBER_FORCE_LENGTH_CURVE_H_
