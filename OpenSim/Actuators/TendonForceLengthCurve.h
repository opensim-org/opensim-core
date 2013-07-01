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
/** This class serves as a serializable TendonForceLengthCurve, for use in
    muscle models. The user can control the strain the tendon undergoes at 1
    unit load (e0), its stiffness at a strain of e0, and the shape of the tendon
    curve (its 'curviness') using the following parameters:

    <B>Manditory Properties</B>
    \li strainAtOneNormForce

    <B>Optional Properties</B>
    \li stiffnessAtOneNormForce
    \li curviness

    All parameters but the strain of the tendon at 1 unit load (e0) are
    optional. Note that either both optional parameters must be left blank, or
    both must be provided. Filling in one optional parameter but not the other
    will throw an exception when the curve is built.

    The shape of the curve can be varied from a close approximation of a line to
    a sharply bent curve using the optional parameters.

    \image html fig_TendonForceLengthCurve.png

    If the optional parameters are not provided, the tendon force-length curve
    is fit to match the average dimensionless in-vivo tendon curve reported by
    Maganarius et al. and Magnusson et al. In addition, the generated curve will
    have a characteristic toe region that is fit to the in-vivo literature. The
    curve is fit using only the strainAtOneNormForce and the following
    heuristic:

    \li stiffnessAtOneNormForce = 1.375/strainAtOneNormForce
    \li normForceAtToeEnd = 2.0/3.0
    \li curviness = 0.5

    For strains less than 0 or greater than the toe strain (which is computed
    using normForceAtToeEnd and stiffnessAtOneNormForce), the C2-continuous
    curve is linearly extrapolated.

    Note that this curve is not being fit to the commonly used linearly
    extrapolated exponential curve documented by Thelen, as it makes the toe
    region about half as stiff as both the in-vitro and in-vivo data indicate is
    reasonable. Additionally, the linear section of the curve would be nearly
    twice as stiff as the data indicates is reasonable.

    For more details on the fitting process, see the following functions:
    \li TendonForceLengthCurve()
    \li getStiffnessAtOneNormForceInUse()
    \li getNormForceAtToeEndInUse()
    \li getCurvinessInUse()

    <B>Usage</B>
    This object should be updated through the set methods provided. These set
    methods will take care of rebuilding the object correctly. If you modify the
    properties directly, the object will not be rebuilt, and upon calling any
    functions, an exception will be thrown because the object is out-of-date
    with its properties.

    <B>References</B>
    \li Lewis, G., Shaw, K.M. (1997). Tensile properties of human tendo
        Achillis: effect of donor age and strain rate. The Journal of Foot and
        Ankle Surgery 36:435-445.
    \li Maganaris, C.N., Paul, J.P. (2002). Tensile properties of the in vivo
        grastrocnemius tendon. Journal of Biomechanics 35:1639-1646.
    \li Magnusson, S.P., Aagaard, P., Rosager, S., Dyhre-Poulsen, P., Kjaer, M.
        (2001). Load-displacement properties of the human triceps surae
        aponeurosis in vivo. Journal of Physiology 531:277-288.
    \li Thelen, D.G. (2003). Adjustment of muscle mechanics model parameters to
        simulate dynamic contractions in older adults. ASME Journal of
        Biomechanical Engineering 125:70-77.

    <B>Computational Cost Details</B>
    All computational costs assume the following operation costs:
    \verbatim
        Operation Type : #flops
    *,+,-,=,Boolean Op : 1
                     / : 10
                  sqrt : 20
                  trig : 40
    \endverbatim
    Since these relative weightings will vary from processor to processor, all
    of the quoted computational costs are approximate.

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
        "Tendon strain at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(norm_force_at_toe_end, double,
        "Normalized force developed at the end of the toe region");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_one_norm_force, double,
        "Tendon stiffness at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(curviness, double,
        "Tendon curve bend, from linear (0) to maximum bend (1)");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** The default constructor creates an object with a default name that
    doesn't yet define a curve. Calling this function is equivalent to creating
    a fitted curve using the default strainAtOneNormForce.

    <B>Default Parameter</B>
    \verbatim
        strainAtOneNormForce = 0.04
    \endverbatim
    **/
    TendonForceLengthCurve();

    /** Constructs a C2-continuous tendon force-length curve.
    @param strainAtOneNormForce
        The tendon strain at which the tendon develops 1 unit of normalized
        force. The definition of strain used for this quantity is consistent
        with the Cauchy or engineering definition of strain: strain = (l-l0)/l0,
        where l is the current tendon length and l0 is its slack length. In this
        context, strainAtOneNormForce = 0.04 means that the tendon will develop
        a tension of 1 normalized force when it is strained by 4% of its slack
        length or, equivalently, is stretched to 1.04 times its resting length.
    @param stiffnessAtOneNormForce
        The normalized stiffness (or slope) of the tendon curve when the tendon
        is strained by strainAtOneNormForce under a load of 1 normalized unit of
        force.
    @param normForceAtToeEnd
        The normalized force developed at the end of the 'toe' region. The toe
        region lies between 0 strain and some intermediate strain less than the
        strain required to develop 1 unit of normalized force. The toe region is
        nonlinear and more compliant than the rest of the tendon curve.
    @param curviness
        A dimensionless parameter between 0 and 1 that controls how the curve is
        drawn: a value of 0 will create a curve that is very close to a straight
        line segment, while a value of 1 will create a curve that smoothly fills
        the corner formed by the linear extrapolation of
        'stiffnessAtOneNormForce' and the x-axis, as shown in the figure.
    @param muscleName
        The name of the muscle to which this curve belongs. The muscle name is
        used to create the name of this curve simply by appending
        "_TendonForceLengthCurve" to the string in muscleName. The curve name is
        used for reporting meaningful error messages and for naming the XML
        version of this curve when it is serialized.

    \image html fig_TendonForceLengthCurve.png

    This curve has the advantage of being C2-continuous, which results in faster
    simulations when compared to the popular method of using a linearly
    extrapolated exponential (C0-continuous) curve to parameterize the tendon
    force-length relationship. See Millard et al. (2013) for details.

    <B>Conditions</B>
    \verbatim
        strainAtOneNormForce > 0
        stiffnessAtOneNormForce > 1/strainAtOneNormForce
        0 < normForceAtToeEnd < 1
        0 <= curviness <= 1
    \endverbatim

    <B>Computational Cost</B>
    \verbatim
        ~174,100 flops
    \endverbatim

    <B>Example</B>
    @code
        TendonForceLengthCurve fseCurve3(0.10, 50, 0.75, "soleus");
        double fseVal   = fseCurve3.calcValue(0.02);
        double dfselVal = fseCurve3.calcDerivative(0.02, 1);
    @endcode
    */
    TendonForceLengthCurve(double strainAtOneNormForce,
                           double stiffnessAtOneNormForce,
                           double normForceAtToeEnd,
                           double curviness,
                           const std::string& muscleName);

    /** Constructs a C2-continuous tendon force-length curve that is fit to
    match the shape of the tendon load-displacement curves reported in in-vitro
    load-displacement experiments using the strains reported in in-vivo data.

    @param strainAtOneNormForce
        The strain at which the tendon develops 1 unit of normalized force. The
        definition of strain used for this quantity is consistent with the
        Cauchy or engineering definition of strain: strain = (l-l0)/l0, where l
        is the current tendon length and l0 is its slack length. In this
        context, strainAtOneNormForce = 0.04 means that the tendon will develop
        a tension of 1 normalized force when it is strained by 4% of its slack
        length or, equivalently, is stretched to 1.04 times its resting length.
    @param muscleName
        The name of the muscle to which this curve belongs. The muscle name is
        used to create the name of this curve simply by appending
        "_TendonForceLengthCurve" to the string in muscleName. The curve name is
        used for reporting meaningful error messages and for naming the XML
        version of this curve when it is serialized.

    This constructor will create a C2-continuous tendon force-length curve that
    is fit to match the average dimensionless in-vivo tendon curve reported by
    Maganarius et al. and Magnusson et al. In addition, the generated curve will
    have a characteristic toe region that is fit to the in-vivo literature. The
    curve is fit using only the strainAtOneNormForce and the following
    heuristic:

    \li stiffnessAtOneNormForce = 1.375/strainAtOneNormForce
    \li normForceAtToeEnd = 2.0/3.0
    \li curviness = 0.5

    For strains less than 0 or greater than strainAtOneNormForce, the
    C2-continuous curve is linearly extrapolated.

    Note that this curve is not being fitted to the commonly used linearly
    extrapolated exponential curve documented by Thelen (2003), as this curve
    makes the toe region about half as stiff as both the in-vitro and in-vivo
    data indicate is reasonable. Additionally, the linear section of the curve
    would be nearly twice as stiff as the data indicates is reasonable.

    <B>Conditions</B>
    \verbatim
        strainAtOneNormForce > 0
    \endverbatim

    <B>Computational Cost</B>
    \verbatim
        ~174,100 flops
    \endverbatim

    <B>Default Parameter Value</B>
    \verbatim
        strainAtOneNormForce = 0.049
    \endverbatim

    <B>Example</B>
    @code
        TendonForceLengthCurve fseCurve3(0.05, "soleus");
        double fseVal   = fseCurve3.calcValue(1.02);
        double dfselVal = fseCurve3.calcDerivative(1.02, 1);
    @endcode
    */
    TendonForceLengthCurve(double strainAtOneNormForce,
                           const std::string& muscleName);

    /** @returns The strain at which the tendon develops 1 unit of normalized
    force. The definition of strain used for this quantity is consistent with
    the Cauchy or engineering definition of strain: strain = (l-l0)/l0, where l
    is the current tendon length and l0 is its slack length. In this context,
    strainAtOneNormForce = 0.04 means that the tendon will develop a tension of
    1 normalized unit of force when it is strained by 4% of its slack length or,
    equivalently, is stretched to 1.04 times its resting length. */
    double getStrainAtOneNormForce() const;

    /** @returns The normalized stiffness (or slope) of the tendon curve when
    the tendon is strained by strainAtOneNormForce under a load of 1 normalized
    unit of force.

    If the optional parameter stiffnessAtOneNormForce has beens set, its value
    is returned; otherwise, this parameter is assigned a value of
    1.375/strainAtOneNormForce. This value (a heuristic) appears to agree well
    with the in-vivo data of Magnusson et al. and Magnaris and Paul, as well as
    the in-vitro data reported by Lewis for the Achilles tendon.
    */
    double getStiffnessAtOneNormForceInUse() const;

    /** @returns The normalized force developed at the point in the curve where
    the toe region transitions to the linear stiffness region. A value of
    2.0/3.0 is used by default, as it best fits the tendon force-length curves
    reported in the literature. */
    double getNormForceAtToeEndInUse() const;

    /** @returns A dimensionless parameter between 0 and 1 that controls how the
    curve is drawn: a value of 0 will create a curve that is very close to a
    straight line segment, while a value of 1 will create a curve that smoothly
    fills the corner formed by the linear extrapolation of
    'stiffnessAtOneNormForce' and the x-axis, as shown in the figure in the
    class description.

    If the optional 'curviness' parameter has been set, its value is returned;
    otherwise, this parameter is assigned a value of 0.5, producing a toe region
    that agrees well with the in-vitro curves reported by Lewis. */
    double getCurvinessInUse() const;

    /** @returns True if the optional properties are empty and the fitted curve
    is being used, and false if the optional properties are filled and are being
    used to construct the curve. */
    bool isFittedCurveBeingUsed() const;

    /** @param aStrainAtOneNormForce The tendon strain at which the tendon
    develops 1 unit of normalized force. The definition of strain used for this
    quantity is consistent with the Cauchy or engineering definition of strain:
    strain = (l-l0)/l0, where l is the current tendon length and l0 is its slack
    length. In this context, strainAtOneNormForce = 0.04 means that the tendon
    will develop a tension of 1 normalized force when it is strained by 4% of
    its slack length or, equivalently, is stretched to 1.04 times its resting
    length.

    <B>Computational Cost</B>
    The curve is rebuilt at a cost of ~174,100 flops */
    void setStrainAtOneNormForce(double aStrainAtOneNormForce);

    /**
    @param aStiffnessAtOneNormForce The normalized stiffness (or slope) of the
        curve when the tendon is strained by strainAtOneNormForce under a load
        of 1 normalized unit of force.
    @param aNormForceAtToeEnd The normalized force developed at the end of the
        toe region, after which the force-length curve becomes linear.
    @param aCurviness A dimensionless parameter between 0 and 1 that controls
        how the curve is drawn: a value of 0 will create a curve that is very
        close to a straight line segment, while a value of 1 will create a curve
        that smoothly fills the corner formed by the linear extrapolation of
        'stiffnessAtOneNormForce' and the x-axis, as shown in the figure.
    <B>Computational Cost</B>
    The curve is rebuilt at a cost of ~174,100 flops */
    void setOptionalProperties(double aStiffnessAtOneNormForce,
                               double aNormForceAtToeEnd,
                               double aCurviness);

    /** Calculates the value of the tendon force-length curve evaluated at the
    specified normalized tendon length.
    @param aNormLength The normalized tendon length used to evaluate the
        tendon force-length curve for the corresponding normalized force.
        Parameter aNormLength = l/l0, where l is the current length of the
        tendon and l0 is its slack length. A normalized length of 1.0 means that
        the tendon is at its resting length.
    @return The value of the normalized force generated by the tendon.
    <B>Computational Cost</B>
    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim */
    double calcValue(double aNormLength) const;

    /** Calculates the derivative of the tendon force-length curve with respect
    to the specified normalized tendon length.
        @param aNormLength The normalized tendon length used to evaluate the
    tendon force-length curve for the corresponding normalized force. Parameter
    aNormLength = l/l0, where l is the length of the tendon and l0 is the tendon
    slack length. A normalized length of 1.0 means the tendon is at its resting
    length.
        @param order Order of the derivative. Only values of 0, 1, and 2 are
    acceptable.
        @return The derivative of the normalized tendon force-length curve with
    respect to the normalized tendon length.

    <B>Computational Cost</B>
    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops
    \endverbatim */
    double calcDerivative(double aNormLength, int order) const;

    /**
    @param aNormLength The normalized fiber length used to evaluate the tendon
        force-length curve for the corresponding normalized force. Here,
        aNormLength = l/l0, where l is the current tendon length and l0 is its
        slack length. Thus, a normalized length of 1.0 means that the tendon is
        at its resting length.
    @return Computes the normalized area under the curve. For this curve, this
        quantity corresponds to the normalized potential energy stored in the
        tendon. To calculate the potental energy stored in the tendon in units
        of Joules, simply multiply the returned quantity by normForce*normLength
        (where normForce is the number of Newtons represented by a normalized
        force of 1.0, and normLength is the number of meters represented by a
        normalized length of 1.0).
    <B>Computational Cost</B>
    \verbatim
        x in curve domain  : ~13 flops
        x in linear section: ~19 flops
    \endverbatim */
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
    curve name (e.g., "bicepfemoris_TendonForceLengthCurve.csv"). This function
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
    to strains beyond which the tendon generates normalized forces greater than
    1.

    The curve is sampled quite densely: the tendon force-length .csv file will
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
        data = csvread('bicepfemoris_TendonForceLengthCurve.csv', 1, 0);
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

    /** <B>No Longer Used</B>
    @param strainAtOneNormForce The Cauchy or engineering strain of the tendon
        at 1 unit of normalized force. Cauchy or engineering strain is defined
        as strainAtOneNormForce = (l-l0)/l0, where l is the length of the tendon
        under tension and l0 is its resting or slack length.
    @returns A vector that contains the parameters that define the properties of
        the extrapolated exponental curve that is used to construct the
        reference tendon defined in Thelen 2003. The fields of the vector are
        as follows:
        [0]: e0
        [1]: etoe
        [2]: Ftoe
        [3]: ktoe
        [4]: klin
        [5]: normalized potental energy from a strain of 0 to etoe
        [6]: normalized potental energy from a strain of 0 to e0

    <B>Computational Cost</B>
    \verbatim
        ~211 flops
    \endverbatim */
    SimTK::Vector calcReferenceTendon(double strainAtOneNormForce);

    void setNull();
    void constructProperties();

    /** This function will take all of the current property values. If they have
    changed since the last time the curve was built, the curve is rebuilt.

    <B>Computational Cost</B>
    \verbatim
        Curve construction costs ~20,500 flops
    \endverbatim
    */
    void buildCurve(bool computeIntegral = false);

    SmoothSegmentedFunction   m_curve;

    double m_normForceAtToeEndInUse;
    double m_stiffnessAtOneNormForceInUse;
    double m_curvinessInUse;
    bool   m_isFittedCurveBeingUsed;

};

}

#endif // OPENSIM_TENDON_FORCE_LENGTH_CURVE_H_
