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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Common/SmoothSegmentedFunction.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {
/** This class serves as a serializable TendonForceLengthCurve for use in muscle
    models. The tendon-force-length curve is dimensionless: force is normalized
    to maximum isometric force and length is normalized to tendon slack length.
    The user can adjust the strain the tendon undergoes at 1 unit load (e0), its
    stiffness at a strain of e0, and the shape of the tendon curve (its
    'curviness'):

    @param strainAtOneNormForce
        The tendon strain at which the tendon develops 1 unit of normalized
        force. strainAtOneNormForce = 0.04 means that the tendon will develop a
        tension of 1 normalized force when it is strained by 4% of its slack
        length or, equivalently, is stretched to 1.04 times its resting length.
    @param stiffnessAtOneNormForce
        The normalized stiffness (slope of the tendon curve) when the tendon is
        strained by strainAtOneNormForce under a load of 1 normalized unit of
        force.
    @param normForceAtToeEnd
        The normalized force developed at the end of the 'toe' region. The toe
        region lies between 0 strain and some intermediate strain less than the
        strain required to develop 1 unit of normalized force. The toe region is
        nonlinear and more compliant than the rest of the tendon curve.
    @param curviness
        A dimensionless parameter between 0 and 1 that describes the shape of
        the curve: a value of 0 indicates that the curve is very close to a
        straight line segment and a value of 1 indicates a curve that smoothly
        fills the corner formed by the linear extrapolation of
        'stiffnessAtOneNormForce' and the x-axis, as shown in the figure.

    Note that we use the Cauchy or engineering definition of strain throughout:
    strain = (l-l0)/l0, where l is the current tendon length and l0 is its slack
    length.

    <B>Required Parameters</B>
    \li strainAtOneNormForce

    <B>Optional Parameters</B>
    \li stiffnessAtOneNormForce
    \li normForceAtToeEnd
    \li curviness

    \image html fig_TendonForceLengthCurve.png

    <B>Conditions</B>
    \verbatim
    strainAtOneNormForce > 0
    stiffnessAtOneNormForce > 1/strainAtOneNormForce
    0 < normForceAtToeEnd < 1
    0 <= curviness <= 1
    \endverbatim

    All parameters but the strain of the tendon at 1 unit load (e0) are
    optional. Note that filling in one optional parameter but not the others
    will throw an exception when the curve is built. The optional parameters can
    be used to vary the shape of the curve from a close approximation of a line
    to a sharply-bent curve.

    This curve has the advantage of being C2-continuous, which results in faster
    simulations when compared to the popular method of using a linearly
    extrapolated exponential (C0-continuous) curve to parameterize the
    tendon-force-length relationship. See Millard et al. (2013) for details.

    <B>Default Parameter Values</B>
    If the optional parameters are not specified, the curve is fit to match the
    average dimensionless in-vivo tendon curve reported by Maganaris et al. and
    Magnusson et al. In addition, the generated curve will have a characteristic
    toe region that is fit to the in-vivo literature. Note that this curve is
    not being fit to the commonly used linearly extrapolated exponential curve
    documented by Thelen, as it makes the toe region about half as stiff as both
    the in-vitro and in-vivo data indicate is reasonable. Additionally, the
    linear section of the curve would be nearly twice as stiff as the data
    indicates is reasonable.

    \verbatim
    strainAtOneNormForce ....... 0.049
    stiffnessAtOneNormForce .... 1.375/strainAtOneNormForce
    normForceAtToeEnd .......... 2.0/3.0
    curviness .................. 0.5
    \endverbatim

    <B>Example</B>
    @code
        TendonForceLengthCurve fseCurve(0.049, 28.1, 0.67, 0.5);
        double fseVal   = fseCurve.calcValue(0.02);
        double dfselVal = fseCurve.calcDerivative(0.02, 1);
    @endcode

    Note that this object should be updated through the set methods provided.
    These set methods will take care of rebuilding the curve correctly. If you
    modify the properties directly, the curve will not be rebuilt, and upon
    calling a function like calcValue, calcDerivative, or printCurveToCSVFile,
    an exception will be thrown because the curve is out-of-date with its
    properties.

    <B>References</B>
    \li Lewis, G., Shaw, K.M. (1997) Tensile properties of human tendo Achillis:
        effect of donor age and strain rate. The Journal of Foot and Ankle
        Surgery 36:435--445.
    \li Maganaris, C.N., Paul, J.P. (2002) Tensile properties of the in vivo
        gastrocnemius tendon. Journal of Biomechanics 35:1639--1646.
    \li Magnusson, S.P., Aagaard, P., Rosager, S., Dyhre-Poulsen, P., Kjaer, M.
        (2001) Load-displacement properties of the human triceps surae
        aponeurosis in vivo. Journal of Physiology 531:277--288.
    \li Thelen, D.G. (2003) Adjustment of muscle mechanics model parameters to
        simulate dynamic contractions in older adults. ASME Journal of
        Biomechanical Engineering 125:70--77.

    @author Matt Millard
*/
class OSIMACTUATORS_API TendonForceLengthCurve : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(TendonForceLengthCurve, Function);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(strain_at_one_norm_force, double,
        "Tendon strain at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_one_norm_force, double,
        "Tendon stiffness at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(norm_force_at_toe_end, double,
        "Normalized force developed at the end of the toe region");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(curviness, double,
        "Tendon curve bend, from linear (0) to maximum bend (1)");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** The default constructor creates a tendon-force-length curve using the
    default property values and assigns a default name. */
    TendonForceLengthCurve();

    /** Constructs a tendon-force-length curve using the provided parameters and
    assigns a default name. */
    TendonForceLengthCurve(double strainAtOneNormForce,
                           double stiffnessAtOneNormForce,
                           double normForceAtToeEnd,
                           double curviness);

    /** This constructor will create a C2-continuous tendon-force-length curve
    that is fit to match the average dimensionless in-vivo tendon curve reported
    by Maganaris et al. and Magnusson et al. In addition, the generated curve
    will have a characteristic toe region that is fit to the in-vivo literature.
    @param strainAtOneNormForce
        The tendon strain at which the tendon develops 1 unit of normalized
        force. strainAtOneNormForce = 0.04 means that the tendon will develop a
        tension of 1 normalized force when it is strained by 4% of its slack
        length or, equivalently, is stretched to 1.04 times its resting length.

    <B>Conditions</B>
    \verbatim
    strainAtOneNormForce > 0
    \endverbatim
    */
    TendonForceLengthCurve(double strainAtOneNormForce);

    /** @returns The tendon strain at which the tendon develops 1 unit of
    normalized force. strainAtOneNormForce = 0.04 means that the tendon will
    develop a tension of 1 normalized force when it is strained by 4% of its
    slack length or, equivalently, is stretched to 1.04 times its resting
    length. This property is set to 0.049 by default. */
    double getStrainAtOneNormForce() const;

    /** @returns The normalized stiffness (slope of the tendon curve) when the
    tendon is strained by strainAtOneNormForce under a load of 1 normalized unit
    of force. This property is set to 1.375/strainAtOneNormForce by default. */
    double getStiffnessAtOneNormForceInUse() const;

    /** @returns The normalized force developed at the end of the 'toe' region.
    The toe region lies between 0 strain and some intermediate strain less than
    the strain required to develop 1 unit of normalized force. The toe region is
    nonlinear and more compliant than the rest of the tendon curve. This
    property is set to 2.0/3.0 by default. */
    double getNormForceAtToeEndInUse() const;

    /** @returns A dimensionless parameter between 0 and 1 that describes the
    shape of the curve: a value of 0 indicates that the curve is very close to a
    straight line segment and a value of 1 indicates a curve that smoothly fills
    the corner formed by the linear extrapolation of 'stiffnessAtOneNormForce'
    and the x-axis, as shown in the figure in the class description. This
    property is set to 0.5 by default. */
    double getCurvinessInUse() const;

    /** @returns True if the optional properties are empty and the fitted curve
    is being used, false if the optional properties are filled and are being
    used to construct the curve. */
    bool isFittedCurveBeingUsed() const;

    /**
    @param aStrainAtOneNormForce
        The tendon strain at which the tendon develops 1 unit of normalized
        force. strainAtOneNormForce = 0.04 means that the tendon will develop a
        tension of 1 normalized force when it is strained by 4% of its slack
        length or, equivalently, is stretched to 1.04 times its resting length.
    */
    void setStrainAtOneNormForce(double aStrainAtOneNormForce);

    /**
    @param stiffnessAtOneNormForce
        The normalized stiffness (slope of the tendon curve) when the tendon is
        strained by strainAtOneNormForce under a load of 1 normalized unit of
        force.
    @param normForceAtToeEnd
        The normalized force developed at the end of the 'toe' region. The toe
        region lies between 0 strain and some intermediate strain less than the
        strain required to develop 1 unit of normalized force. The toe region is
        nonlinear and more compliant than the rest of the tendon curve.
    @param curviness
        A dimensionless parameter between 0 and 1 that describes the shape of
        the curve: a value of 0 indicates that the curve is very close to a
        straight line segment and a value of 1 indicates a curve that smoothly
        fills the corner formed by the linear extrapolation of
        'stiffnessAtOneNormForce' and the x-axis, as shown in the figure in the
        class description.
    */
    void setOptionalProperties(double stiffnessAtOneNormForce,
                               double normForceAtToeEnd,
                               double curviness);

    /** Implement the generic OpenSim::Function interface **/
    double calcValue(const SimTK::Vector& x) const override
    {
        return calcValue(x[0]);
    }

    /** Evaluates the tendon-force-length curve at a normalized tendon length of
    'aNormLength'. */
    double calcValue(double aNormLength) const;

    /** Calculates the derivative of the tendon-force-length multiplier with
    respect to the normalized tendon length.
    @param aNormLength
        The normalized length of the tendon.
    @param order
        The order of the derivative. Only values of 0, 1, and 2 are acceptable.
    @return
        The derivative of the tendon-force-length curve with respect to the
        normalized tendon length.
    */
    double calcDerivative(double aNormLength, int order) const;
    
    /// If possible, use the simpler overload above.
    double calcDerivative(const std::vector<int>& derivComponents,
                          const SimTK::Vector& x) const override;

    /** Calculates the normalized area under the curve. Since it is expensive to
    construct, the curve is built only when necessary.
    @param aNormLength
        The normalized length of the tendon.
    @return The normalized area under the curve, which corresponds to the
        normalized potential energy stored in the tendon. To calculate the
        potential energy stored in the tendon in units of Joules, multiply the
        returned quantity by normForce*normLength (where normForce is the number
        of Newtons represented by a normalized force of 1.0 and normLength is
        the number of meters represented by a normalized length of 1.0).
    */
    double calcIntegral(double aNormLength) const;

    /** Returns a SimTK::Vec2 containing the lower (0th element) and upper (1st
    element) bounds on the domain of the curve. Outside this domain, the curve
    is approximated using linear extrapolation.
    @return
        The minimum and maximum value of the domain, x, of the curve y(x).
        Within this range, y(x) is a curve; outside this range, the function
        y(x) is a C2-continuous linear extrapolation.
    */
    SimTK::Vec2 getCurveDomain() const;

    /** Generates a .csv file with a name that matches the curve name (e.g.,
    "bicepsfemoris_TendonForceLengthCurve.csv"). This function is not const to
    permit the curve to be rebuilt if it is out-of-date with its properties.
    @param path
        The full destination path. Note that forward slashes ('/') must be used
        and there should not be a slash after the last folder.

    The file will contain the following data:
    \verbatim
    column: 1 | 2 |     3 |       4
      data: x | y | dy/dx | d2y/dx2
    \endverbatim

    Samples will be taken from the zero-force region, through the curve, out to
    strains beyond which the tendon generates normalized forces greater than 1.
    The curve is sampled quite densely: the tendon-force-length .csv file will
    have 200+20 rows.

    <B>Example</B>
    To read the .csv file into Matlab, you need to set csvread to ignore the
    header row. Since csvread is 0-indexed, the following example will begin
    reading the .csv file from the first column of the second row:
    \verbatim
    data = csvread('bicepsfemoris_TendonForceLengthCurve.csv', 1, 0);
    \endverbatim
    */
    void printMuscleCurveToCSVFile(const std::string& path);

    void ensureCurveUpToDate();
//==============================================================================
// PRIVATE
//==============================================================================
private:
    // OpenSim::Function Interface
    // Create the underlying SimTK::Function that implements the calculations
    // necessary for this curve.
    SimTK::Function* createSimTKFunction() const override;

    /** NO LONGER USED
    @param strainAtOneNormForce
        The tendon strain at which the tendon develops 1 unit of normalized
        force. strainAtOneNormForce = 0.04 means that the tendon will develop a
        tension of 1 normalized force when it is strained by 4% of its slack
        length or, equivalently, is stretched to 1.04 times its resting length.
    @returns A vector that contains the parameters that define the properties of
        the extrapolated exponential curve that is used to construct the
        reference tendon defined in Thelen 2003. The fields of the vector are as
        follows:
            [0]: e0
            [1]: etoe
            [2]: Ftoe
            [3]: ktoe
            [4]: klin
            [5]: normalized potential energy from a strain of 0 to etoe
            [6]: normalized potential energy from a strain of 0 to e0
    */
    SimTK::Vector calcReferenceTendon(double strainAtOneNormForce);

    void setNull();
    void constructProperties();

    // This function will take all of the current property values. If they have
    // changed since the last time the curve was built, the curve is rebuilt.
    void buildCurve(bool computeIntegral = false);

    SmoothSegmentedFunction m_curve;

    double m_normForceAtToeEndInUse;
    double m_stiffnessAtOneNormForceInUse;
    double m_curvinessInUse;
    bool   m_isFittedCurveBeingUsed;

};

}

#endif // OPENSIM_TENDON_FORCE_LENGTH_CURVE_H_
