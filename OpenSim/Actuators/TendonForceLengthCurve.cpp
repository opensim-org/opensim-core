/* -------------------------------------------------------------------------- *
 *                    OpenSim:  TendonForceLengthCurve.cpp                    *
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
#include "TendonForceLengthCurve.h"
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

static int RefTendon_e0_Idx        = 0;
static int RefTendon_etoe_Idx      = 1;
static int RefTendon_Ftoe_Idx      = 2;
static int RefTendon_ktoe_Idx      = 3;
static int RefTendon_klin_Idx      = 4;
static int RefTendon_normToePE_Idx = 5;
static int RefTendon_normPE_Idx    = 6;

//==============================================================================
// CONSTRUCTION, COPY CONSTRUCTION, ASSIGNMENT
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy
// assignment operator.

TendonForceLengthCurve::TendonForceLengthCurve()
{
    setNull();
    constructProperties();
    setName(getConcreteClassName());
    ensureCurveUpToDate();
}

TendonForceLengthCurve::TendonForceLengthCurve(double strainAtOneNormForce,
                                               double stiffnessAtOneNormForce,
                                               double normForceAtToeEnd,
                                               double curviness)
{
    setNull();
    constructProperties();
    setName(getConcreteClassName());

    setStrainAtOneNormForce(strainAtOneNormForce);
    setOptionalProperties(stiffnessAtOneNormForce,normForceAtToeEnd,curviness);

    ensureCurveUpToDate();
}

TendonForceLengthCurve::TendonForceLengthCurve(double strainAtOneNormForce)
{
    setNull();
    constructProperties();
    setName(getConcreteClassName());

    setStrainAtOneNormForce(strainAtOneNormForce);
    ensureCurveUpToDate();
}

void TendonForceLengthCurve::setNull()
{
    setAuthors("Matthew Millard and Ajay Seth");
}

void TendonForceLengthCurve::constructProperties()
{
    // A 4.9% strain matches the strain observed by Maganaris et al. during
    // in-vivo measurements of tendon strain during a maximum isometric
    // contraction.
    constructProperty_strain_at_one_norm_force(0.049);
    constructProperty_stiffness_at_one_norm_force();
    constructProperty_norm_force_at_toe_end();
    constructProperty_curviness();
}

void TendonForceLengthCurve::buildCurve(bool computeIntegral)
{
    SmoothSegmentedFunction* f = SmoothSegmentedFunctionFactory::
        createTendonForceLengthCurve(get_strain_at_one_norm_force(),
                                     m_stiffnessAtOneNormForceInUse,
                                     m_normForceAtToeEndInUse,
                                     m_curvinessInUse,
                                     computeIntegral,
                                     getName());
    m_curve = *f;
    delete f;
    setObjectIsUpToDateWithProperties();
}

void TendonForceLengthCurve::ensureCurveUpToDate()
{
    if(isObjectUpToDateWithProperties()) {
        return;
    }

    if(!getProperty_stiffness_at_one_norm_force().empty()
        && !getProperty_norm_force_at_toe_end().empty()
        && !getProperty_curviness().empty()) {

        m_stiffnessAtOneNormForceInUse = get_stiffness_at_one_norm_force();
        m_normForceAtToeEndInUse       = get_norm_force_at_toe_end();
        m_curvinessInUse               = get_curviness();
        m_isFittedCurveBeingUsed       = false;

    } else if(getProperty_stiffness_at_one_norm_force().empty()
        && getProperty_norm_force_at_toe_end().empty()
        && getProperty_curviness().empty()) {

        // Get properties of the reference curve. The reference tendon is from
        // Thelen (2003) and is far too stiff.

        double e0 = get_strain_at_one_norm_force();
        //SimTK::Vector refTendon = calcReferenceTendon(e0);
        //tdnProp[RefTendon_etoe_Idx] = eToe;
        //tdnProp[RefTendon_Ftoe_Idx] = FToe;

        // Assign the stiffness. By eyeball, this agrees well with Maganaris and
        // Paul (2002), Magnusson (2001), and Sharkey (1995) in-vitro tendon
        // data.
        m_stiffnessAtOneNormForceInUse = 1.375/e0;
        m_normForceAtToeEndInUse       = 2.0/3.0;
        m_curvinessInUse               = 0.5;
        m_isFittedCurveBeingUsed       = true;

    } else {
        // Ensure that the reference curve is being used to populate either all
        // optional properties or none of them; otherwise, there would be an
        // inconsistency in the computed curve and it wouldn't reflect the
        // reference curve.

        SimTK_ERRCHK1_ALWAYS(false,
            "TendonForceLengthCurve::ensureCurveUpToDate()",
            "%s: Optional parameters stiffness_at_one_norm_force, "
            "norm_force_at_toe_end, and curviness must either all be set or "
            "all be empty.", getName().c_str());
    }

    buildCurve();
}

//==============================================================================
// GET AND SET METHODS
//==============================================================================
double TendonForceLengthCurve::getStrainAtOneNormForce() const
{   return get_strain_at_one_norm_force(); }
double TendonForceLengthCurve::getStiffnessAtOneNormForceInUse() const
{   return m_stiffnessAtOneNormForceInUse; }
double TendonForceLengthCurve::getCurvinessInUse() const
{   return m_curvinessInUse; }
double TendonForceLengthCurve::getNormForceAtToeEndInUse() const
{   return m_normForceAtToeEndInUse; }
bool TendonForceLengthCurve::isFittedCurveBeingUsed() const
{   return m_isFittedCurveBeingUsed; }

void TendonForceLengthCurve::
setStrainAtOneNormForce(double aStrainAtOneNormForce)
{
    set_strain_at_one_norm_force(aStrainAtOneNormForce);
    ensureCurveUpToDate();
}

void TendonForceLengthCurve::setOptionalProperties(
                                            double aStiffnessAtOneNormForce,
                                            double aNormForceAtToeEnd,
                                            double aCurviness)
{
    SimTK_ERRCHK_ALWAYS(aStiffnessAtOneNormForce > SimTK::SignificantReal*1000,
        "TendonForceLengthCurve::setOptionalProperties",
        "The tendon must have a non-zero stiffness");
    SimTK_ERRCHK_ALWAYS(aNormForceAtToeEnd > SimTK::SignificantReal*1000
                        && aNormForceAtToeEnd <= 1,
        "TendonForceLengthCurve::setOptionalProperties",
        "The tendon must have a normForceAtToeEnd between 0 and 1");
    SimTK_ERRCHK_ALWAYS(aCurviness >= 0 && aCurviness <= 1,
        "TendonForceLengthCurve::setOptionalProperties",
        "The tendon must have a curviness between 0 and 1");

    set_stiffness_at_one_norm_force(aStiffnessAtOneNormForce);
    set_norm_force_at_toe_end(aNormForceAtToeEnd);
    set_curviness(aCurviness);
    ensureCurveUpToDate();
}

//==============================================================================
// OpenSim::Function Interface
//==============================================================================
SimTK::Function* TendonForceLengthCurve::createSimTKFunction() const
{
    // Back the OpenSim::Function with this SimTK::Function.
    return SmoothSegmentedFunctionFactory::createTendonForceLengthCurve(
                get_strain_at_one_norm_force(),
                m_stiffnessAtOneNormForceInUse,
                m_normForceAtToeEndInUse,
                m_curvinessInUse,
                true,
                getName());
}

//==============================================================================
// SERVICES
//==============================================================================
double TendonForceLengthCurve::calcValue(double aNormLength) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "TendonForceLengthCurve: Tendon is not up-to-date with its properties");
    return m_curve.calcValue(aNormLength);
}

double TendonForceLengthCurve::calcDerivative(double aNormLength,
                                              int order) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "TendonForceLengthCurve: Tendon is not up-to-date with its properties");
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2,
        "TendonForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);

    return m_curve.calcDerivative(aNormLength,order);
}

double TendonForceLengthCurve::
    calcDerivative(const std::vector<int>& derivComponents,
                   const SimTK::Vector& x) const
{
    return m_curve.calcDerivative(derivComponents, x);
}

double TendonForceLengthCurve::calcIntegral(double aNormLength) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "TendonForceLengthCurve: Tendon is not up-to-date with its properties");

    if (!m_curve.isIntegralAvailable()) {
        TendonForceLengthCurve* mutableThis =
            const_cast<TendonForceLengthCurve*>(this);
        mutableThis->buildCurve(true);
    }

    return m_curve.calcIntegral(aNormLength);
}

SimTK::Vec2 TendonForceLengthCurve::getCurveDomain() const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "TendonForceLengthCurve: Tendon is not up-to-date with its properties");
    return m_curve.getCurveDomain();
}

void TendonForceLengthCurve::printMuscleCurveToCSVFile(const std::string& path)
{
    ensureCurveUpToDate();

    double xmin = 0.9;
    double xmax = 1.0+get_strain_at_one_norm_force()*1.1;

    m_curve.printMuscleCurveToCSVFile(path, xmin, xmax);
}

//==============================================================================
// PRIVATE SERVICES
//==============================================================================
SimTK::Vector TendonForceLengthCurve::
calcReferenceTendon(double strainAtOneNormForce)
{
    SimTK::Vector tdnProp(7);

    // Calculate the stiffness given the strain at 1 unit of normalized force.
    // Use the extrapolated exponential tendon curve documented in Thelen
    // (2003).
    double FToe = 0.33; //Normalized force at which the transition from the
                        //  exponential to the linear function is made.
    double kToe = 3.0;  //The exponential shape factor.
    double e0   = strainAtOneNormForce;

    //Compute the normalized strain at which the transition from an exponential
    // to a linear function is made.
    double FkToe   = FToe*kToe;
    double expkToe = exp(kToe);
    double eToe    = FkToe*expkToe*e0 / (-0.1e1+ FkToe*expkToe
                                         + expkToe - FToe*expkToe + FToe);
    double kLin    = (1-FToe) / (e0 - eToe);

    // Compute the (normalized) potential energy stored at e0. For a tendon with
    // a strain of 0.04 under 1 unit load, the area computed using the
    // trapezoidal method in Matlab is 1.266816749781739e-002.

    // Compute the potential energy stored in the toe region of the exponential.
    double peNormToe = (FToe/(expkToe-1))*(   ((eToe/kToe)*exp(kToe) - eToe)
                                            - ((eToe/kToe)*exp(0.0)  -  0.0) );
    // Compute the potential energy stored in the linear region of the curve
    // between the toe region and e0.
    double peNormLin = (kLin*(0.5*e0*e0 - eToe*e0) + FToe*e0)
                     - (kLin*(0.5*eToe*eToe - eToe*eToe) + FToe*eToe);

    double peNormTotal = peNormToe + peNormLin;

    tdnProp[RefTendon_e0_Idx]        = strainAtOneNormForce;
    tdnProp[RefTendon_etoe_Idx]      = eToe;
    tdnProp[RefTendon_Ftoe_Idx]      = FToe;
    tdnProp[RefTendon_ktoe_Idx]      = kToe;
    tdnProp[RefTendon_klin_Idx]      = kLin;
    tdnProp[RefTendon_normToePE_Idx] = peNormToe;
    tdnProp[RefTendon_normPE_Idx]    = peNormTotal;

    return tdnProp;
}
