/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ForceVelocityCurve.cpp                      *
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
#include "ForceVelocityCurve.h"
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==============================================================================
// CONSTRUCTION
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy
// assignment operator.
ForceVelocityCurve::ForceVelocityCurve()
{
    setNull();
    constructProperties();
    setName(getConcreteClassName());
    ensureCurveUpToDate();
}

ForceVelocityCurve::ForceVelocityCurve(double concentricSlopeAtVmax,
                                   double concentricSlopeNearVmax,
                                   double isometricSlope,
                                   double eccentricSlopeAtVmax,
                                   double eccentricSlopeNearVmax,
                                   double maxEccentricVelocityForceMultiplier,
                                   double concentricCurviness,
                                   double eccentricCurviness)
{
    setNull();
    constructProperties();
    setName(getConcreteClassName());

    set_concentric_slope_at_vmax(concentricSlopeAtVmax);
    set_concentric_slope_near_vmax(concentricSlopeNearVmax);
    set_isometric_slope(isometricSlope);
    set_eccentric_slope_at_vmax(eccentricSlopeAtVmax);
    set_eccentric_slope_near_vmax(eccentricSlopeNearVmax);
    set_max_eccentric_velocity_force_multiplier(
        maxEccentricVelocityForceMultiplier);

    set_concentric_curviness(concentricCurviness);
    set_eccentric_curviness(eccentricCurviness);

    ensureCurveUpToDate();
}

void ForceVelocityCurve::setNull()
{
    setAuthors("Matthew Millard");
}

void ForceVelocityCurve::constructProperties()
{
    constructProperty_concentric_slope_at_vmax(0.0);
    constructProperty_concentric_slope_near_vmax(0.25);
    constructProperty_isometric_slope(5.0);
    constructProperty_eccentric_slope_at_vmax(0.0);
    constructProperty_eccentric_slope_near_vmax(0.15); //guess for now
    constructProperty_max_eccentric_velocity_force_multiplier(1.4);
    constructProperty_concentric_curviness(0.6);
    constructProperty_eccentric_curviness(0.9);
}

void ForceVelocityCurve::buildCurve()
{
    SimTK::Function* f = createSimTKFunction();
    m_curve = *(static_cast<SmoothSegmentedFunction*>(f));
    delete f;
    setObjectIsUpToDateWithProperties();
}

void ForceVelocityCurve::ensureCurveUpToDate()
{
    if(!isObjectUpToDateWithProperties()) {
        buildCurve();
    }
}

//==============================================================================
// OpenSim::Function Interface
//==============================================================================
SimTK::Function* ForceVelocityCurve::createSimTKFunction() const
{
    // Back the OpenSim::Function with this SimTK::Function.
    return SmoothSegmentedFunctionFactory::createFiberForceVelocityCurve(
                get_max_eccentric_velocity_force_multiplier(),
                get_concentric_slope_at_vmax(),
                get_concentric_slope_near_vmax(),
                get_isometric_slope(),
                get_eccentric_slope_at_vmax(),
                get_eccentric_slope_near_vmax(),
                get_concentric_curviness(),
                get_eccentric_curviness(),
                false,
                getName());
}

//==============================================================================
// GET AND SET METHODS
//==============================================================================
double ForceVelocityCurve::getConcentricSlopeNearVmax() const
{   return get_concentric_slope_near_vmax(); }
double ForceVelocityCurve::getConcentricSlopeAtVmax() const
{   return get_concentric_slope_at_vmax(); }
double ForceVelocityCurve::getIsometricSlope() const
{   return get_isometric_slope(); }
double ForceVelocityCurve::getEccentricSlopeAtVmax() const
{   return get_eccentric_slope_at_vmax(); }
double ForceVelocityCurve::getEccentricSlopeNearVmax() const
{   return get_eccentric_slope_near_vmax(); }
double ForceVelocityCurve::getMaxEccentricVelocityForceMultiplier() const
{   return get_max_eccentric_velocity_force_multiplier(); }
double ForceVelocityCurve::getConcentricCurviness() const
{   return get_concentric_curviness(); }
double ForceVelocityCurve::getEccentricCurviness() const
{   return get_eccentric_curviness(); }

void ForceVelocityCurve::setCurveShape(double aConcentricSlopeAtVmax,
                                       double aConcentricSlopeNearVmax,
                                       double aIsometricSlope,
                                       double aEccentricSlopeAtVmax,
                                       double aEccentricSlopeNearVmax,
                                       double aMaxForceMultiplier)
{
    set_concentric_slope_at_vmax(aConcentricSlopeAtVmax);
    set_concentric_slope_near_vmax(aConcentricSlopeNearVmax);
    set_isometric_slope(aIsometricSlope);
    set_eccentric_slope_at_vmax(aEccentricSlopeAtVmax);
    set_eccentric_slope_near_vmax(aEccentricSlopeNearVmax);
    set_max_eccentric_velocity_force_multiplier(aMaxForceMultiplier);
    ensureCurveUpToDate();
}

void ForceVelocityCurve::setConcentricCurviness(double aConcentricCurviness)
{
    set_concentric_curviness(aConcentricCurviness);
    ensureCurveUpToDate();
}

void ForceVelocityCurve::setEccentricCurviness(double aEccentricCurviness)
{
    set_eccentric_curviness(aEccentricCurviness);
    ensureCurveUpToDate();
}

//==============================================================================
// SERVICES
//==============================================================================
double ForceVelocityCurve::calcValue(double normFiberVelocity) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "ForceVelocityCurve: Curve is not up-to-date with its properties");
    return m_curve.calcValue(normFiberVelocity);
}

double ForceVelocityCurve::calcDerivative(double normFiberVelocity,
                                          int order) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "ForceVelocityCurve: Curve is not up-to-date with its properties");
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2,
        "ForceVelocityCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);

    return m_curve.calcDerivative(normFiberVelocity,order);
}

double ForceVelocityCurve::
    calcDerivative(const std::vector<int>& derivComponents,
                   const SimTK::Vector& x) const
{
    return m_curve.calcDerivative(derivComponents, x);
}

SimTK::Vec2 ForceVelocityCurve::getCurveDomain() const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "ForceVelocityCurve: Curve is not up-to-date with its properties");
    return m_curve.getCurveDomain();
}

void ForceVelocityCurve::printMuscleCurveToCSVFile(const std::string& path)
{
    ensureCurveUpToDate();
    m_curve.printMuscleCurveToCSVFile(path, -1.25, 1.25);
}
