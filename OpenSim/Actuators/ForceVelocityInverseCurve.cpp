/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ForceVelocityInverseCurve.cpp                   *
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
#include "ForceVelocityInverseCurve.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//=============================================================================
// CONSTRUCTION
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

ForceVelocityInverseCurve::ForceVelocityInverseCurve()
{
    setNull();
    constructProperties();
    setName("default_ForceVelocityInverseCurve");
    ensureCurveUpToDate();
}

ForceVelocityInverseCurve::ForceVelocityInverseCurve
   (double concentricMinSlope, 
    double isometricMaxSlope,        
    double eccentricMinSlope,
    double maxEccentricVelocityForceMultiplier,
    double concentricCurviness,
    double eccentricCurviness,
    const std::string& muscleName)
{
    setNull();
    constructProperties();
    setName(muscleName + "_ForceVelocityInverseCurve");

    
    set_min_concentric_slope(concentricMinSlope);
    set_isometric_slope(isometricMaxSlope);
    set_min_eccentric_slope(eccentricMinSlope);
    set_max_eccentric_velocity_force_multiplier(maxEccentricVelocityForceMultiplier);
    set_concentric_curviness(concentricCurviness);
    set_eccentric_curviness(eccentricCurviness);

    ensureCurveUpToDate();
}


void ForceVelocityInverseCurve::setNull()
{
    
    setAuthors("Matthew Millard");
}

void ForceVelocityInverseCurve::constructProperties()
{
    constructProperty_min_concentric_slope(0.1);
    constructProperty_isometric_slope(5);
    constructProperty_min_eccentric_slope(0.1);
    constructProperty_max_eccentric_velocity_force_multiplier(1.4);
    constructProperty_concentric_curviness(0.5);
    constructProperty_eccentric_curviness(0.9);
}

void ForceVelocityInverseCurve::buildCurve()
{      
        double dydxC =  get_min_concentric_slope();
        double dydxIso= get_isometric_slope();
        double dydxE =  get_min_eccentric_slope();
        double fmax  =  get_max_eccentric_velocity_force_multiplier();
        double ccurv =  get_concentric_curviness();
        double ecurv =  get_eccentric_curviness();

        //Here's where you call the SmoothSegmentedFunctionFactory
        SmoothSegmentedFunction tmp = 
            SmoothSegmentedFunctionFactory::
            createFiberForceVelocityInverseCurve(  fmax,
                                            dydxC,
                                            dydxIso,
                                            dydxE,
                                            ccurv,
                                            ecurv,
                                            false,
                                            getName());

        this->m_curve = tmp;
        setObjectIsUpToDateWithProperties();      
    
}

void ForceVelocityInverseCurve::ensureCurveUpToDate()
{
    if(isObjectUpToDateWithProperties() == false){
        buildCurve();
    }

    //Since the name is not counted as a property, but it can change,
    //and needs to be kept up to date.
    std::string name = getName();
    m_curve.setName(name);
}

//=============================================================================
// MODEL COMPPONENT INTERFACE
//=============================================================================
void ForceVelocityInverseCurve::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);
    ensureCurveUpToDate();
}

void ForceVelocityInverseCurve::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);
}

void ForceVelocityInverseCurve::
addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
    "FiberForceVelocityInverseCurve: Curve is not"
    " to date with its properties"); 
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double ForceVelocityInverseCurve::getConcentricMinSlope() const
{
    return get_min_concentric_slope();
}

double ForceVelocityInverseCurve::getIsometricMaxSlope() const
{
    return get_isometric_slope();
}

double ForceVelocityInverseCurve::getEccentricMinSlope() const
{
    return get_min_eccentric_slope();
}

double ForceVelocityInverseCurve::getMaxEccentricVelocityForceMultiplier() const
{
    return get_max_eccentric_velocity_force_multiplier();
}
    
double ForceVelocityInverseCurve::getConcentricCurviness() const
{
    return get_concentric_curviness();
}
    
double ForceVelocityInverseCurve::getEccentricCurviness() const
{
    return get_eccentric_curviness();
}


void ForceVelocityInverseCurve::setCurveShape(double aConcentricMinSlope,
                                                double aIsometricMaxSlope,
                                                double aEccentricMinSlope,
                                                double aMaxForceMultiplier)
{
    set_min_concentric_slope(aConcentricMinSlope); 
    set_isometric_slope(aIsometricMaxSlope);
    set_min_eccentric_slope(aEccentricMinSlope);
    set_max_eccentric_velocity_force_multiplier(aMaxForceMultiplier);
    ensureCurveUpToDate();
}




void ForceVelocityInverseCurve::setConcentricCurviness(double aConcentricCurviness)
{
        set_concentric_curviness(aConcentricCurviness);
        ensureCurveUpToDate();
}


void ForceVelocityInverseCurve::setEccentricCurviness(double aEccentricCurviness)
{
        set_eccentric_curviness(aEccentricCurviness);
        ensureCurveUpToDate();
}


//=============================================================================
// SERVICES
//=============================================================================

double ForceVelocityInverseCurve::
    calcValue(double aForceVelocityMultiplier) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
    "FiberForceVelocityInverseCurve: Curve is not"
    " to date with its properties");    

    return m_curve.calcValue(aForceVelocityMultiplier);
}

double ForceVelocityInverseCurve::
    calcDerivative(double aForceVelocityMultiplier, int order) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
    "FiberForceVelocityInverseCurve: Curve is not"
    " to date with its properties");    

    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "ForceVelocityInverseCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
        
    return m_curve.calcDerivative(aForceVelocityMultiplier,order);
}

SimTK::Vec2 ForceVelocityInverseCurve::getCurveDomain() const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
    "FiberForceVelocityInverseCurve: Curve is not"
    " to date with its properties");    

    return m_curve.getCurveDomain();
}

void ForceVelocityInverseCurve::
    printMuscleCurveToCSVFile(const std::string& path)
{
    ensureCurveUpToDate();

    double xmin = 0;
    double xmax = get_max_eccentric_velocity_force_multiplier();

    m_curve.printMuscleCurveToCSVFile(path,xmin,xmax);
}
