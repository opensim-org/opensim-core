/* -------------------------------------------------------------------------- *
 *                         Blankevoort1991Ligament.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
 * Author(s): Colin Smith                                                     *
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

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PointForceDirection.h>
#include "Blankevoort1991Ligament.h"

using namespace OpenSim;

//=============================================================================
// CONSTRUCTORS
//=============================================================================

Blankevoort1991Ligament::Blankevoort1991Ligament() : Force() {
    constructProperties();
    setNull();
}

Blankevoort1991Ligament::Blankevoort1991Ligament(std::string name,
        const PhysicalFrame& frame1, SimTK::Vec3 point1, 
        const PhysicalFrame& frame2, SimTK::Vec3 point2)
        : Blankevoort1991Ligament() {
    setName(name);
    upd_GeometryPath().appendNewPathPoint("p1", frame1, point1);
    upd_GeometryPath().appendNewPathPoint("p2", frame2, point2);
}

Blankevoort1991Ligament::Blankevoort1991Ligament(std::string name,
        const PhysicalFrame& frame1, SimTK::Vec3 point1, 
        const PhysicalFrame& frame2, SimTK::Vec3 point2, 
        double linear_stiffness, double slack_length)
        : Blankevoort1991Ligament(name, frame1, point1, frame2, point2) {
    set_linear_stiffness(linear_stiffness);
    set_slack_length(slack_length);
}

Blankevoort1991Ligament::Blankevoort1991Ligament(
        std::string name, double linear_stiffness, double slack_length)
        : Blankevoort1991Ligament() {
    setName(name);
    set_linear_stiffness(linear_stiffness);
    set_slack_length(slack_length);
}

void Blankevoort1991Ligament::setNull()
{
    setAuthors("Colin Smith");
    setReferences(
    "Blankevoort, L. and Huiskes, R., (1991)."
    "Ligament-bone interaction in a three-dimensional model of the knee."
    "J Biomech Eng, 113(3), 263-269; "

    "Smith, C.R., Lenhart, R.L., Kaiser, J., Vignos, M.F. and Thelen, D.G.,"
    "(2016). Influence of ligament properties on tibiofemoral mechanics"
    " in walking. J Knee Surg, 29(02), 99-106.; "

    "Wismans, J.A.C., Veldpaus, F., Janssen, J., Huson, A. and Struben, P.,"
    "(1980). A three-dimensional mathematical model of the knee-joint. "
    "J Biomech, 13(8), 677-685."
    );
}

void Blankevoort1991Ligament::constructProperties() {
    constructProperty_GeometryPath(GeometryPath());
    constructProperty_linear_stiffness(1.0);
    constructProperty_transition_strain(0.06);
    constructProperty_damping_coefficient(0.003);
    constructProperty_slack_length(0.0);
}

void Blankevoort1991Ligament::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    // Check that properties are valid
    OPENSIM_THROW_IF_FRMOBJ(get_slack_length() < 0., InvalidPropertyValue,
            getProperty_slack_length().getName(),
            "Slack Length cannot be less than 0");

    OPENSIM_THROW_IF_FRMOBJ(get_linear_stiffness() < 0., InvalidPropertyValue,
            getProperty_linear_stiffness().getName(),
            "Linear Stiffness cannot be less than 0");

    OPENSIM_THROW_IF_FRMOBJ(get_damping_coefficient() < 0.,
            InvalidPropertyValue, getProperty_damping_coefficient().getName(),
            "Normalized Damping Coefficient cannot be less than 0");

    OPENSIM_THROW_IF_FRMOBJ(get_transition_strain() < 0., InvalidPropertyValue,
            getProperty_transition_strain().getName(),
            "Transistion Strain cannot be less than 0");

    // Set Default Ligament Color
    GeometryPath& path = upd_GeometryPath();
    path.setDefaultColor(SimTK::Vec3(0.1202, 0.7054, 0.1318));
}

void Blankevoort1991Ligament::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    this->_strainCV = addCacheVariable("strain", 0.0, SimTK::Stage::Position);
    this->_strainRateCV = addCacheVariable("strain_rate", 0.0, SimTK::Stage::Velocity);
    this->_forceSpringCV = addCacheVariable("force_spring", 0.0, SimTK::Stage::Position);
    this->_forceDampingCV = addCacheVariable("force_damping", 0.0, SimTK::Stage::Velocity);
    this->_forceTotalCV = addCacheVariable("force_total", 0.0, SimTK::Stage::Velocity);
}

//=============================================================================
// SCALING
//=============================================================================

void Blankevoort1991Ligament::extendPostScale(
    const SimTK::State& s, const ScaleSet& scaleSet) {
    Super::extendPostScale(s, scaleSet);

    GeometryPath& path = upd_GeometryPath();
    double slack_length = get_slack_length();
    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        set_slack_length(scaleFactor * slack_length);

        // Clear the pre-scale length that was stored in the GeometryPath.
        path.setPreScaleLength(s, 0.0);
    }
}

//=============================================================================
// GET
//=============================================================================

double Blankevoort1991Ligament::getLength(const SimTK::State& state) const {
    return get_GeometryPath().getLength(state);
}

double Blankevoort1991Ligament::getLengtheningSpeed(
        const SimTK::State& state) const {
    return get_GeometryPath().getLengtheningSpeed(state);
}

double Blankevoort1991Ligament::getStrain(const SimTK::State& state) const {
    if (isCacheVariableValid(state, _strainCV)) {
        return getCacheVariableValue(state, _strainCV);
    }

    double length = getLength(state);
    double strain = length/get_slack_length() - 1;
    setCacheVariableValue(state, _strainCV, strain);
    return strain;
}

double Blankevoort1991Ligament::getStrainRate(const SimTK::State& state) const {
    if (isCacheVariableValid(state, _strainRateCV)) {
        return getCacheVariableValue(state, _strainRateCV);
    }

    double lengthening_speed = getLengtheningSpeed(state);
    double strain_rate = lengthening_speed / get_slack_length();
    setCacheVariableValue(state, _strainRateCV, strain_rate);
    return strain_rate;
}

double Blankevoort1991Ligament::getSpringForce(const SimTK::State& state) const {
    if (isCacheVariableValid(state, _forceSpringCV)) {
        return getCacheVariableValue(state, _forceSpringCV);
    }

    double spring_force = calcSpringForce(state);
    setCacheVariableValue(state, _forceSpringCV, spring_force);
    return spring_force;
}

double Blankevoort1991Ligament::getDampingForce(const SimTK::State& state) const {
    if (isCacheVariableValid(state, _forceDampingCV)) {
        return getCacheVariableValue(state, _forceDampingCV);
    }

    double damping_force = calcDampingForce(state);
    setCacheVariableValue(state, _forceDampingCV, damping_force);
    return damping_force;
}

double Blankevoort1991Ligament::getTotalForce(const SimTK::State& state) const {
    if (isCacheVariableValid(state, _forceTotalCV)) {
        return getCacheVariableValue(state, _forceTotalCV);
    }

    double force_total = calcTotalForce(state);
    setCacheVariableValue(state, _forceTotalCV, force_total);
    return force_total;
}

double Blankevoort1991Ligament::getLinearStiffnessForcePerLength() const {
    return get_linear_stiffness() * get_slack_length();
};

double Blankevoort1991Ligament::getDampingCoefficientForceTimePerLength() const {
    return get_damping_coefficient() * get_slack_length();
};

double Blankevoort1991Ligament::getTransitionLength() const {
    return get_slack_length() * (get_transition_strain() + 1.0);
}

//=============================================================================
// SET
//=============================================================================

void Blankevoort1991Ligament::setSlackLengthFromReferenceStrain(
    double strain, const SimTK::State& reference_state) {    
    double reference_length = getLength(reference_state);
    double slack_length = reference_length / (1.0 + strain);
    
    set_slack_length(slack_length);
};

void Blankevoort1991Ligament::setSlackLengthFromReferenceForce(
        double force, const SimTK::State& reference_state) {

    OPENSIM_THROW_IF(force < 0.0, Exception,"force parameter cannot be less "
        "than 0. in setSlackLengthFromReferenceForce()")

    double strain = calcInverseForceStrainCurve(force);
    
    setSlackLengthFromReferenceStrain(strain, reference_state);
};

void Blankevoort1991Ligament::setLinearStiffnessForcePerLength(
        double linear_stiffness_length) {
    double linear_stiffness_strain =
            linear_stiffness_length / get_slack_length();
    set_linear_stiffness(linear_stiffness_strain);
};

void Blankevoort1991Ligament::setDampingCoefficientForceTimePerLength(
    double damping_coeff_length) {
    double damping_coeff_strain = damping_coeff_length / get_slack_length();
    set_damping_coefficient(damping_coeff_strain);
}
//=============================================================================
// COMPUTATION
//=============================================================================
double Blankevoort1991Ligament::calcInverseForceStrainCurve(
    double force) const{

    double transition_force = get_transition_strain() * 
        get_linear_stiffness() /2;

    double strain;

    if (force > 0 && force < transition_force){
        strain =
            sqrt(2 * get_transition_strain() * force / get_linear_stiffness());
    } else if (force >= transition_force) {
        strain = force / get_linear_stiffness() + get_transition_strain() / 2;
    } else { //force < 0.0
        strain = 0.0;
    }
    return strain;
}

double Blankevoort1991Ligament::calcSpringForce(
        const SimTK::State& state) const {

    double strain = getStrain(state);
    double k = get_linear_stiffness();
    double e_t = get_transition_strain();

    double force_spring;
    // slack region
    if (strain <= 0) {
        force_spring = 0.0;
    }
    // toe region F = 1/2 * k / e_t * e^2
    else if ((strain > 0) && (strain < e_t)) {        
        force_spring = 0.5 * k / e_t * SimTK::square(strain);
    }
    // linear region F = k * (e-e_t/2)
    else {//strain >= e_t 
        force_spring = k * (strain - e_t / 2); 
    }
    return force_spring;
}

double Blankevoort1991Ligament::calcDampingForce(const SimTK::State& s) const {
    double strain = getStrain(s);
    double strain_rate = getStrainRate(s);
    double force_damping = 0.0;

    if (strain > 0 && strain_rate > 0) {
        double damping_coeff = get_damping_coefficient();
        force_damping = damping_coeff*strain_rate;
    }
    else {
        return 0.0;
    }

    //Phase-out damping as strain goes to zero with smooth-step function
    SimTK::Function::Step step(0, 1, 0, 0.01);
    SimTK::Vector in_vec(1, strain);
    force_damping = force_damping * step.calcValue(in_vec);
    
    return force_damping;
}

double Blankevoort1991Ligament::calcTotalForce(const SimTK::State& s) const {
    double force_total = getDampingForce(s) + getSpringForce(s);

    // make sure the ligament is only acting in tension
    if (force_total < 0.0) {
        force_total = 0.0;
    }

    return force_total;
}

void Blankevoort1991Ligament::computeForce(const SimTK::State& s,
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                              SimTK::Vector& generalizedForces) const {
    if (get_appliesForce()) {
        // total force
        double force_total = getTotalForce(s);

        const GeometryPath &path = get_GeometryPath();

        path.addInEquivalentForces(
                s, force_total, bodyForces, generalizedForces);
    }
}

double Blankevoort1991Ligament::computePotentialEnergy(
        const SimTK::State& state) const {

    const double& strain = getStrain(state);
    const double& k = get_linear_stiffness();
    const double& e_t = get_transition_strain();
    const double& l_0 = get_slack_length();   

    //Toe Region
    if (strain > 0 && strain < e_t) {
        return 1.0 / 6.0 * k * l_0 / e_t * SimTK::cube(strain); 
    } 
    //Linear Region
    else if (strain >= e_t){
        return 1.0 / 6.0 * k * l_0 * SimTK::square(e_t) +
             1.0 / 2.0 * k * l_0 * strain * (strain - e_t); 
    } 
    //Slack Region
    else { //strain <= 0.0
        return 0.0;
    }    
}

double Blankevoort1991Ligament::computeMomentArm(
        const SimTK::State& s, Coordinate& aCoord) const {
    return get_GeometryPath().computeMomentArm(s, aCoord);
}

//=============================================================================
// Reporting
//=============================================================================
OpenSim::Array<std::string> Blankevoort1991Ligament::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(getName() + ".force_spring");
    labels.append(getName() + ".force_damping");
    labels.append(getName() + ".force_total");
    labels.append(getName() + ".length");
    labels.append(getName() + ".lengthening_speed");
    labels.append(getName() + ".strain");
    labels.append(getName() + ".strain_rate");
    return labels;
}

OpenSim::Array<double> Blankevoort1991Ligament::getRecordValues(
        const SimTK::State& s) const {
    OpenSim::Array<double> values(1);

    // Report values
    values.append(getSpringForce(s));
    values.append(getDampingForce(s));
    values.append(getTotalForce(s));
    values.append(getLength(s));
    values.append(getLengtheningSpeed(s));
    values.append(getStrain(s));
    values.append(getStrainRate(s));
    return values;
}
