/* -------------------------------------------------------------------------- *
 *                OpenSim:  MuscleFixedWidthPennationModel.cpp                *
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
#include "MuscleFixedWidthPennationModel.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

void MuscleFixedWidthPennationModel::setNull()
{
    setAuthors("Matthew Millard");
    m_parallelogramHeight           = SimTK::NaN;
    m_maximumSinPennation           = SimTK::NaN;
    m_minimumFiberLength            = SimTK::NaN;
    m_minimumFiberLengthAlongTendon = SimTK::NaN;
}

void MuscleFixedWidthPennationModel::constructProperties()
{
    constructProperty_optimal_fiber_length(1.0);
    constructProperty_optimal_pennation_angle(0.0);
    constructProperty_maximum_pennation_angle(acos(0.1));
}

MuscleFixedWidthPennationModel::MuscleFixedWidthPennationModel()
{
    setNull();
    constructProperties();
    ensureModelUpToDate();
}

MuscleFixedWidthPennationModel::
MuscleFixedWidthPennationModel(double optimalFiberLength,
                               double optimalPennationAngle,
                               double maximumPennationAngle)
{
    SimTK_ERRCHK_ALWAYS(optimalFiberLength > 0,
        "MuscleFixedWidthPennationModel::MuscleFixedWidthPennationModel",
        "Optimal fiber length must be greater than zero.");

    SimTK_ERRCHK_ALWAYS(optimalPennationAngle >= 0
                        && optimalPennationAngle < SimTK::Pi/2,
        "MuscleFixedWidthPennationModel::MuscleFixedWidthPennationModel",
        "Optimal pennation angle must be in the range [0, Pi/2) radians.");

    SimTK_ERRCHK_ALWAYS(maximumPennationAngle >= 0
                        && maximumPennationAngle <= SimTK::Pi/2,
        "MuscleFixedWidthPennationModel::MuscleFixedWidthPennationModel",
        "Maximum pennation angle must be in the range [0, Pi/2] radians.");

    setNull();
    constructProperties();

    setOptimalFiberLength(optimalFiberLength);
    setOptimalPennationAngle(optimalPennationAngle);
    setMaximumPennationAngle(maximumPennationAngle);

    ensureModelUpToDate();
}

void MuscleFixedWidthPennationModel::buildModel()
{
    // Compute quantities that are used often.
    m_parallelogramHeight = getOptimalFiberLength()
                            * sin(getOptimalPennationAngle());
    m_maximumSinPennation = sin(getMaximumPennationAngle());

    // Compute the minimum fiber length:
    // - pennated: the length of the fiber when at its maximum pennation angle
    // - straight: 1% of the optimal fiber length
    if(getMaximumPennationAngle() > SimTK::SignificantReal) {
        m_minimumFiberLength = m_parallelogramHeight / m_maximumSinPennation;
    } else {
        m_minimumFiberLength = getOptimalFiberLength()*0.01;
    }
    m_minimumFiberLengthAlongTendon = m_minimumFiberLength
                                      * cos(getMaximumPennationAngle());

    // Mark model as being up-to-date with its properties
    setObjectIsUpToDateWithProperties();
}

void MuscleFixedWidthPennationModel::ensureModelUpToDate()
{
    ensurePropertiesSet();
    if(!isObjectUpToDateWithProperties()) {
        buildModel();
    }
}

void MuscleFixedWidthPennationModel::ensurePropertiesSet() const
{
    SimTK_ERRCHK1_ALWAYS(!isNaN(getOptimalFiberLength())
                         && !isNaN(getOptimalPennationAngle())
                         && !isNaN(getMaximumPennationAngle()),
                         "MuscleFixedWidthPennationModel",
                         "%s: Properties have not been set.",
                         getName().c_str());
}

double MuscleFixedWidthPennationModel::getParallelogramHeight() const
{   return m_parallelogramHeight; }
double MuscleFixedWidthPennationModel::getOptimalFiberLength() const
{   return get_optimal_fiber_length(); }
double MuscleFixedWidthPennationModel::getMinimumFiberLength() const
{   return m_minimumFiberLength; }
double MuscleFixedWidthPennationModel::getMinimumFiberLengthAlongTendon() const
{   return m_minimumFiberLengthAlongTendon; }
double MuscleFixedWidthPennationModel::getOptimalPennationAngle() const
{   return get_optimal_pennation_angle(); }
double MuscleFixedWidthPennationModel::getMaximumPennationAngle() const
{   return get_maximum_pennation_angle(); }

bool MuscleFixedWidthPennationModel::
setOptimalFiberLength(double aOptimalFiberLength)
{
    if(aOptimalFiberLength > SimTK::SignificantReal) {
        set_optimal_fiber_length(aOptimalFiberLength);
        ensureModelUpToDate();
        return true;
    }
    return false;
}

bool MuscleFixedWidthPennationModel::
setOptimalPennationAngle(double aOptimalPennationAngle)
{
    if(aOptimalPennationAngle >= 0 && aOptimalPennationAngle < SimTK::Pi/2) {
        set_optimal_pennation_angle(aOptimalPennationAngle);
        ensureModelUpToDate();
        return true;
    }
    return false;
}

bool MuscleFixedWidthPennationModel::
setMaximumPennationAngle(double aMaximumPennationAngle)
{
    if(aMaximumPennationAngle >= 0 && aMaximumPennationAngle < SimTK::Pi/2) {
        set_maximum_pennation_angle(aMaximumPennationAngle);
        ensureModelUpToDate();
        return true;
    }
    return false;
}

double MuscleFixedWidthPennationModel::
clampFiberLength(double fiberLength) const
{
    return max(m_minimumFiberLength, fiberLength);
}

//==============================================================================
// Position-level kinematics
//==============================================================================
double MuscleFixedWidthPennationModel::
calcPennationAngle(double fiberLength) const
{
    double phi = 0;
    double optimalPennationAngle = getOptimalPennationAngle();

    // This computation is only worth performing on pennated muscles.
    if(optimalPennationAngle > SimTK::Eps) {
        if(fiberLength > m_minimumFiberLength) {
            double sin_phi = m_parallelogramHeight/fiberLength;
            phi = (sin_phi < m_maximumSinPennation) ?
                  asin(sin_phi) : getMaximumPennationAngle();
        } else {
            phi = getMaximumPennationAngle();
        }
    }
    return phi;
}

double MuscleFixedWidthPennationModel::calcTendonLength(
    double cosPennationAngle, double fiberLength, double muscleLength) const
{
    return muscleLength - fiberLength*cosPennationAngle;
}

double MuscleFixedWidthPennationModel::calcFiberLengthAlongTendon(
    double fiberLength, double cosPennationAngle) const
{
    return fiberLength*cosPennationAngle;
}

//==============================================================================
// Velocity-level kinematics
//==============================================================================
double MuscleFixedWidthPennationModel::calcPennationAngularVelocity(
    double tanPennationAngle, double fiberLength, double fiberVelocity) const
{
    double dphi = 0;
    double optimalPennationAngle = getOptimalPennationAngle();

    if(optimalPennationAngle > SimTK::Eps) {
        SimTK_ERRCHK_ALWAYS(fiberLength > 0,
            "MuscleFixedWidthPennationModel::calcPennationAngularVelocity",
            "Fiber length cannot be zero.");
        dphi = -(fiberVelocity/fiberLength) * tanPennationAngle;
    }
    return dphi;
}

double MuscleFixedWidthPennationModel::
calcTendonVelocity(double cosPennationAngle,
                   double sinPennationAngle,
                   double pennationAngularVelocity,
                   double fiberLength,
                   double fiberVelocity,
                   double muscleVelocity) const
{
    return muscleVelocity - fiberVelocity*cosPennationAngle
        + fiberLength*sinPennationAngle*pennationAngularVelocity;
}

double MuscleFixedWidthPennationModel::
calcFiberVelocityAlongTendon(double fiberLength,
                             double fiberVelocity,
                             double sinPennationAngle,
                             double cosPennationAngle,
                             double pennationAngularVelocity) const
{
    return fiberVelocity*cosPennationAngle
        - fiberLength*sinPennationAngle*pennationAngularVelocity;
}

//==============================================================================
// Acceleration-level kinematics
//==============================================================================
double MuscleFixedWidthPennationModel::
calcPennationAngularAcceleration(double fiberLength,
                                 double fiberVelocity,
                                 double fiberAcceleration,
                                 double sinPennationAngle,
                                 double cosPennationAngle,
                                 double pennationAngularVelocity) const
{
    SimTK_ERRCHK_ALWAYS(fiberLength > 0,
        "MuscleFixedWidthPennationModel::calcPennationAngularAcceleration",
        "Fiber length cannot be zero.");
    SimTK_ERRCHK_ALWAYS(cosPennationAngle > 0,
        "MuscleFixedWidthPennationModel::calcPennationAngularAcceleration",
        "cosPennationAngle cannot be zero.");

    double numer = sinPennationAngle*cosPennationAngle*
                   (fiberVelocity*fiberVelocity - fiberLength*fiberAcceleration)
                   - fiberLength*fiberVelocity*pennationAngularVelocity;
    return numer / (fiberLength*fiberLength
                    * cosPennationAngle*cosPennationAngle);
}

double MuscleFixedWidthPennationModel::
calcFiberAccelerationAlongTendon(double fiberLength,
                                 double fiberVelocity,
                                 double fiberAcceleration,
                                 double sinPennationAngle,
                                 double cosPennationAngle,
                                 double pennationAngularVelocity,
                                 double pennationAngularAcceleration) const
{
    return fiberAcceleration*cosPennationAngle
           - fiberLength*sinPennationAngle*pennationAngularAcceleration
           - pennationAngularVelocity*
             (2.0*fiberVelocity*sinPennationAngle
              + fiberLength*cosPennationAngle*pennationAngularVelocity);
}

//==============================================================================
// Partial derivatives with respect to fiber length
//==============================================================================
double MuscleFixedWidthPennationModel::
calc_DFiberLengthAlongTendon_DfiberLength(double fiberLength,
                                          double sinPennationAngle,
                                          double cosPennationAngle,
                                          double DpennationAngle_DfiberLength)
                                          const
{
    // d/dlce (lce*cos(phi)) = cos(phi) - lce*sin(phi)*dphi_dlce
    return cosPennationAngle
           - fiberLength*sinPennationAngle*DpennationAngle_DfiberLength;
}

double MuscleFixedWidthPennationModel::
calc_DPennationAngularVelocity_DfiberLength(double fiberLength,
                                            double fiberVelocity,
                                            double sinPennationAngle,
                                            double cosPennationAngle,
                                            double pennationAngularVelocity,
                                            double DpennationAngle_DfiberLength)
                                            const
{
    SimTK_ERRCHK_ALWAYS(cosPennationAngle > SimTK::Eps,
        "MuscleFixedWidthPennationModel::"
        "calc_DFiberVelocityAlongTendon_DfiberLength",
        "Pennation angle must be less than 90 degrees");
    SimTK_ERRCHK_ALWAYS(fiberLength > SimTK::Eps,
        "MuscleFixedWidthPennationModel::"
        "calc_DFiberVelocityAlongTendon_DfiberLength",
        "Fiber length is close to 0.");

    // dphidt = -(fiberVelocity/fiberLength)*tanPennationAngle
    double tanPhi = sinPennationAngle/cosPennationAngle;
    double DtanPhi_Dlce = (1+tanPhi*tanPhi)*DpennationAngle_DfiberLength;
    return (fiberVelocity/(fiberLength*fiberLength))*tanPhi
           - (fiberVelocity/fiberLength)*DtanPhi_Dlce;
}

double MuscleFixedWidthPennationModel::
calc_DFiberVelocityAlongTendon_DfiberLength(
                                double fiberLength,
                                double fiberVelocity,
                                double sinPennationAngle,
                                double cosPennationAngle,
                                double pennationAngularVelocity,
                                double DpennationAngle_DfiberLength,
                                double DpennationAngularVelocity_DfiberLength)
                                const
{
    // dlceAT = dlce*cos(phi) - lce*sin(phi)*dphidt
    // Now take the partial derivative of dlceAT with respect to lce
    return fiberVelocity*(-sinPennationAngle*DpennationAngle_DfiberLength)
           - sinPennationAngle*pennationAngularVelocity
           - fiberLength*cosPennationAngle
             *DpennationAngle_DfiberLength*pennationAngularVelocity
           - fiberLength*sinPennationAngle
             *DpennationAngularVelocity_DfiberLength;
}

double MuscleFixedWidthPennationModel::
calc_DPennationAngle_DfiberLength(double fiberLength) const
{
    SimTK_ERRCHK_ALWAYS(fiberLength > m_parallelogramHeight,
        "MuscleFixedWidthPennationModel::calc_DPennationAngle_DfiberLength",
        "Fiber length is below the lower bound for this muscle.");

    // phi = asin( parallelogramHeight/lce)
    // d_phi/d_lce = d/dlce (asin(parallelogramHeight/lce))
    double h_over_l = m_parallelogramHeight / fiberLength;
    return (-h_over_l/fiberLength) / sqrt(1.0 - h_over_l*h_over_l);
}

double MuscleFixedWidthPennationModel::
calc_DTendonLength_DfiberLength(double fiberLength,
                                double sinPennationAngle,
                                double cosPennationAngle,
                                double DpennationAngle_DfiberLength) const
{
    SimTK_ERRCHK_ALWAYS(fiberLength > m_parallelogramHeight,
        "MuscleFixedWidthPennationModel::calc_DTendonLength_DfiberLength",
        "Fiber length is below the lower bound for this muscle.");

    // dtl_dlce = DmuscleLength_DfiberLength - cosPennationAngle
    //            + fiberLength*sinPennationAngle*DpennationAngle_DfiberLength
    return fiberLength*sinPennationAngle*DpennationAngle_DfiberLength
           - cosPennationAngle;
}

//==============================================================================
// Kinematic fiber pose equations (useful during initialization)
//==============================================================================
double MuscleFixedWidthPennationModel::calcFiberLength(double muscleLength,
                                                     double tendonLength) const
{
    double fiberLengthAT = muscleLength-tendonLength;
    double fiberLength   = 0.0;

    if(fiberLengthAT >= getMinimumFiberLengthAlongTendon()) {
        fiberLength = sqrt(m_parallelogramHeight*m_parallelogramHeight
                           + fiberLengthAT*fiberLengthAT);
    } else {
        fiberLength = getMinimumFiberLength();
    }
    return fiberLength;
}

double MuscleFixedWidthPennationModel::
calcFiberVelocity(double cosPennationAngle,
                  double muscleVelocity,
                  double tendonVelocity) const
{
    // Differentiate l^M cos(phi) + l^T = l^MT ....(1)
    // Differentiate h = l^M sin(phi) .............(2)
    // Solve (2) for phidot, substitute into (1), and solve for v^M.
    return (muscleVelocity-tendonVelocity)*cosPennationAngle;
}
