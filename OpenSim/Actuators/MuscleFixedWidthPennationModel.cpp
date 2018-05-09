/* -------------------------------------------------------------------------- *
 *                OpenSim:  MuscleFixedWidthPennationModel.cpp                *
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
    constructProperty_pennation_angle_at_optimal(0.0);
    constructProperty_maximum_pennation_angle(acos(0.1));
}

MuscleFixedWidthPennationModel::MuscleFixedWidthPennationModel()
{
    setNull();
    constructProperties();
}

MuscleFixedWidthPennationModel::
MuscleFixedWidthPennationModel(double optimalFiberLength,
                               double optimalPennationAngle,
                               double maximumPennationAngle)
{
    setNull();
    constructProperties();

    set_optimal_fiber_length(optimalFiberLength);
    set_pennation_angle_at_optimal(optimalPennationAngle);
    set_maximum_pennation_angle(maximumPennationAngle);
}

double MuscleFixedWidthPennationModel::getParallelogramHeight() const
{   return m_parallelogramHeight; }
double MuscleFixedWidthPennationModel::getMinimumFiberLength() const
{   return m_minimumFiberLength; }
double MuscleFixedWidthPennationModel::getMinimumFiberLengthAlongTendon() const
{   return m_minimumFiberLengthAlongTendon; }

double MuscleFixedWidthPennationModel::
clampFiberLength(double fiberLength) const
{
    return max(m_minimumFiberLength, fiberLength);
}

//==============================================================================
// COMPONENT INTERFACE
//==============================================================================
void MuscleFixedWidthPennationModel::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    std::string errorLocation = getName() +
        " MuscleFixedWidthPennationModel::extendFinalizeFromProperties";

    // Ensure property values are within appropriate ranges.
    OPENSIM_THROW_IF_FRMOBJ(
        get_optimal_fiber_length() <= 0,
        InvalidPropertyValue,
        getProperty_optimal_fiber_length().getName(),
        "Optimal fiber length must be greater than zero");
    OPENSIM_THROW_IF_FRMOBJ(
        get_pennation_angle_at_optimal() < 0 ||
        get_pennation_angle_at_optimal() > SimTK::Pi/2.0-SimTK::SignificantReal,
        InvalidPropertyValue,
        getProperty_pennation_angle_at_optimal().getName(),
        "Pennation angle at optimal fiber length must be in the range [0, Pi/2)");
    OPENSIM_THROW_IF_FRMOBJ(
        get_maximum_pennation_angle() < 0 ||
        get_maximum_pennation_angle() > SimTK::Pi/2.0,
        InvalidPropertyValue,
        getProperty_maximum_pennation_angle().getName(),
        "Maximum pennation angle must be in the range [0, Pi/2]");

    // Compute quantities that are used often.
    m_parallelogramHeight = get_optimal_fiber_length()
                            * sin(get_pennation_angle_at_optimal());
    m_maximumSinPennation = sin(get_maximum_pennation_angle());

    // Compute the minimum fiber length:
    // - pennated: the length of the fiber when at its maximum pennation angle
    // - straight: 1% of the optimal fiber length
    if(get_maximum_pennation_angle() > SimTK::SignificantReal) {
        m_minimumFiberLength = m_parallelogramHeight / m_maximumSinPennation;
    } else {
        m_minimumFiberLength = get_optimal_fiber_length()*0.01;
    }
    m_minimumFiberLengthAlongTendon = m_minimumFiberLength
                                      * cos(get_maximum_pennation_angle());
}

//==============================================================================
// Position-level kinematics
//==============================================================================
double MuscleFixedWidthPennationModel::
calcPennationAngle(double fiberLength) const
{
    double phi = 0;
    double optimalPennationAngle = get_pennation_angle_at_optimal();

    // This computation is only worth performing on pennated muscles.
    if(optimalPennationAngle > SimTK::Eps) {
        if(fiberLength > m_minimumFiberLength) {
            double sin_phi = m_parallelogramHeight/fiberLength;
            phi = (sin_phi < m_maximumSinPennation) ?
                  asin(sin_phi) : get_maximum_pennation_angle();
        } else {
            phi = get_maximum_pennation_angle();
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
    double optimalPennationAngle = get_pennation_angle_at_optimal();

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
