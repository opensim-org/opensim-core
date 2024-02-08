/* -------------------------------------------------------------------------- *
 *                           OpenSim: Surface.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Andreas Scholz                                 *
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

#include "Surface.h"
#include <OpenSim/Common/Assertion.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
Surface::Surface() : ModelComponent(),
                     m_implicitSurfaceData(),
                     m_parametricSurfaceData(),
                     m_analyticFormAvailable(false),
                     m_parametricFormAvailable(false),
                     m_implicitFormAvailable(false) {
}

Surface::~Surface() noexcept = default;

Surface::Surface(const Surface&) = default;

Surface& Surface::operator=(const Surface&) = default;

Surface::Surface(Surface&&) = default;

Surface& Surface::operator=(Surface&&) = default;

//=============================================================================
// GET AND SET
//=============================================================================
bool Surface::getAnalyticFormAvailable() const {
    return m_analyticFormAvailable;
}
void Surface::setAnalyticFormAvailable(bool tf) {
    m_analyticFormAvailable = tf;
}

bool Surface::getImplicitFormAvailable() const {
    return m_implicitFormAvailable;
}
void Surface::setImplicitFormAvailable(bool tf) {
    m_implicitFormAvailable = tf;
}

bool Surface::getParametricFormAvailable() const {
    return m_parametricFormAvailable;
}
void Surface::setParametricFormAvailable(bool tf) {
    m_parametricFormAvailable = tf;
}

void Surface::setImplicitSurfaceData(ImplicitSurfaceData data) {
    m_implicitSurfaceData = std::move(data);
}
const ImplicitSurfaceData& Surface::getImplicitSurfaceData() const {
    return m_implicitSurfaceData;
}
ImplicitSurfaceData& Surface::updImplicitSurfaceData() {
    return m_implicitSurfaceData;
}

void Surface::setParametricSurfaceData(ParametricSurfaceData data) {
    m_parametricSurfaceData = std::move(data);
}
const ParametricSurfaceData& Surface::getParametricSurfaceData() const {
    return m_parametricSurfaceData;
}
ParametricSurfaceData& Surface::updParametricSurfaceData() {
    return m_parametricSurfaceData;
}

//=============================================================================
// PARAMETRIC FORM
//=============================================================================
void Surface::evaluateParametricEquation(const SimTK::Real& u,
        const SimTK::Real& v) {
    OPENSIM_THROW_IF_FRMOBJ(getParametricFormAvailable(), Exception,
            "The parametric form was marked as available for Surface '{}', "
            "but the virtual method 'evaluateParametricEquation()' has not "
            "been implemented.", getName())

    log_warn("evaluateParametricEquation() is not available for Surface '{}'.",
             getName());
}

void Surface::evaluateSurface(const SimTK::Real& u,
        const SimTK::Real& v) {
    updParametricSurfaceData().u = u;
    updParametricSurfaceData().v = v;
    evaluateParametricEquation(u, v);

    evaluateFirstFundamentalForm();
    evaluateDeterminantOfMetricTensor();

    evaluateSecondFundamentalForm();
    evaluateGaussianCurvatureParametrically();
}

void Surface::evaluateFirstFundamentalForm() {
    auto& data = updParametricSurfaceData();
    data.E = ~data.xu * data.xu;
    data.F = ~data.xu * data.xv;
    data.G = ~data.xv * data.xv;
}

void Surface::evaluateDeterminantOfMetricTensor() {
    auto& data = updParametricSurfaceData();
    data.detT = data.E * data.G - data.F * data.F;
}

void Surface::evaluateSecondFundamentalForm() {
    auto& data = updParametricSurfaceData();
    data.e = ~data.xuu * data.N;
    data.f = ~data.xuv * data.N;
    data.g = ~data.xvv * data.N;
}

void Surface::evaluateGaussianCurvatureParametrically() {
    auto& data = updParametricSurfaceData();
    data.K = (data.e * data.g - data.f * data.f) / data.detT;
}

SimTK::Real Surface::computeGeodesicTorsion(const SimTK::Real& du,
        const SimTK::Real& dv) const {
    OPENSIM_ASSERT(getParametricFormAvailable())
    const auto& data = getParametricSurfaceData();
    const auto& E = data.E;
    const auto& F = data.F;
    const auto& G = data.G;
    const auto& e = data.e;
    const auto& f = data.f;
    const auto& g = data.g;
    const auto& detT = data.detT;

    return ((F*e - E*f)*du*du + (G*e - E*g)*du*dv + (G*f - F*g)*dv*dv) /
           (sqrt(detT) * (E*du*du + 2.0*F*du*dv + G*dv*dv));
}

SimTK::Real Surface::computeNormalCurvature(const SimTK::Real& du,
        const SimTK::Real& dv) const {
    OPENSIM_ASSERT(getParametricFormAvailable())
    const auto& data = getParametricSurfaceData();
    const auto& E = data.E;
    const auto& F = data.F;
    const auto& G = data.G;
    const auto& e = data.e;
    const auto& f = data.f;
    const auto& g = data.g;

    return (e*du*du + 2.0*f*du*dv + g*dv*dv) /
           (E*du*du + 2.0*F*du*dv + G*dv*dv);
}

//=============================================================================
// IMPLICIT FORM
//=============================================================================
void Surface::evaluateImplicitEquation(const SimTK::Vec3& p) {
    OPENSIM_THROW_IF_FRMOBJ(getImplicitFormAvailable(), Exception,
            "The implicit form was marked as available for Surface '{}', "
            "but the virtual method 'evaluateImplicitEquation()' has not "
            "been implemented.", getName())

    log_warn("evaluateImplicitEquation() is not available for Surface '{}'.",
            getName());
}
void Surface::evaluateSurface(const SimTK::Vec3& p) {
    updImplicitSurfaceData().p = p;
    evaluateImplicitEquation(p);

    evaluateSurfaceConstraintGradient();
    evaluateSurfaceNormalFromGradient();
    evaluateSurfaceConstraintHessian();
    evaluateGaussianCurvatureImplicitly();
}

void Surface::evaluateSurfaceConstraintGradient() {
    auto& data = updImplicitSurfaceData();
    data.G = SimTK::Vec3(data.fx, data.fy, data.fz);
}

void Surface::evaluateSurfaceNormalFromGradient() {
    auto& data = updImplicitSurfaceData();
    data.N = data.G / data.G.norm();
}

void Surface::evaluateSurfaceConstraintHessian() {
    auto& data = updImplicitSurfaceData();
    data.H = SimTK::Mat33(data.fxx, data.fxy, data.fxz,
                          data.fxy, data.fyy, data.fyz,
                          data.fxz, data.fyz, data.fzz);
}

void Surface::evaluateGaussianCurvatureImplicitly() {
    auto& data = updImplicitSurfaceData();
    const auto& fxx = data.fxx;
    const auto& fyy = data.fyy;
    const auto& fzz = data.fzz;
    const auto& fxy = data.fxy;
    const auto& fxz = data.fxz;
    const auto& fyz = data.fyz;

    SimTK::Mat33 A(fyy*fzz-fyz*fyz, fyz*fxz-fxy*fzz, fxy*fyz-fyy*fxz,
                   fxz*fyz-fxy*fzz, fxx*fzz-fxz*fxz, fxy*fxz-fxx*fyz,
                   fxy*fyz-fxz*fyy, fxy*fxz-fxx*fyz, fxx*fyy-fxy*fxy);

    SimTK::Real normG = data.G.norm();

    data.K = (~data.G * (A * data.G)) / (normG * normG * normG * normG);
}

SimTK::Real Surface::computeGeodesicTorsion(const SimTK::Vec3& dp) const {
    OPENSIM_ASSERT(getImplicitFormAvailable())
    const auto& data = getImplicitSurfaceData();
    return ~dp * (((data.H*dp - data.N*(~data.N*(data.H*dp))) / data.G.norm())
                  % data.N);
}

SimTK::Real Surface::computeNormalCurvature(const SimTK::Vec3& dp) const {
    OPENSIM_ASSERT(getImplicitFormAvailable())
    const auto& data = getImplicitSurfaceData();
    return -~dp * (data.H * dp) / data.G.norm();
}

//=============================================================================
// ANALYTIC FORM
//=============================================================================
Geodesic Surface::computeGeodesicAnalytically(
        const ParametricGeodesicParameters& geodesicParameters,
        int numSteps) const {
    OPENSIM_THROW_IF_FRMOBJ(getAnalyticFormAvailable(), Exception,
            "The analytic form was marked as available for Surface '{}', "
            "but the virtual method 'computeGeodesicAnalytically()' has not "
            "been implemented.", getName())

    log_warn("computeGeodesicAnalytically() is not available for Surface '{}'.",
            getName());

    return {};
}

void Surface::evaluateGaussianCurvatureAnalytically() {
    OPENSIM_THROW_IF_FRMOBJ(getAnalyticFormAvailable(), Exception,
            "The analytic form was marked as available for Surface '{}', "
            "but the virtual method 'evaluateGaussianCurvatureAnalytically()' "
            "has not been implemented.", getName())

    log_warn("evaluateGaussianCurvatureAnalytically() is not available for "
            "Surface '{}'.", getName());
}

void Surface::solveScalarJacobiEquationAnalytically(const JacobiScalar& yStart,
        const SimTK::Real& arcLength, JacobiScalar& yEnd) const {
    OPENSIM_THROW_IF_FRMOBJ(getAnalyticFormAvailable(), Exception,
            "The analytic form was marked as available for Surface '{}', "
            "but the virtual method 'solveScalarJacobiEquationAnalytically()' "
            "has not been implemented.", getName())

    log_warn("solveScalarJacobiEquationAnalytically() is not available for "
            "Surface '{}'.", getName());
}
