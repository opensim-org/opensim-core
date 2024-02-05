#ifndef OPENSIM_SURFACE_H
#define OPENSIM_SURFACE_H
/* -------------------------------------------------------------------------- *
 *                           OpenSim: Surface.h                               *
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

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
#include <OpenSim/Simulation/GeodesicWrapping/Geodesic.h>

namespace OpenSim {

struct ParametricSurfaceData {
    /// Parametric surface coordinates.
    SimTK::Real u, v;
    /// Vector to point on surface, parametrized by u and v.
    SimTK::Vec3 x;
    /// Partial derivatives of x with respect to u and v.
    SimTK::Vec3 xu, xv, xuu, xuv, xvv;
    /// Outward normal vector from surface.
    SimTK::Vec3 N;
    /// First fundamental form coefficients.
    SimTK::Real E, F, G;
    /// Second fundamental form coefficients.
    SimTK::Real e, f, g;
    /// Gaussian curvature.
    SimTK::Real K;
    /// Determinant of the metric tensor.
    SimTK::Real detT;
};

struct ImplicitSurfaceData {
    /// Point on surface.
    SimTK::Vec3 p;
    /// Outward normal vector from surface.
    SimTK::Vec3 N;
    /// Surface constraint residual.
    SimTK::Real f;
    /// Partial derivatives of f with respect to x, y, and z.
    SimTK::Real fx, fy, fz;
    /// Second partial derivatives of f with respect to x, y, and z.
    SimTK::Real fxx, fxy, fxz, fyy, fyz, fzz;
    /// Gradient of f with respect to p.
    SimTK::Vec3 G;
    /// Hessian of f with respect to p.
    SimTK::Mat33 H;
    /// Gaussian curvature.
    SimTK::Real K;
};


class OSIMSIMULATION_API Surface : public ModelComponent {
    OpenSim_DECLARE_ABSTRACT_OBJECT(Surface, ModelComponent);
public:
//==============================================================================
// SOCKETS
//==============================================================================
    OpenSim_DECLARE_SOCKET(frame, PhysicalFrame,
            "The physical frame defining the position and orientation of "
            "this Surface.");

//==============================================================================
// METHODS
//==============================================================================

    // CONSTRUCTION AND DESTRUCTION
    Surface();
    ~Surface() noexcept override;

    Surface(const Surface&);
    Surface& operator=(const Surface&);

    Surface(Surface&& other);
    Surface& operator=(Surface&& other);

    // GET AND SET
    bool getParametricFormAvailable() const;
    bool getImplicitFormAvailable() const;
    bool getAnalyticFormAvailable() const;

    void setImplicitSurfaceData(ImplicitSurfaceData data);
    const ImplicitSurfaceData& getImplicitSurfaceData() const;
    ImplicitSurfaceData& updImplicitSurfaceData();

    void setParametricSurfaceData(ParametricSurfaceData data);
    const ParametricSurfaceData& getParametricSurfaceData() const;
    ParametricSurfaceData& updParametricSurfaceData();

    // PARAMETRIC FORM
    void evaluateSurface(const SimTK::Real& u,
            const SimTK::Real& v);
    SimTK::Real computeGeodesicTorsion(const SimTK::Real& du,
            const SimTK::Real& dv);
    SimTK::Real computeNormalCurvature(const SimTK::Real& du,
            const SimTK::Real& dv);

    // IMPLICIT FORM
    void evaluateSurface(const SimTK::Vec3& p);
    SimTK::Real computeGeodesicTorsion(const SimTK::Vec3& dp);
    SimTK::Real computeNormalCurvature(const SimTK::Vec3& dp);

    // ANALYTIC FORM
    virtual Geodesic computeGeodesicAnalytically(
            const ParametricGeodesicParameters& geodesicParameters,
            int numSteps = 10);
    virtual evaluateGaussianCurvatureAnalytically();

protected:
    virtual void setAvailableForms() = 0;
    void setAnalyticFormAvailable(bool tf);
    void setParametricFormAvailable(bool tf);
    void setImplicitFormAvailable(bool tf);

    // PARAMETRIC FORM
    virtual void evaluateParametricEquation(const SimTK::Real& u,
            const SimTK::Real& v);
    void evaluateFirstFundamentalForm();
    void evaluateDeterminantOfMetricTensor();
    void evaluateSecondFundamentalForm();
    void evaluateGaussianCurvatureParametrically();

    // IMPLICIT FORM
    virtual void evaluateImplicitEquation(const SimTK::Vec3& p);
    void evaluateSurfaceConstraintGradient();
    void evaluateSurfaceNormalFromGradient();
    void evaluateSurfaceConstraintHessian();
    void evaluateGaussianCurvatureImplicitly();

    // ANALYTIC FORM
    virtual void solveScalarJacobiEquationAnalytically(
            const JacobiScalar& yStart,
            const SimTK::Real& arcLength,
            JacobiScalar& yEnd);

private:
    ImplicitSurfaceData m_implicitSurfaceData;
    ParametricSurfaceData m_parametricSurfaceData;
    bool m_analyticFormAvailable;
    bool m_parametricFormAvailable;
    bool m_implicitFormAvailable;
};

} // namespace OpenSim

#endif // OPENSIM_SURFACE_H
