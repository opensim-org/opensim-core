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
#include <OpenSim/Simulation/GeodesicWrapping/GeodesicParameters.h>

namespace OpenSim {

/**
 * A structure that holds parametric surface coordinates and other related
 * quantities needed for wrapping obstacle computations.
 */
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

/**
 * A structure that holds implicit surface coordinates and other related
 * quantities needed for wrapping obstacle computations.
 */
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

//=============================================================================
//                                  SURFACE
//=============================================================================
/**
 * A base class that represents a geometric surface used to define wrapping
 * obstacles.
 *
 * `Surface`s can be defined in parametric, implicit, or analytic form and can
 * support multiple forms simultaneously. This class provides methods for
 * computing surface data (gradients, Hessians, normal vectors, etc.), geodesic
 * torsion, and normal curvature. To enable these computations, `Surface`s must
 * implement the appropriate virtual methods for the form(s) they support.
 * Parametric and implicit surfaces must implement methods to evaluate the
 * surface equation and its derivatives, while analytic surfaces must implement
 * methods to compute geodesics and Gaussian curvature directly.
 *
 */
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
    /**
     * Return whether the parametric form of the surface is available.
     */
    bool getParametricFormAvailable() const;
    /**
     * Return whether the implicit form of the surface is available.
     */
    bool getImplicitFormAvailable() const;
    /**
     * Return whether the analytic form of the surface is available.
     */
    bool getAnalyticFormAvailable() const;

    /**
     * The parametric surface data containing parametric surface coordinates,
     * the surface point, partial derivatives of the surface point
     * with respect to the surface coordinates, the outward normal vector, the
     * first and second fundamental form coefficients, the Gaussian curvature,
     * and the determinant of the metric tensor.
     */
    void setParametricSurfaceData(ParametricSurfaceData data);
    /// @copydoc setParametricSurfaceData
    const ParametricSurfaceData& getParametricSurfaceData() const;
    /// @copydoc setParametricSurfaceData
    ParametricSurfaceData& updParametricSurfaceData();

    /**
     * The implicit surface data containing the surface point, the outward
     * normal vector, the surface constraint residual, the partial derivatives
     * of the constraint residual with respect to the surface point, the second
     * partial derivatives of the constraint residual with respect to the surface
     * point, the gradient of the constraint residual with respect to the surface
     * point, the Hessian of the constraint residual with respect to the surface
     * point, and the Gaussian curvature.
     */
    void setImplicitSurfaceData(ImplicitSurfaceData data);
    /// @copydoc setImplicitSurfaceData
    const ImplicitSurfaceData& getImplicitSurfaceData() const;
    /// @copydoc setImplicitSurfaceData
    ImplicitSurfaceData& updImplicitSurfaceData();

    // PARAMETRIC FORM
    /**
     * Evaluate the parametric surface equation and its derivatives at the
     * specified parametric coordinates.
     *
     * @param u  The first parametric coordinate.
     * @param v  The second parametric coordinate.
     *
     * @throws Exception if the parametric form is not available for this
     * Surface.
     */
    void evaluateSurface(const SimTK::Real& u,
            const SimTK::Real& v);

    /**
     * Compute the geodesic torsion at the specified parametric coordinate
     * derivatives.
     *
     * @param du  The derivative of the first parametric coordinate.
     * @param dv  The derivative of the second parametric coordinate.
     * @return    The geodesic torsion.
     *
     * @note Should be called after `evaluateSurface()` when the parametric form
     * is available.
     */
    SimTK::Real computeGeodesicTorsion(const SimTK::Real& du,
            const SimTK::Real& dv) const;

    /**
     * Compute the normal curvature at the specified parametric coordinate
     * derivatives.
     *
     * @param du  The derivative of the first parametric coordinate.
     * @param dv  The derivative of the second parametric coordinate.
     * @return    The normal curvature.
     *
     * @note Should be called after `evaluateSurface()` when the parametric form
     * is available.
     */
    SimTK::Real computeNormalCurvature(const SimTK::Real& du,
            const SimTK::Real& dv) const;

    // IMPLICIT FORM
    /**
     * Evaluate the implicit surface equation and its derivatives at the
     * specified surface point.
     *
     * @param p  The surface point.
     *
     * @throws Exception if the implicit form is not available for this Surface.
     */
    void evaluateSurface(const SimTK::Vec3& p);

    /**
     * Compute the geodesic torsion at the specified surface point derivative.
     *
     * @param dp  The derivative of the surface point.
     * @return    The geodesic torsion.
     *
     * @note Should be called after `evaluateSurface()` when the implicit form is
     * available.
     */
    SimTK::Real computeGeodesicTorsion(const SimTK::Vec3& dp) const;

    /**
     * Compute the normal curvature at the specified surface point derivative.
     *
     * @param dp  The derivative of the surface point.
     * @return    The normal curvature.
     *
     * @note Should be called after `evaluateSurface()` when the implicit form is
     * available.
     */
    SimTK::Real computeNormalCurvature(const SimTK::Vec3& dp) const;

    // ANALYTIC FORM
    /**
     * Compute a geodesic analytically.
     *
     * This method should be overridden by surfaces that support an analytic
     * form to compute the geodesic directly.
     *
     * @param parameters  The parametric parameters that define the geodesic.
     * @param steps       The number of steps to use in the computation.
     * @return            The computed geodesic.
     *
     * @throws Exception if the analytic form is not available for this Surface.
     */
    virtual Geodesic computeGeodesicAnalytically(
            const ParametricGeodesicParameters& parameters,
            int steps) const;

    /**
     * Evaluate the Gaussian curvature analytically.
     *
     * Surfaces that support an analytic form should override this method to
     * compute the Gaussian curvature directly. Concrete classes that override
     * this method must determine whether the surface supports parametric or
     * implicit forms and set the Gaussian curvature accordingly.
     */
    virtual void evaluateGaussianCurvatureAnalytically();

protected:
    // Concrete classes must define the available forms for a `Surface`.
    virtual void setAvailableForms() = 0;
    void setAnalyticFormAvailable(bool tf);
    void setParametricFormAvailable(bool tf);
    void setImplicitFormAvailable(bool tf);

    // Evaluate the parametric surface equation and its derivatives. Concrete
    // classes must implement this method if the parametric form is available.
    virtual void evaluateParametricEquation(const SimTK::Real& u,
            const SimTK::Real& v);
    void evaluateFirstFundamentalForm();
    void evaluateDeterminantOfMetricTensor();
    void evaluateSecondFundamentalForm();
    void evaluateGaussianCurvatureParametrically();

    // Evaluate the implicit surface equation and its derivatives. Concrete
    // classes must implement this method if the implicit form is available.
    virtual void evaluateImplicitEquation(const SimTK::Vec3& p);
    void evaluateSurfaceConstraintGradient();
    void evaluateSurfaceNormalFromGradient();
    void evaluateSurfaceConstraintHessian();
    void evaluateGaussianCurvatureImplicitly();

    // Solve the scalar Jacobi equation analytically. Concrete classes must
    // implement this method if the analytic form is available.
    virtual void solveScalarJacobiEquationAnalytically(
            const JacobiScalar& yStart,
            const SimTK::Real& arcLength,
            JacobiScalar& yEnd) const;

private:
    ImplicitSurfaceData m_implicitSurfaceData;
    ParametricSurfaceData m_parametricSurfaceData;
    bool m_analyticFormAvailable;
    bool m_parametricFormAvailable;
    bool m_implicitFormAvailable;
};

} // namespace OpenSim

#endif // OPENSIM_SURFACE_H
