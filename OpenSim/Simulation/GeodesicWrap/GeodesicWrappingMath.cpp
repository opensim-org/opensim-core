#include "GeodesicCurveState.h"
#include "ImplicitSurfaceObject.h"
#include "ImplicitSurfaceParams.h"
#include "ImplicitSurfaceState.h"
#include "ImplicitSphere.h"
#include "GeodesicWrapObject.h"
#include <iostream>
#include <memory>

namespace OpenSim
{

//==============================================================================
//                      PRINTING
//==============================================================================

std::ostream& operator<<(std::ostream& os, const ImplicitSurfaceState& q)
{
    return os << "ImplicitSurfaceState{"
        << "position{"
        << q.postion[0] << ", "
        << q.postion[1] << ", "
        << q.postion[2] << "},"
        << "velocity{"
        << q.velocity[0] << ", "
        << q.velocity[1] << ", "
        << q.velocity[2] << "}";
}

std::ostream& operator<<(std::ostream& os, const JacobiFieldScalar& x)
{
    return os << "JacobiFieldScalar{"
        << "value: "
        << x.value << ", "
        << "derivative: "
        << x.derivative << "}";
}

std::ostream& operator<<(std::ostream& os, const ImplicitGeodesicState& x)
{
    const ImplicitSurfaceState& q = x;
    return os << "ImplicitGeodesicState{"
        << "q: " << q << ", "
        << "a: " << x.a << ", "
        << "r: " << x.r << "}";
}

/// Calculates adjoint of matrix.
/// Assumes matrix is symmetric.
SimTK::Mat33 calcAdjoint(const ImplicitSurfaceParamsImpl::Hessian& mat)
{
    double fxx = mat(0, 0);
    double fyy = mat(1, 1);
    double fzz = mat(2, 2);

    double fxy = mat(0, 1);
    double fxz = mat(0, 2);
    double fyz = mat(1, 2);

    // This is now correct (tested)
    std::array<double, 9> elements = {
        fyy * fzz - fyz * fyz,
        fyz * fxz - fxy * fzz,
        fxy * fyz - fyy * fxz,
        fxz * fyz - fxy * fzz,
        fxx * fzz - fxz * fxz,
        fxy * fxz - fxx * fyz,
        fxy * fyz - fxz * fyy,
        fxy * fxz - fxx * fyz,
        fxx * fyy - fxy * fxy};
    SimTK::Mat33 adj;
    size_t i = 0;
    for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 3; ++c) {
            adj(r, c) = elements[++i];
        }
    }
    return adj;
}

//==============================================================================
//                      CURVATURES
//==============================================================================

double calcNormalCurvature(
    const ImplicitSurfaceParams& s,
    const ImplicitSurfaceState& q)
{
    const SimTK::Vec3& v  = q.velocity;
    const SimTK::Vec3 g   = s.calcSurfaceConstraintGradient(q);
    const SimTK::Vec3 h_v = s.calcSurfaceConstraintHessian(q) * v;
    return SimTK::dot(v, h_v) / g.norm();
}

double calcTangentialCurvature(
    const ImplicitSurfaceParams& s,
    const ImplicitSurfaceState& q)
{
    // ALWAYS
    return 0.;
}

double calcGeodesicTorsion(
    const ImplicitSurfaceParams& s,
    const ImplicitSurfaceState& q)
{
    const SimTK::Vec3& v  = q.velocity;
    const SimTK::Vec3 g   = s.calcSurfaceConstraintGradient(q);
    const SimTK::Vec3 h_v = s.calcSurfaceConstraintHessian(q) * v;
    const SimTK::Vec3 jxv = g.cross(v);
    return h_v.dot(jxv) / g.dot(g);
}

// TODO use normalized vector.
SimTK::Vec3 calcSurfaceNormal(
    const ImplicitSurfaceParams& s,
    const ImplicitSurfaceState& q)
{
    const SimTK::Vec3 gradient = s.calcSurfaceConstraintGradient(q);
    return gradient / gradient.norm();
}

SimTK::Vec3 calcAcceleration(
    const ImplicitSurfaceParams& s,
    const ImplicitSurfaceState& q)
{
    // TODO Writing it out saves a root, but optimizers are smart.
    return -calcNormalCurvature(s, q) * calcSurfaceNormal(s, q);
}

double calcGaussianCurvature(
    const ImplicitSurfaceParams& s,
    const ImplicitSurfaceState& q)
{
    SimTK::Vec3 g       = s.calcSurfaceConstraintGradient(q);
    double gDotg = g.dot(g);
    SimTK::Mat33 adj   = calcAdjoint(s.calcSurfaceConstraintHessian(q));

    return (g.dot(adj * g)) / (gDotg * gDotg);
}

DarbouxFrame calcDarbouxFrame(
        const ImplicitSurfaceParams& s,
    const ImplicitSurfaceState& q)
{
    return {q.velocity, calcSurfaceNormal(s, q)};
}

DarbouxFrame::DarbouxFrame(
        SimTK::Vec3 velocity,
        SimTK::Vec3 surfaceNormal)
    : t(std::move(velocity))
    , n(-t.cross(t.cross(std::move(surfaceNormal))))
    , b(t.cross(n)) {
        t = t/t.norm();
        n = n/n.norm();
        b = b/b.norm();
    }

//==============================================================================
//                      IMPLICIT SURFACE SHOOTER HELPERS
//==============================================================================

template <typename T, typename D = T>
void RungeKutta4(
    T& y,
    double& t,
    double dt,
    std::function<D(const T&, double)> f)
{
    D k0, k1, k2, k3;

    {
        const double h  = 0.;
        const double tk = t + h;
        const T& yk     = y;
        k0              = f(yk, tk);
    }

    {
        const double h  = dt / 2.;
        const double tk = t + h;
        T yk            = y;
        y += k0 * h;
        k1 = f(yk, tk);
    }

    {
        const double h  = dt / 2.;
        const double tk = t + h;
        T yk            = y;
        y += k1 * h;
        k2 = f(yk, tk);
    }

    {
        const double h  = dt;
        const double tk = t + h;
        T yk            = y;
        y += k2 * h;
        k3 = f(yk, tk);
    }

    y += dt / 6. * (k0 + 2. * k1 + 2. * k2 + k3);
    t += dt;
}

void RungeKutta4(
    const ImplicitSurfaceParams& s,
    ImplicitSurfaceState& q,
    double& t,
    double dt)
{
    RungeKutta4<ImplicitSurfaceState, Vec10>(
        q,
        t,
        dt,
        [&](const ImplicitSurfaceState& qk, double) -> Vec10 {
            return qk.calcDerivative(
                calcAcceleration(s, qk));});
}

void RungeKutta4(
    const ImplicitSurfaceParams& s,
    ImplicitGeodesicState& q,
    double& t,
    double dt)
{
    RungeKutta4<ImplicitGeodesicState, Vec10>(
        q,
        t,
        dt,
        [&](const ImplicitGeodesicState& qk, double) -> Vec10 {
            return qk.calcDerivative(
                calcAcceleration(s, qk),
                calcGaussianCurvature(s, qk));
        });
}

// Compute the constraint that projects the ImplicitSurface's State to the
// ImplicitSurface. (See 7.1, 7.20 and 7.21)
//
// Returns three constraints stacked:
// 1. implicit surface constraint,
// 2. velocity direction lies in the surface normal plane,
// 3. velocity has norm one.
SimTK::Vec3 calcSurfaceProjectionConstraint(
    const ImplicitSurfaceParams& s,
    const ImplicitSurfaceState& q)
{
    const SimTK::Vec3& v  = q.velocity;
    const SimTK::Vec3& p  = q.postion;
    const double c = s.calcSurfaceConstraint(q);
    const SimTK::Vec3 n   = calcSurfaceNormal(s, q);
    return {c, v.dot(n), v.dot(v) - 1.};
}

// Compute the jacobian of the constraint from calcSurfaceProjectionConstraint
// to the ImplicitSurface's State. The first three columns contain the jacobian
// to the position, and the last three columns to the velocity.
void calcSurfaceProjectionConstraintJacobian(
    const ImplicitSurfaceParams& s,
    const ImplicitSurfaceState& q,
    Mat3x6& jacobian)
{
    const SimTK::Vec3 zeros = {0., 0., 0.};

    // Jacobian of constraint that keeps position on surface.
    jacobian.block<1, 3>(0, 0) =
        s.calcSurfaceConstraintGradient(q); // To position.
    jacobian.block<1, 3>(0, 3) = zeros;           // To velocity.

    // Jacobian of constraint that keeps velocity on the surface normal plane.
    const SimTK::Vec3& v              = q.velocity;
    const SimTK::Vec3 h_v             = s.calcSurfaceConstraintHessian(q) * v;
    const SimTK::Vec3 normal          = calcSurfaceNormal(s, q);
    jacobian.block<1, 3>(1, 0) = s.calcSurfaceConstraintHessian(q) *
                                 (v - v.dot(normal) * normal); // To position.
    jacobian.block<1, 3>(1, 3) = normal;                       // To velocity.

    // Jacobian of constraint that keeps velocity norm to one.
    jacobian.block<1, 3>(2, 0) = zeros;  // To position.
    jacobian.block<1, 3>(2, 3) = 2. * v; // To velocity.
}

// Compute the state projected onto the surface.
// Returns number of required iterations.
size_t calcStateProjectedToSurface(
    const ImplicitSurfaceParams& s,
    ImplicitSurfaceState& q,
    double eps          = 1e-13,
    std::size_t maxIter = 100)
{
    Mat3x6 jacobian;

    double error     = INF;
    size_t iteration = 0;
    for (; iteration < maxIter; ++iteration) {
        SimTK::Vec3 vec = calcSurfaceProjectionConstraint(s, q);

        // TODO use Inf norm.
        error = vec.norm();

        if (error < eps) {
            return iteration;
        }

        calcSurfaceProjectionConstraintJacobian(s, q, jacobian);

        // TODO use other decomposition if needed.
        Vec6 step = -jacobian.colPivHouseholderQr().solve(vec);
        q += step;
    }

    // TODO use opensim's assert
    throw std::runtime_error("Failed to project state to implicit surface");
}

std::pair<ImplicitGeodesicState, ImplicitGeodesicState> 
ImplicitSurfaceParams::
calcLocalGeodesic(
        State qStart,
        double length,
        std::vector<SimTK::Vec3>& log) const
{
    size_t steps = _impl->integrationSteps;
    double s   = 0.;
    double ds  = length / static_cast<double>(steps);
    ImplicitGeodesicState xStart(std::move(qStart));
    ImplicitGeodesicState xEnd = xStart;

    log.clear();
    log.resize(steps);
    log.push_back(xEnd.postion);

    for (size_t k = 0; k < steps; ++k) {
        RungeKutta4(*this, xEnd, s, ds);
        // TODO can be improved by split integration.
        calcStateProjectedToSurface(*this, xEnd);
        // TODO don't log all points.
        log.push_back(xEnd.postion);
    }
    std::cout << "s = " << s << std::endl;

    return {std::move(xStart), std::move(xEnd)};
}

// TODO NOT YET IMPLEMENTED
ImplicitSurfaceState
ImplicitSurfaceParams::
calcInitState(
        const SimTK::Vec3& pStart,
        const SimTK::Vec3& pEnd) const
{
    SimTK::Vec3 d = pEnd - pStart;
    ImplicitSurfaceState q {pStart, d/d.norm()};
    calcStateProjectedToSurface(*this, q);
    return q;
}

// TODO NOT YET IMPLEMENTED
double
ImplicitSurfaceParams::
calcInitLength(
        const SimTK::Vec3& pStart,
        const SimTK::Vec3& pEnd) const
{
    const SimTK::Vec3 pStartGuess = calcInitState(pStart, pEnd).postion;
    const SimTK::Vec3 pEndGuess = calcInitState(pEnd, pStart).postion;
    return (pStartGuess - pEndGuess).norm();
}

// TODO CORRECT?
void
ImplicitSurfaceParams::
applyVariation(
        const GeodesicVariation& var,
        ImplicitSurfaceState& q) const
{
    DarbouxFrame frame = calcDarbouxFrame(*this, q);

    q.postion += var.dBeta * frame.n
        + var.dTangential * frame.t;
    q.velocity += cos(var.dTheta) * frame.t
        + sin(var.dTheta) * frame.b;

    calcStateProjectedToSurface(*this, q);
}

//==============================================================================
//                      IMPLICIT SPHERE IMPL
//==============================================================================

ImplicitSphere::ImplicitSphere(double radius) : _radius(radius)
{
    for (size_t r = 0; r < _hessian.rows(); ++r) {
        for (size_t c = 0; c < _hessian.cols(); ++c) {
            _hessian(r, c) = r == c ? 2. : 0.;
        }
    }
}

double ImplicitSphere::calcSurfaceConstraint(const SimTK::Vec3& p) const
{
    return p.dot(p) - _radius * _radius;
}

SimTK::Vec3 ImplicitSphere::calcSurfaceConstraintGradient(const SimTK::Vec3& p) const
{
    return 2. * p;
}

const ImplicitSurfaceParamsImpl::Hessian& ImplicitSphere::calcSurfaceConstraintHessian(const SimTK::Vec3&) const
{
    return _hessian;
}

//==============================================================================
//                      
//==============================================================================

ImplicitSurfaceState::Dot
ImplicitSurfaceState::calcDerivative(const SimTK::Vec3& acceleration)
const
{
    return {velocity, acceleration};
}

JacobiFieldScalar::Dot
JacobiFieldScalar::calcDerivative(double gaussianCurvature)
const
{
    return {derivative, -gaussianCurvature*value};
}

ImplicitGeodesicState::Dot
ImplicitGeodesicState::calcDerivative(
const SimTK::Vec3& acceleration,
        double gaussianCurvature)
const
{
    ImplicitGeodesicState::Dot dx;
    const ImplicitSurfaceState q = *this;
    // TODO magic numbers, better way to switch betweeen struct and vector?
    dx.topRows(6) = q.calcDerivative(acceleration);
    reinterpret_cast<JacobiFieldScalar::Dot&>(dx[6]) = a.calcDerivative(gaussianCurvature);
    reinterpret_cast<JacobiFieldScalar::Dot&>(dx[8]) = r.calcDerivative(gaussianCurvature);
    return dx;
}

ImplicitGeodesicState&
ImplicitGeodesicState::
operator+=(const ImplicitGeodesicState::Dot &derivative)
{
    ImplicitSurfaceState& q = *this;
    q += derivative.topRows(6);
    a += reinterpret_cast<const JacobiFieldScalar::Dot&>(derivative[6]);
    r += reinterpret_cast<const JacobiFieldScalar::Dot&>(derivative[8]);
    return *this;
}

//==============================================================================
//                      IMPLICIT SURFACE OBJECT
//==============================================================================

GeodesicCurveState
calcGeodesicCurveState(
        const ImplicitSurfaceParams& s,
        const ImplicitGeodesicState& x)
{
    GeodesicCurveState out;
    out.frame = calcDarbouxFrame(s, x);
    out.position = x.postion;
    out.normalCurvature = calcNormalCurvature(s, x);
    out.geodesicTorsion = calcGeodesicTorsion(s, x);
    out.a = x.a;
    out.r = x.r;
    return out;
}

ImplicitSurfaceObject::ImplicitSurfaceObject(
        ImplicitSurfaceParams&& params) :
    _surface(std::move(params))
{}

void
ImplicitSurfaceObject::calcInitState(
        const SimTK::Vec3& pStart,
        const SimTK::Vec3& pEnd)
{
    _startState = _surface.calcInitState(pStart, pEnd);
    _length = _surface.calcInitLength(pStart, pEnd);
}

void
ImplicitSurfaceObject::calcLocalGeodesic(
        GeodesicCurveState& start,
        GeodesicCurveState& end,
        double& length,
        std::vector<SimTK::Vec3>& log)
{
    auto geodesicEndPoints = _surface.calcLocalGeodesic(_startState, length, log);
    start = calcGeodesicCurveState(_surface, geodesicEndPoints.first);
    end = calcGeodesicCurveState(_surface, geodesicEndPoints.second);
    length = _length;
}

void ImplicitSurfaceObject::applyVariation(
        const GeodesicVariation& var)
{
    _surface.applyVariation(var, _startState);
}


//==============================================================================
//                      WRAPOBJECT COMPONENT
//==============================================================================

WrapObject::WrapObject(
        std::unique_ptr<ImplicitSurfaceParamsImpl>&& surface,
        std::shared_ptr<Transform> transformHandle)
    : WrapObject(
            std::unique_ptr<ImplicitSurfaceObject>(
            std::make_unique<ImplicitSurfaceObject>(std::move(surface))), transformHandle)
    /* : _impl( */
    /*         std::unique_ptr<ImplicitSurfaceObject>( */
    /*         std::make_unique<ImplicitSurfaceObject>(std::move(surface)))) */
{
}

WrapObject::WrapObject(
        std::unique_ptr<WrapObjectImpl>&& surface,
        std::shared_ptr<Transform> transformHandle)
    : _impl(std::move(surface)), _transform(std::move(transformHandle)) {}

//==============================================================================
//                      NATURAL VARIATIONS OF DARBOUX FRAME
//==============================================================================

// write down the wrapping constraints
// Take derivatives to darboux
// Take derivatives of darboux to variations
// Compute variations from newton
// Apply variations to surface coordinates

// For both start and end
SimTK::Vec3 calcStartVariationToTangentialDisplacementVariation(
    double normalCurvature,
    double geodesicTorque)
{
    return {NAN, NAN, NAN};
}

// For start
SimTK::Vec3 calcStartVariationToBinormalDisplacementVariation(
    double normalCurvature,
    double geodesicTorque)
{
    return {NAN, NAN, NAN};
}

// For end
SimTK::Vec3 calcEndVariationToBinormalDisplacementVariation(
    double normalCurvature,
    double geodesicTorque,
    double a,
    double aDot)
{
    return {NAN, NAN, NAN};
}

}
