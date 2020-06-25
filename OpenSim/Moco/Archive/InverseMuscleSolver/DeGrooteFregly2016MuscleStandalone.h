#ifndef OPENSIM_DEGROOTEFREGLY2016MUSCLESTANDALONE_H_
#define OPENSIM_DEGROOTEFREGLY2016MUSCLESTANDALONE_H_
/* -------------------------------------------------------------------------- *
 * OpenSim: DeGrooteFregly2016MuscleStandalone.h                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#ifdef _MSC_VER
    // Ignore warnings from ADOL-C headers.
    #pragma warning(push)
    // 'argument': conversion from 'size_t' to 'locint', possible loss of data.
    #pragma warning(disable: 4267)
#endif
#include <adolc/adouble.h>
#ifdef _MSC_VER
    #pragma warning(pop)
#endif


/// This class template implements the muscle model from De Groote et al.,
/// 2016, Evaluation of Direct Collocation Optimal Control Problem
/// Formulations for Solving the Muscle Redundancy Problem (see supplemental
/// materials), and is partially based on the implementation at
/// http://simtk.org/projects/optcntrlmuscle.
/// This class template is not part of OpenSim.
// TODO unify the function signatures (return types).
// TODO splitting calcEquilibriumResidual() into calcNorm*() functions caused
// testSingleMuscleDeGrooteFregly2016()'s trajectory optimization and
// INDYGO to increase duration from 9 seconds to 13 seconds
// (in Debug).
template <typename T>
class DeGrooteFregly2016MuscleStandalone {
public:
    DeGrooteFregly2016MuscleStandalone() = default;
    DeGrooteFregly2016MuscleStandalone(const double& max_isometric_force,
                       const double& optimal_fiber_length,
                       const double& tendon_slack_length,
                       const double& pennation_angle_at_optimal,
                       const double& max_contraction_velocity) :
            _max_isometric_force(max_isometric_force),
            _optimal_fiber_length(optimal_fiber_length),
            _tendon_slack_length(tendon_slack_length),
            _pennation_angle_at_optimal(pennation_angle_at_optimal),
            _max_contraction_velocity(max_contraction_velocity) {
        _norm_fiber_width = sin(pennation_angle_at_optimal);
        _fiber_width = optimal_fiber_length * _norm_fiber_width;
    }
    DeGrooteFregly2016MuscleStandalone<double> convert_scalartype_double() const {
        return DeGrooteFregly2016MuscleStandalone<double>(_max_isometric_force,
                _optimal_fiber_length, _tendon_slack_length,
                _pennation_angle_at_optimal, _max_contraction_velocity);
    }

    double get_max_isometric_force() const { return _max_isometric_force; }
    double get_optimal_fiber_length() const { return _optimal_fiber_length; }
    double get_tendon_slack_length() const { return _tendon_slack_length; }
    double get_pennation_angle_at_optimal() const
    { return _pennation_angle_at_optimal; }
    double get_max_contraction_velocity() const
    { return _max_contraction_velocity; }

private:
    // TODO allow users to set kT; it has a huge effect on the runtime of
    // INDYGO.
    constexpr static double kT = 35;
    constexpr static double c1 = 0.200;
    constexpr static double c2 = 0.995;
    constexpr static double c3 = 0.250;
public:
    // TODO rename to calcTendonForceLengthCurve().
    T calcNormTendonForce(const T& normTendonLength) const {
        // Tendon force-length curve.
        return c1 * exp(kT * (normTendonLength - c2)) - c3;
    }
    /// For use with the tendon force state formulation.
    T calcTendonForceLengthCurveInverse(const T& normTendonForce) const {
        return 1.0 / kT * log(1 / c1 * (normTendonForce + c3)) + c2;
    }
    T calcNormTendonForce(const T& musTenLength,
                          const T& normFiberLength) const {
        // TODO computing tendon force is why we'd want a single "continuous"
        // function rather than separate dynamics and path constraints
        // functions.
        const T fibLen = normFiberLength * _optimal_fiber_length;
        // Tendon length.
        // lT = lMT - sqrt(lM^2 - w^2)
        const T tenLen = musTenLength
                       - sqrt(fibLen*fibLen - _fiber_width*_fiber_width);
        const T normTenLen = tenLen / _tendon_slack_length;
        // std::cout << "DEBUG normTenLen " << static_cast<const double&>
        // (normTenLen + 0) << " " <<
        //         static_cast<const double&>(fibLen + 0) << " " <<
        //         _tendon_slack_length << std::endl;
        // TODO Friedl did not find it necessary to clip.
        // TODO need to handle buckling.
        // return fmax(0, c1 * exp(kT * (normTenLen - c2)) - c3);
        return calcNormTendonForce(normTenLen);
    }
    void calcTendonForce(const T& musTenLength, const T& normFiberLength,
                         T& tendonForce) const {
        tendonForce = _max_isometric_force
                * calcNormTendonForce(musTenLength, normFiberLength);
    }
    /// If you differentiate the tendon force-length curve with respect to
    /// normalized tendon length, and then solve for the rate of change of
    /// normalized tendon length, you get this function.
    /// The units are 1/s. To obtain tendon lengthening rate in m/s,
    /// multiply by tendon slack length.
    /// This function is for use with the tendon force state formulation.
    T calcNormTendonVelocity(const T& normTenForceRate,
            const T& normTendonLength) const {
        return normTenForceRate / (c1 * kT * exp(kT * (normTendonLength - c2)));
    }
    T calcActiveForceLengthMultiplier(const T& normFiberLength) const {
        static const double b11 =  0.815;
        static const double b21 =  1.055;
        static const double b31 =  0.162;
        static const double b41 =  0.063;
        static const double b12 =  0.433;
        static const double b22 =  0.717;
        static const double b32 = -0.030;
        static const double b42 =  0.200;
        static const double b13 =  0.100;
        static const double b23 =  1.000;
        static const double b33 =  0.354;
        static const double b43 =  0.000;
        // Sum of 3 gaussians.
        return gaussian(normFiberLength, b11, b21, b31, b41) +
               gaussian(normFiberLength, b12, b22, b32, b42) +
               gaussian(normFiberLength, b13, b23, b33, b43);
    }
    T calcForceVelocityMultiplier(const T& normFiberVelocity) const {

        // Muscle force-velocity.
        static const double d1 = -0.318;
        static const double d2 = -8.149;
        static const double d3 = -0.374;
        static const double d4 =  0.886;

        const T tempV = d2*normFiberVelocity + d3;
        const T tempLogArg = tempV + sqrt(pow(tempV, 2) + 1);
        return  d1*log(tempLogArg) + d4;
    }
    T calcNormPassiveFiberForce(const T& normFiberLength) const {
        // Passive force-length curve.
        static const double kPE = 4.0;
        static const double e0  = 0.6;
        static const double denom = exp(kPE) - 1;
        static const double numer_offset = exp(kPE * (0.2 - 1)/e0);
        // The version of this equation in the supplementary materials of De
        // Groote, et al. 2016 has an error. The correct equation passes
        // through y = 0 at x = 0.2, and therefore is never negative within
        // the allowed range of the optimal fiber length. The version in the
        // supplementary materials allows for negative forces.
        return (exp(kPE*(normFiberLength - 1)/e0) - numer_offset) / denom;
    }
    T calcNormFiberForce(const T& activation,
                         const T& normFiberLength,
                         const T& normFiberVelocity) const {

        // Active force-length curve.
        const T activeForceLenMult =
                calcActiveForceLengthMultiplier(normFiberLength);

        // Passive force-length curve.
        const T normPassiveFibForce =
                calcNormPassiveFiberForce(normFiberLength);

        // Force-velocity curve.
        const T forceVelMult = calcForceVelocityMultiplier(normFiberVelocity);

        //std::cout << "DEBUG fib force comps "
        //        << static_cast<const double&>(activation + 0) << " "
        //        << static_cast<const double&>(activeForceLenMult + 0) << " "
        //        << static_cast<const double&>(forceVelMult + 0) << " "
        //        << static_cast<const double&>(normPassiveFibForce + 0) << " "
        //        << std::endl;
        return activation*activeForceLenMult*forceVelMult + normPassiveFibForce;
    }
    T calcNormFiberForceAlongTendon(const T& activation,
                                    const T& normFiberLength,
                                    const T& normFiberVelocity) const {
        const T normFiberForce = calcNormFiberForce(activation,
                                                    normFiberLength,
                                                    normFiberVelocity);
        const T cosPenn = sqrt(normFiberLength*normFiberLength -
                _norm_fiber_width*_norm_fiber_width) / normFiberLength;
        return normFiberForce * cosPenn;
    }
    /// There are contexts where we may want to use a different numeric type
    /// than T, which is why this function is templated on scalar type.
    // TODO the strategy of templating member functions won't work in OpenSim.
    template <typename S>
    void calcRigidTendonFiberKinematics(const S& musTenLength,
                                        const S& musTenVelocity,
                                        S& normFiberLength,
                                        S& normFiberVelocity) const {
        // TODO there is too much repetition of calculations surrounding
        // lengths and pennation.
        // TODO can we use temporaries (adub etc) to speed up calculation?
        const S fiberLengthAlongTendon = musTenLength - _tendon_slack_length;
        const S fiberLength = sqrt(pow(musTenLength - _tendon_slack_length, 2)
                                           + pow(_fiber_width, 2));
        normFiberLength = fiberLength / _optimal_fiber_length;
        const S cosPenn = fiberLengthAlongTendon / fiberLength;
        // lMT = lT + lM cos(alpha) -> differentiate:
        // vMT = vM cos(alpha) - lM alphaDot sin(alpha) (1)
        // w = lM sin(alpha) -> differentiate:
        // 0 = lMdot sin(alpha) + lM alphaDot cos(alpha) ->
        // alphaDot = -vM sin(alpha) / (lM cos(alpha)) -> plug into (1)
        // vMT = vM cos(alpha) + vM sin^2(alpha) / cos(alpha)
        // vMT = vM (cos^2(alpha) + sin^2(alpha)) / cos(alpha)
        // vM = vMT cos(alpha)
        const S fiberVelocity = musTenVelocity * cosPenn;
        // TODO cache max_contraction_velocity * opt_fib_len
        normFiberVelocity = fiberVelocity /
                (_max_contraction_velocity * _optimal_fiber_length);
    }
    /// Compute the fiber force projected along the tendon, under the assumption
    /// that the tendon is rigid.
    T calcRigidTendonFiberForceAlongTendon(const T& activation,
                                           const T& musTenLength,
                                           const T& musTenVelocity) const {
        T normFiberLength;
        T normFiberVelocity;
        calcRigidTendonFiberKinematics(musTenLength, musTenVelocity,
                                          normFiberLength, normFiberVelocity);
        // TODO what about buckling the tendon (MTU length < slack length)?
        const T normFiberForce = calcNormFiberForceAlongTendon(
                activation, normFiberLength, normFiberVelocity);
        return _max_isometric_force * normFiberForce;
    }
    void calcActivationDynamics(const T& excitation, const T& activation,
                                T& activationDot) const {
        static const double actTimeConst   = 0.015;
        static const double deactTimeConst = 0.060;
        static const double tanhSteepness  = 0.1;
        //     f = 0.5 tanh(b(e - a))
        //     z = 0.5 + 1.5a
        // da/dt = [(f + 0.5)/(tau_a * z) + (-f + 0.5)*z/tau_d] * (e - a)
        const T timeConstFactor = 0.5 + 1.5 * activation;
        const T tempAct = 1.0 / (actTimeConst * timeConstFactor);
        const T tempDeact = timeConstFactor / deactTimeConst;
        const T f = 0.5 * tanh(tanhSteepness * (excitation - activation));
        const T timeConst = tempAct * (f + 0.5) + tempDeact * (-f + 0.5);
        activationDot = timeConst * (excitation - activation);
    }
    void calcEquilibriumResidual(const T& activation,
                                 const T& musTenLength,
                                 const T& normFiberLength,
                                 const T& normFiberVelocity,
                                 T& residual, T& normTendonForce) const {
        const T normFibForceAlongTen = calcNormFiberForceAlongTendon(
                activation, normFiberLength, normFiberVelocity);

        normTendonForce = calcNormTendonForce(musTenLength, normFiberLength);

        //std::cout << "DEBUG eq " <<
        //        static_cast<const double&>(normTendonForce) <<
        //        " " <<
        //        static_cast<const double&>(normFibForceAlongTen + 0) << " " <<
        //        _tendon_slack_length <<
        //       std::endl;
        residual = normFibForceAlongTen - normTendonForce;
    }
    /// This alternative does not return normalized tendon force.
    void calcEquilibriumResidual(const T& activation,
                                 const T& musTenLength,
                                 const T& normFiberLength,
                                 const T& normFiberVelocity,
                                 T& residual) const {
        T normTendonForce;
        calcEquilibriumResidual(activation, musTenLength, normFiberLength,
                                normFiberVelocity, residual, normTendonForce);
    }
    /// Useful when using the tendon force state formulation to compute
    /// intermediate quantities.
    void calcIntermediatesForTendonForceState(const T& musTenLength,
            const T& musTenVelocity, const T& normTenForce,
            const T& normTenForceRate,
            T& normFibLength, T& normFibVelocity, T& cosPenn, T& tendonForce)
                const {

        const T normTenLength = calcTendonForceLengthCurveInverse(normTenForce);
        const T tendonLength = _tendon_slack_length * normTenLength;
        const T fiberLength = sqrt(pow(musTenLength - tendonLength, 2)
                + pow(_fiber_width, 2));

        normFibLength = fiberLength / _optimal_fiber_length;
        cosPenn = (musTenLength - tendonLength) / fiberLength;

        const T tendonVelocity = _tendon_slack_length *
                calcNormTendonVelocity(normTenForceRate, normTenLength);
        const T fibVelocity = (musTenVelocity - tendonVelocity) * cosPenn;
        normFibVelocity = fibVelocity /
                (_max_contraction_velocity * _optimal_fiber_length);

        tendonForce = _max_isometric_force * normTenForce;
    }
    /// Compute the equilibrium error where we know the normalized tendon
    /// force but do not know the fiber length.
    void calcTendonForceStateEquilibriumResidual(const T& activation,
            const T& musTenLength,
            const T& musTenVelocity,
            const T& normTenForce,
            const T& normTenForceRate,
            T& residual,
            T& tendonForce) const {

        T normFibLength;
        T normFibVelocity;
        T cosPenn;
        calcIntermediatesForTendonForceState(musTenLength, musTenVelocity,
                normTenForce, normTenForceRate,
                normFibLength, normFibVelocity, cosPenn, tendonForce);
        const T normFibForce = calcNormFiberForce(activation, normFibLength,
                normFibVelocity);
        residual = normFibForce * cosPenn - normTenForce;
    }

protected:
    static T gaussian(const T& x, const double& b1, const double& b2,
                      const double& b3, const double& b4) {
        return b1*exp((-0.5*pow(x - b2, 2)) / pow(b3 + b4*x, 2));
    };

private:
    constexpr static double NaN = std::numeric_limits<double>::quiet_NaN();
    double _max_isometric_force = NaN;
    double _optimal_fiber_length = NaN;
    double _tendon_slack_length = NaN;
    double _pennation_angle_at_optimal = NaN;
    double _max_contraction_velocity = NaN;
    double _norm_fiber_width = NaN;
    double _fiber_width = NaN;
};

#endif // OPENSIM_DEGROOTEFREGLY2016MUSCLESTANDALONE_H_
