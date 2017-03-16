#ifndef MRS_DEGROOTE2016MUSCLE_H_
#define MRS_DEGROOTE2016MUSCLE_H_

/// This class template implements the muscle model from De Groote et al.,
/// 2016, Evaluation of Direct Collocation Optimal Control Problem
/// Formulations for Solving the Muscle Redundancy Problem (see supplemental
/// materials), and is partially based on the implementation at
/// http://simtk.org/projects/optcntrlmuscle.
/// This class template is not part of OpenSim.
template <typename T>
class DeGroote2016Muscle {
public:
    DeGroote2016Muscle() = default;
    DeGroote2016Muscle(const double& max_isometric_force,
                       const double& optimal_fiber_length,
                       const double& tendon_slack_length,
                       const double& pennation_angle_at_optimal,
                       const double& max_contraction_velocity) :
            _max_isometric_force(max_isometric_force),
            _optimal_fiber_length(optimal_fiber_length),
            _tendon_slack_length(tendon_slack_length),
            _pennation_angle_at_optimal(pennation_angle_at_optimal),
            _max_contraction_velocity(max_contraction_velocity) {
        _fiber_width = optimal_fiber_length*sin(pennation_angle_at_optimal);
    }

    double get_max_isometric_force() const { return _max_isometric_force; }
    double get_optimal_fiber_length() const { return _optimal_fiber_length; }
    double get_tendon_slack_length() const { return _tendon_slack_length; }
    double get_pennation_angle_at_optimal() const
    { return _pennation_angle_at_optimal; }
    double get_max_contraction_velocity() const
    { return _max_contraction_velocity; }

    T calcNormTendonForce(const T& musTenLength,
                          const T& normFiberLength) const {
        // Tendon force-length curve.
        static const double kT = 35;
        static const double c1 = 0.200;
        static const double c2 = 0.995;
        static const double c3 = 0.250;
        // TODO computing tendon force is why we'd want a single "continuous"
        // function rather than separate dynamics and path constraints
        // functions.
        const T fibLen = normFiberLength * _optimal_fiber_length;
        // Tendon length.
        // lT = lMT - sqrt(lM^2 - w^2)
        const T tenLen = musTenLength
                       - sqrt(fibLen*fibLen - _fiber_width*_fiber_width);
        const T normTenLen = tenLen / _tendon_slack_length;
        return c1 * exp(kT * (normTenLen - c2)) - c3;
    }
    void calcTendonForce(const T& musTenLength, const T& normFiberLength,
                         T& tendonForce) const {
        tendonForce = _max_isometric_force
                * calcNormTendonForce(musTenLength, normFiberLength);
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
    void calcEquilibriumResidual(const T& musTenLength,
                                 const T& activation,
                                 const T& normFiberLength,
                                 const T& normFiberVelocity,
                                 T& residual, T& normTendonForce) const {
        // Curve parameters.
        // -----------------
        // Active force-length curve.
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

        // Passive force-length curve.
        static const double kPE = 4.0;
        static const double e0  = 0.6;

        // Muscle force-velocity.
        static const double d1 = -0.318;
        static const double d2 = -8.149;
        static const double d3 = -0.374;
        static const double d4 =  0.886;

        // Intermediate quantities.
        // ------------------------
        const T fibLength = normFiberLength*_optimal_fiber_length;
        // Tendon length.
        // lT = lMT - sqrt(lM^2 - w^2)
        const T tenLength = musTenLength
                - sqrt(fibLength*fibLength - _fiber_width*_fiber_width);
        const T cosPenn = (musTenLength - tenLength)/fibLength;

        // Curves/multipliers.
        // -------------------
        // Tendon force-length curve.
        normTendonForce = calcNormTendonForce(musTenLength,
                                              normFiberLength);

        // Active force-length curve.
        // Sum of 3 gaussians.
        const T activeForceLenMult =
                gaussian(normFiberLength, b11, b21, b31, b41) +
                        gaussian(normFiberLength, b12, b22, b32, b42) +
                        gaussian(normFiberLength, b13, b23, b33, b43);

        // Passive force-length curve.
        const T passiveFibForce = (exp(kPE*(normFiberLength - 1)/e0) - 1)/
                (exp(kPE) - 1);

        // Force-velocity curve.
        const T tempV = d2*normFiberVelocity + d3;
        const T tempLogArg = tempV + sqrt(pow(tempV, 2) + 1);
        const T forceVelMult = d1*log(tempLogArg) + d4;

        // Equilibrium constraint.
        // -----------------------
        const T normFibForce =
                activation*activeForceLenMult*forceVelMult
                        + passiveFibForce;
        const T normFibForceAlongTen = normFibForce*cosPenn;

        residual = normFibForceAlongTen - normTendonForce;
    }
    /// This alternative does not return normalized tendon force.
    void calcEquilibriumResidual(const T& musTenLength,
                                 const T& activation,
                                 const T& normFiberLength,
                                 const T& normFiberVelocity,
                                 T& residual) const {
        T normTendonForce;
        calcEquilibriumResidual(musTenLength, activation, normFiberLength,
                                normFiberVelocity, residual, normTendonForce);
    }

protected:
    static T gaussian(const T& x, const double& b1, const double& b2,
                      const double& b3, const double& b4) {
        return b1*exp((-0.5*pow(x - b2, 2))/(b3 + b4*x));
    };

private:
    constexpr static double NaN = std::numeric_limits<double>::quiet_NaN();
    double _max_isometric_force = NaN;
    double _optimal_fiber_length = NaN;
    double _tendon_slack_length = NaN;
    double _pennation_angle_at_optimal = NaN;
    double _max_contraction_velocity = NaN;
    double _fiber_width = NaN;
};

#endif // MRS_DEGROOTE2016MUSCLE_H_
