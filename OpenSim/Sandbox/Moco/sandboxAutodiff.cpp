#include <casadi/casadi.hpp>
#include <OpenSim/Moco/osimMoco.h>

using namespace casadi;

/// Translate a point mass in one dimension in minimum time. This is a very
/// simple example that shows only the basics of Moco.
///
/// @verbatim
/// minimize   t_f
/// subject to xdot = v
///            vdot = F/m
///            x(0)   = 0
///            x(t_f) = 1
///            v(0)   = 0
///            v(t_f) = 0
/// w.r.t.     x   in [-5, 5]    position of mass
///            v   in [-50, 50]  speed of mass
///            F   in [-50, 50]  force applied to the mass
///            t_f in [0, 5]     final time
/// constants  m       mass
/// @endverbatim

class TranscriptionSlidingMass {
public:
    TranscriptionSlidingMass(int degree, int numMeshIntervals)
            : m_degree(degree), m_numMeshIntervals(numMeshIntervals) {

        // Transcription scheme info (Legendre-Gauss).
        m_numGridPoints = m_numMeshIntervals * (m_degree + 1) + 1;
        m_numDefectsPerMeshInterval = (degree + 1) * m_numStates;
        m_numConstraints = m_numDefectsPerMeshInterval * m_numMeshIntervals;

        m_legendreRoots = casadi::collocation_points(degree, "legendre");
        casadi::collocation_coeff(m_legendreRoots,
                m_differentiationMatrix,
                m_interpolationCoefficients,
                m_quadratureCoefficients);

         // Create the mesh.
        for (int i = 0; i < (numMeshIntervals + 1); ++i) {
            m_mesh.push_back(i / (double)(numMeshIntervals));
        }

        // Create the grid.
        m_grid = casadi::DM::zeros(1, m_numGridPoints);
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            const double t_i = m_mesh[imesh];
            const double t_ip1 = m_mesh[imesh + 1];
            int igrid = imesh * (m_degree + 1);
            m_grid(igrid) = t_i;
            for (int d = 0; d < m_degree; ++d) {
                m_grid(igrid + d + 1) = t_i + (t_ip1 - t_i) * m_legendreRoots[d];
            }
        }
        m_grid(m_numGridPoints - 1) = m_mesh[m_numMeshIntervals];

        // Create variables and set bounds.
        createVariablesAndSetBounds();
    }

    void createVariablesAndSetBounds() {
        // Create variables.
        m_variables[initial_time] = MX::sym("initial_time");
        m_variables[final_time] = MX::sym("final_time");
        m_variables[states] = MX::sym("states", m_numStates, m_numGridPoints);
        m_variables[controls] = MX::sym("controls", m_numControls, 
                m_numGridPoints);

        // Create the time vector.
        m_times = createTimes(m_variables[initial_time], m_variables[final_time]);
        m_duration = m_variables[final_time] - m_variables[initial_time];


        // Set variable bounds.
        auto initializeBoundsDM = [&](VariablesDM& bounds) {
        for (auto& kv : m_variables) {
                bounds[kv.first] = DM(kv.second.rows(), kv.second.columns());
            }
        };
        initializeBoundsDM(m_lowerBounds);
        initializeBoundsDM(m_upperBounds);

        setVariableBounds(initial_time, 0, 0, {0, 0});
        setVariableBounds(final_time, 0, 0, {0, 5});

        setVariableBounds(states, 0, 0, {0, 0});
        setVariableBounds(states, 0, -1, {1, 1});
        setVariableBounds(states, 0, Slice(1, m_numGridPoints-1), {-5, 5});

        setVariableBounds(states, 1, 0, {0, 0});
        setVariableBounds(states, 1, -1, {0, 0});
        setVariableBounds(states, 1, Slice(1, m_numGridPoints-1), {-50, 50});

        setVariableBounds(controls, Slice(), Slice(), {-50, 50});
    }

    void setObjective() {
        m_objective = m_variables[final_time];
    }

    void transcribe() {
        // Cost.
        // =====
        setObjective(); 

        // Defects.
        // ========
        const int NS = m_numStates;
        const int NC = m_numControls;
        m_xdot = MX(NS, m_numGridPoints);
        m_constraints.defects = MX(casadi::Sparsity::dense(
                m_numDefectsPerMeshInterval, m_numMeshIntervals));
        m_constraintsLowerBounds.defects =
                DM::zeros(m_numDefectsPerMeshInterval, m_numMeshIntervals);
        m_constraintsUpperBounds.defects =
                DM::zeros(m_numDefectsPerMeshInterval, m_numMeshIntervals);

        calcStateDerivativesSymbolic(m_variables[states], m_variables[controls], 
                m_xdot);

        calcDefects(m_variables[states], m_xdot, m_constraints.defects);
    }

    void calcStateDerivativesSymbolic(const MX& x, const MX& c, MX& xdot) {
        xdot(0, Slice()) = x(1, Slice());
        xdot(1, Slice()) = c(0, Slice()) / m_mass;
    }

    void calcDefects(const casadi::MX& x, const casadi::MX& xdot, 
            casadi::MX& defects) {

        const int NS = m_numStates;
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            int igrid = imesh * (m_degree + 1);
            const auto h = m_times(igrid + m_degree + 1) - m_times(igrid);
            const auto x_i = x(Slice(), Slice(igrid, igrid + m_degree + 1));
            const auto xdot_i = xdot(Slice(), Slice(igrid + 1, igrid + m_degree + 1));
            const auto x_ip1 = x(Slice(), igrid + m_degree + 1);

            // Residual function defects.
            MX residual = h * xdot_i - MX::mtimes(x_i, m_differentiationMatrix);
            for (int d = 0; d < m_degree; ++d) {
                defects(Slice(d * NS, (d + 1) * NS), imesh) = residual(Slice(), d);
            }

            // End state interpolation.
            defects(Slice(m_degree * NS, (m_degree + 1) * NS), imesh) =
                    x_ip1 - MX::mtimes(x_i, m_interpolationCoefficients);
        }
    }

    void solve() {
        // Define the NLP.
        // ---------------
        transcribe();

        // Create a guess.
        // ---------------
        VariablesDM guess;
        guess[initial_time] = 0;
        guess[final_time] = 1;
        guess[states] = DM::zeros(m_numStates, m_numGridPoints);
        guess[controls] = DM::zeros(m_numControls, m_numGridPoints);

        auto x = flattenVariables(m_variables);
        casadi_int numVariables = x.numel();

        auto g = flattenConstraints(m_constraints);
        casadi_int numConstraints = g.numel();

        casadi::MXDict nlp;
        nlp.emplace(std::make_pair("x", x));
        nlp.emplace(std::make_pair("f", m_objective));
        nlp.emplace(std::make_pair("g", g));

        const casadi::Function nlpFunc =
            casadi::nlpsol("nlp", "ipopt", nlp);

        // Run the optimization (evaluate the CasADi NLP function).
        // --------------------------------------------------------
        // The inputs and outputs of nlpFunc are numeric (casadi::DM).
        const casadi::DMDict nlpResult = nlpFunc(casadi::DMDict{
                    {"x0", flattenVariables(guess)},
                    {"lbx", flattenVariables(m_lowerBounds)},
                    {"ubx", flattenVariables(m_upperBounds)},
                    {"lbg", flattenConstraints(m_constraintsLowerBounds)},
                    {"ubg", flattenConstraints(m_constraintsUpperBounds)}});
    }

private:
    // Structs
    // -------
    // Variables
    enum Var {
        initial_time,
        final_time,
        states,
        controls,
    };
    template <typename T>
    using Variables = std::unordered_map<Var, T, std::hash<int>>;
    using VariablesDM = Variables<casadi::DM>;
    using VariablesMX = Variables<casadi::MX>;

    // Constraints
    template <typename T>
    struct Constraints {
        T defects;
    };

    // Bounds
    struct Bounds {
        Bounds() = default;
        Bounds(double lower, double upper) : lower(lower), upper(upper) {}
        double lower = std::numeric_limits<double>::quiet_NaN();
        double upper = std::numeric_limits<double>::quiet_NaN();
        bool isSet() const { return !std::isnan(lower) && !std::isnan(upper); }
    };

    // Helper functions
    // ----------------
    template <typename T>
    T createTimes(const T& initialTime, const T& finalTime) const {
        return (finalTime - initialTime) * m_grid + initialTime;
    }

    template <typename TRow, typename TColumn>
    void setVariableBounds(Var var, const TRow& rowIndices,
            const TColumn& columnIndices, const Bounds& bounds) {
        if (bounds.isSet()) {
            const auto& lower = bounds.lower;
            m_lowerBounds[var](rowIndices, columnIndices) = lower;
            const auto& upper = bounds.upper;
            m_upperBounds[var](rowIndices, columnIndices) = upper;
        } else {
            const auto inf = std::numeric_limits<double>::infinity();
            m_lowerBounds[var](rowIndices, columnIndices) = -inf;
            m_upperBounds[var](rowIndices, columnIndices) = inf;
        }
    }

    template <typename T>
    static std::vector<Var> getSortedVarKeys(const Variables<T>& vars) {
        std::vector<Var> keys;
        for (const auto& kv : vars) { keys.push_back(kv.first); }
        std::sort(keys.begin(), keys.end());
        return keys;
    }

    template <typename T>
    static T flattenVariables(const Variables<T>& vars) {
        std::vector<T> stdvec;
        for (const auto& key : getSortedVarKeys(vars)) {
            stdvec.push_back(vars.at(key));
        }
        return T::veccat(stdvec);
    }

    template <typename T>
    T flattenConstraints(const Constraints<T>& constraints) const {
        T flat = T(casadi::Sparsity::dense(m_numConstraints, 1));

        int iflat = 0;
        auto copyColumn = [&flat, &iflat](const T& matrix, int columnIndex) {
            using casadi::Slice;
            if (matrix.rows()) {
                flat(Slice(iflat, iflat + matrix.rows())) =
                        matrix(Slice(), columnIndex);
                iflat += matrix.rows();
            }
        };

        // Constraints for each mesh interval.
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            copyColumn(constraints.defects, imesh);
        }

        OPENSIM_THROW_IF(iflat != m_numConstraints, OpenSim::Exception,
                "Internal error: final value of the index into the flattened "
                "constraints should be equal to the number of constraints.");
        return flat;
    }

    // Member variables
    // ----------------
    double m_mass = 1.0;
    int m_numStates = 2;
    int m_numControls = 1;
    int m_degree;

    std::vector<double> m_legendreRoots;
    casadi::DM m_differentiationMatrix;
    casadi::DM m_interpolationCoefficients;
    casadi::DM m_quadratureCoefficients;

    std::vector<double> m_mesh;
    casadi::DM m_grid;

    int m_numMeshIntervals;
    int m_numGridPoints;
    int m_numDefectsPerMeshInterval;
    int m_numConstraints;

    VariablesMX m_variables;
    VariablesDM m_lowerBounds;
    VariablesDM m_upperBounds;
    casadi::MX m_times;
    casadi::MX m_duration;
    casadi::MX m_objective;
    casadi::MX m_xdot;

    Constraints<casadi::MX> m_constraints;
    Constraints<casadi::DM> m_constraintsLowerBounds;
    Constraints<casadi::DM> m_constraintsUpperBounds;
};



int main() {
    TranscriptionSlidingMass transcription(3, 50);
    transcription.solve();

    return EXIT_SUCCESS;
}