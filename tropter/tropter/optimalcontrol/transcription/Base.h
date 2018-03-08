#ifndef TROPTER_OPTIMALCONTROL_TRANSCRIPTION_BASE_H
#define TROPTER_OPTIMALCONTROL_TRANSCRIPTION_BASE_H
// ----------------------------------------------------------------------------
// tropter: Base.h
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include <tropter/common.h>
#include <tropter/optimization/ProblemDecorator_double.h>
#include <tropter/optimization/ProblemDecorator_adouble.h>
#include <tropter/optimalcontrol/Iterate.h>

//namespace transcription {
//
//class Trapezoidal;
//
//class Orthogonal;
//
//class Pseudospectral;
//}

//template<typename T>
//class TrapezoidalTranscription : public Problem<T> {
//
//};

//template<typename T>
//class Trapezoidal : public Problem<T> {
//public:
//    struct Trajectory {
//        Eigen::RowVectorXd time;
//        Eigen::MatrixXd states;
//        Eigen::MatrixXd controls;
//    };
//protected:
//    virtual void integrate_integral_cost(const VectorX<T>& integrands,
//            T& integral) const = 0;
//    virtual void compute_defects(const StatesView& states,
//            const MatrixX<T>& derivs,
//            DefectsTrajectoryView& defects);
//};

namespace tropter {
namespace transcription {

/// @ingroup optimalcontrol
template<typename T>
class Base : public optimization::Problem<T> {
public:
    // TODO do we still need this type? Use Iterate instead.
    //struct Trajectory {
    //    Eigen::RowVectorXd time;
    //    Eigen::MatrixXd states;
    //    Eigen::MatrixXd controls;
    //};

    /// Get a vector of names of all variables in the optimization problem,
    /// in the correct order. For continuous variables, the format is
    /// `<continuous-variable-name>_<mesh-point-index>`. The mesh point index is
    /// 0-based.
    virtual std::vector<std::string> get_variable_names() const = 0;
    /// Get a vector of names of the constraints (e.g., defects and path
    /// constraints for each mesh point) in the optimization problem, in the
    /// correct order. For defect constraints, the format is
    /// `<state-variable-name>_<mesh-interval-index>` (1-based index). For path
    /// constraints, the format is `<path-constraint-name>_<mesh-point-index>`
    /// (0-based index).
    virtual std::vector<std::string> get_constraint_names() const = 0;

    /// Create a vector of optimization variables (for the generic
    /// optimization problem) from an states and controls.
    virtual Eigen::VectorXd
    construct_iterate(const Iterate&,
            bool interpolate = false) const = 0;
    // TODO change interface to be a templated function so users can pass in
    // writeable blocks of a matrix.
    virtual Iterate
    deconstruct_iterate(const Eigen::VectorXd& x) const = 0;
    /// Print the value of constraint vector for the given iterate. This is
    /// helpful for troubleshooting why a problem may be infeasible.
    /// This function will try to give meaningful names to the
    /// elements of the constraint vector.
    virtual void print_constraint_values(
            const Iterate&,
            std::ostream& stream = std::cout) const
    {
        stream << "The function print_constraint_values() is unimplemented for "
                "this transcription method." << std::endl;
    }
};

} // namespace transcription

} // namespace tropter

#endif // TROPTER_OPTIMALCONTROL_TRANSCRIPTION_BASE_H
