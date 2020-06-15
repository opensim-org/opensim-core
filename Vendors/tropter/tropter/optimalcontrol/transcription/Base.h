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

    /// When calculating total Hessian sparsity and using repeated diagonal 
    /// sparsity blocks to avoid redundant calculations for each mesh point,
    /// how should these blocks be calculated?
    ///  "dense": Mesh point blocks are assumed dense (conservative, default 
    ///           mode)
    /// "sparse": Mesh point block sparsity is detected from the optimal control
    ///           problem initial guess. 
    void set_exact_hessian_block_sparsity_mode(std::string mode) {
        TROPTER_VALUECHECK(mode == "dense" || mode == "sparse",
            "Hessian block sparsity mode", mode, "dense or sparse");
        m_exact_hessian_block_sparsity_mode = mode; 
    }
    /// @copydoc set_exact_hessian_block_sparsity_mode()
    std::string get_exact_hessian_block_sparsity_mode () const
    {   return m_exact_hessian_block_sparsity_mode; }

private:
    std::string m_exact_hessian_block_sparsity_mode{"dense"};

};

} // namespace transcription

} // namespace tropter

#endif // TROPTER_OPTIMALCONTROL_TRANSCRIPTION_BASE_H
