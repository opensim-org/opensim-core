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
#include <tropter/EigenUtilities.h>

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
    /// Linearly interpolate (upsample or downsample) the continuous variables 
    /// (i.e. states and controls) within this iterate to produce a new iterate 
    /// with a desired number of columns with equally spaced time points. This 
    /// is useful when forming an initial guess for an optimal control problem. 
    /// If the size of time matches desired_num_columns, then we return a copy 
    /// of this iterate (no interpolation).
    /// @returns the interpolated iterate.
    // TODO comment about transcription specific differences
    virtual Iterate
    interpolate_iterate(const Iterate&, int desired_num_columns) const = 0;
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

    /// When calculating total hessian sparsity and using repeated diagonal 
    /// sparsity blocks to avoid redundant calculations for each mesh point,
    /// how should these blocks be calculated?
    ///  "dense": Mesh point blocks are assumed dense (conservative, default 
    ///           mode)
    /// "sparse": Mesh point block sparsity is detected from the optimal control
    ///           problem initial guess. 
    void set_hessian_block_sparsity_mode(std::string mode) {
        TROPTER_VALUECHECK(mode == "dense" || mode == "sparse",
            "hessian block sparsity mode", mode, "dense or sparse");
        m_hessian_block_sparsity_mode = mode; 
    }
    /// @copydoc set_hessian_block_sparsity_mode()
    std::string get_hessian_block_sparsity_mode () const
    {   return m_hessian_block_sparsity_mode; }

    // This empty vector is passed to calc_differential_algebraic_equations()
    // for collocation points not on the mesh where we do not enforce path 
    // constraints. If the user tries to write to it, an Eigen runtime assertion 
    // will be violated. If the user tries to resize it, tropter will throw an 
    // exception after exiting the function call.
    mutable VectorX<T> m_empty_path_constraint_col;
    // This empty vector is passed to calc_differential_algebraic_equations()
    // for collocation points on the mesh where we do not have interstep
    // variables. If the user tries to write to it, an Eigen runtime assertion 
    // will be violated. 
    mutable VectorX<T> m_empty_interstep_col;

private:
    std::string m_hessian_block_sparsity_mode{"dense"};

};

} // namespace transcription

} // namespace tropter

//namespace {
//
//    // We can use Eigen's Spline module for linear interpolation, though it's
//    // not really meant for this.
//    // https://eigen.tuxfamily.org/dox/unsupported/classEigen_1_1Spline.html
//    // The independent variable must be between [0, 1].
//    using namespace Eigen;
//    RowVectorXd normalize(RowVectorXd x) {
//        const double lower = x[0];
//        const double denom = x.tail<1>()[0] - lower;
//        for (Index i = 0; i < x.size(); ++i) {
//            // We assume that x is non-decreasing.
//            x[i] = (x[i] - lower) / denom;
//        }
//        return x;
//    }
//
//    MatrixXd interp1(const RowVectorXd& xin, const MatrixXd yin,
//        const RowVectorXd& xout) {
//        // Make sure we're not extrapolating.
//        assert(xout[0] >= xin[0]);
//        assert(xout.tail<1>()[0] <= xin.tail<1>()[0]);
//
//        typedef Spline<double, 1> Spline1d;
//
//        MatrixXd yout(yin.rows(), xout.size());
//        RowVectorXd xin_norm = normalize(xin);
//        RowVectorXd xout_norm = normalize(xout);
//        for (Index irow = 0; irow < yin.rows(); ++irow) {
//            const Spline1d spline = SplineFitting<Spline1d>::Interpolate(
//                yin.row(irow), // dependent variable.
//                1, // linear interp
//                xin_norm); // "knot points" (independent variable).
//            for (Index icol = 0; icol < xout.size(); ++icol) {
//                yout(irow, icol) = spline(xout_norm[icol]).value();
//            }
//        }
//        return yout;
//    }
//}

#endif // TROPTER_OPTIMALCONTROL_TRANSCRIPTION_BASE_H
