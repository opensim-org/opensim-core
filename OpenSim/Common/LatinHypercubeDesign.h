#ifndef OPENSIM_LATINHYPERCUBEDESIGN_H
#define OPENSIM_LATINHYPERCUBEDESIGN_H
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  LatinHypercubeDesign.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include "osimCommonDLL.h"
#include "CommonUtilities.h"

namespace OpenSim {

/**
 * A class for generating Latin hypercube designs.
 * 
 * # Latin hypercube design
 *
 * A Latin hypercube is an m-by-n matrix, where 'm' is the number of design
 * samples and 'n' is the number of variables in the design. Each sample point
 * (i.e., each row in the design) is the only sample point in its row and column
 * in the hypercube defined by the number of variables. 
 * 
 * \cond 
 * For example, consider a design with 5 samples and 2 variables:
 *
 *             grid                         matrix
 *        __ __ __ __ __                  __      __
 *       |    x         |                | 0.6  0.2 |
 *       |         x    |                | 1.0  0.4 |
 *   q2  | x            |          q  =  | 0.2  0.6 |
 *       |            x |                | 0.8  0.8 |
 *       |      x       |                | 0.4  1.0 |
 *        ‾‾ ‾‾ ‾‾ ‾‾ ‾‾                  ‾‾      ‾‾
 *             q1
 *  
 * On the left is the 5-by-5 square grid (i.e., 2-D hypercube) that is produced
 * if the samples are plotted in the variable space defined by q1 and q2. On the
 * right is the 5-by-2 design matrix that contains the 5 samples.
 * \endcond
 * 
 *
 * Latin hypercube designs are useful for sampling large, multivariate parameter 
 * spaces. Optimal Latin hypercube designs are those that maximize the minimum 
 * distance between samples in the design. This class provides methods for 
 * generating random Latin hypercube designs, as well as methods for
 * generating Latin hypercube designs using the translational propagation
 * algorithm from Viana et al. (2009) and the enhanced stochastic evolutionary
 * algorithm from Jin et al. (2005).
 *
 * # How to create a Latin hypercube design
 * To create a Latin hypercube design, you must first specify the number of
 * variables and samples in the design, and, optionally, the distance criterion
 * used to evaluate each design. The distance criterion can be either "maximin"
 * or "phi_p", where "maximin" is the default. The "phi_p" distance criterion
 * is an approximation of the "maximin" criterion that can be used to generate
 * designs that minimize the sum of inverse distances raised to a large exponent
 * (see [2] for more details). The exponent used for the "phi_p" distance
 * criterion can be set using the setPhiPDistanceExponent() method.
 *
 * For example, a random Latin hypercube design with 10 samples and 3 variables
 * can be created as follows:
 * @code
 * LatinHypercubeDesign lhs;
 * lhs.setNumSamples(10);
 * lhs.setNumVariables(3);
 * SimTK::Matrix design = lhs.generateRandomDesign();
 * @endcode
 *
 * A design using the translational propagation algorithm can be created as
 * follows:
 * @code
 * SimTK::Matrix tplhsDesign = lhs.generateTranslationalPropagationDesign();
 * @endcode
 *
 * A design using the enhanced stochastic evolutionary algorithm can be created
 * as follows:
 * @code
 * SimTK::Matrix eseaDesign = lhs.generateStochasticEvolutionaryDesign();
 * @endcode
 *
 * Finally, each design can be evaluated using the evaluateDesign() method:
 * @code
 * double score = lhs.evaluateDesign(design);
 * @endcode
 *
 * Lower design scores are better. Since the original "maximin" is a
 * maximization criterion, here we negate it so it strictly takes on negative
 * values. The "phi_p" criterion returns a positive value since it is a
 * minimization criterion. While both criteria aim to achieve a similar goal,
 * the values returned by each are not directly comparable.
 *
 * # Recommendations for different sized designs
 * To rapidly create a random Latin hypercube design of any size,
 * generateRandomDesign() is recommended. This method is fast, but does not
 * guarantee that the design is optimal.
 *
 * To rapidly create Latin hypercube designs with a small number of samples
 * (fewer than ~25) and variables (fewer than ~5),
 * generateTranslationalPropagationDesign() is recommended. This method uses a
 * deterministic algorithm that translates and propagates a "seed" design
 * throughout the design space to generate a Latin hypercube design.
 *
 * Finally, to optimize Latin hypercube designs with larger numbers of samples
 * and variable, generateStochasticEvolutionaryDesign() is recommended. This
 * approach randomly exchanges samples between columns in the design matrix and
 * accepts new designs based on an evolving warming and cooling schedule. This
 * method is slower than the other two methods, but generally leads to better
 * designs. This algorithm requires many evaluations of the design score, so
 * it is recommended to use the "phi_p" distance criterion, which approximates
 * "maximin", but is much faster.
 *
 * # References
 * - [1] Viana, F.A.C., Venter, G. and Balabanov, V. (2010), An algorithm for
 *       fast optimal Latin hypercube design of experiments. Int. J. Numer. Meth.
 *       Engng., 82: 135-156. https://doi.org/10.1002/nme.2750
 * - [2] Jin, R., Chen, W., and Sudjianto, A. (2005) An efficient algorithm for
 *       constructing optimal design of computer experiments, Journal of
 *       Statistical Planning and Inference, Volume 134, Issue 1, Pages 268-287,
 *       https://doi.org/10.1016/j.jspi.2004.02.014.
 */
class OSIMCOMMON_API LatinHypercubeDesign {
public:
    // CONSTRUCTION
    LatinHypercubeDesign();
    LatinHypercubeDesign(const LatinHypercubeDesign&);
    LatinHypercubeDesign(LatinHypercubeDesign&&);
    LatinHypercubeDesign& operator=(const LatinHypercubeDesign&);
    LatinHypercubeDesign& operator=(LatinHypercubeDesign&&);

    // GET AND SET
    /**
     * The number of samples in the Latin hypercube design (i.e., the number of
     * rows in the design matrix).
     */
    void setNumSamples(int numSamples);
    /// @copydoc setNumSamples()
    int getNumSamples() const;

    /**
     * The number of variables in the Latin hypercube design (i.e., the number
     * of columns in the design matrix).
     */
    void setNumVariables(int numVariables);
    /// @copydoc setNumVariables()
    int getNumVariables() const;

    /**
     * The criterion used to evaluate the distance between samples in the Latin
     * hypercube design ("maximin" or "phi_p").
     */
    void setDistanceCriterion(std::string distanceCriterion);
    /// @copydoc setDistanceCriterion()
    const std::string& getDistanceCriterion() const;

    /**
     * The exponent used to evaluate the "phi_p" distance criterion. This can be
     * any non-zero integer, but should be set to a relatively large value
     * (e.g., 50) to ensure that the criterion sufficiently approximates the
     * "maximin" criterion.
     */
    void setPhiPDistanceExponent(int exponent);
    /// @copydoc setPhiPDistanceExponent()
    int getPhiPDistanceExponent() const;

    // LATIN HYPERCUBE DESIGN
    /**
     * Generate a Latin hypercube design based on the best design score from
     * multiple random designs. Use the 'iterations' parameter to specify the
     * number of random designs to evaluate before selecting a design.
     */
    SimTK::Matrix generateRandomDesign(int iterations = 100) const;

    /**
     * Generate a Latin hypercube design based on the translational propagation
     * algorithm from Viana et al. (2009). Use the 'numSeedPoints' parameter to
     * specify the number of seed points to use in the algorithm. The default
     * behavior (numSeedPoints = -1) will iterate through several designs
     * starting with seeds with number of seed points between 5% and 25% of the
     * total number of samples in the design.
     */
    SimTK::Matrix generateTranslationalPropagationDesign(
            int numSeedPoints = -1) const;

    /**
     * Generate a Latin hypercube design based on the enhanced stochastic
     * evolutionary algorithm from Jin et al. (2005). Use the 'iterations'
     * parameter to specify the number of iterations to run the algorithm (i.e.,
     * the number of outer loop iterations). The number of inner loop iterations
     * and column exchanges are determined automatically based on the number of
     * samples in the design. Optionally, an initial design can be provided to
     * the algorithm using the 'initialDesign' parameter.
     */
    SimTK::Matrix generateStochasticEvolutionaryDesign(
            int iterations = 10,
            const SimTK::Matrix& initialDesign = {}) const;

    /**
     * Evaluate the distance between samples in the Latin hypercube design
     * according to the distance criterion specified by the user. The "maximin"
     * criterion returns a negative value since it is a maximization criterion,
     * while the "phi_p" criterion returns a positive value since it is a
     * minimization criterion.
     */
    double evaluateDesign(const SimTK::Matrix& design) const;

private:
    // HELPER FUNCTIONS
    /**
     * Helper function for generating Latin hypercube designs based on the
     * enhanced stochastic evolutionary algorithm.
     */
    SimTK::Matrix computeStochasticEvolutionaryDesign(
            const SimTK::Matrix& initialDesign,
            int maxColumnExchanges,
            int maxInnerIterations,
            int maxOuterIterations) const;

    /**
     * Helper function for generating Latin hypercube designs based on the
     * translational propagation algorithm.
     */
    SimTK::Matrix computeTranslationalPropagationDesign(
            SimTK::Matrix seed) const;

    /**
     * Helper functions for computing the distance between samples in a Latin
     * hypercube design.
     */
    static double computeMaximinDistanceCriterion(const SimTK::Matrix& design);
    double computePhiDistanceCriterion(const SimTK::Matrix& design) const;

    /**
     * Helper functions for computing random Latin hypercube designs.
     */
    // Returns a matrix with uniform random values between 0 and 1.
    static SimTK::Matrix computeRandomMatrix(
            int numSamples, int numVariables);

    // Compute a random valid Latin hypercube design. The design matrix is
    // populated with integer values between 1 and 'numSamples' in ascending
    // order, and then the values are randomly shuffled within each column. The
    // design values are *not* normalized.
    static SimTK::Matrix computeRandomHypercube(
            int numSamples, int numVariables);

    // Compute a random Latin hypercube design by iterating through multiple
    // designs generated by computeRandomHypercube() and selecting the design
    // with the best distance score. Before computing a score, the values in the
    // design matrix are randomly distributed by an offset between 0 and 1
    // from the nominal design values and then normalized by the number of
    // samples.
    SimTK::Matrix computeRandomDesign(
            int numSamples, int numVariables, int iterations) const;

    /**
     * Helper function to check that the current configuration of the Latin
     * hypercube design is valid.
     */
    void checkConfiguration() const;

    // MEMBER VARIABLES
    int m_numSamples = -1;
    int m_numVariables = -1;
    std::string m_distanceCriterion = "maximin";
    bool m_useMaximinDistanceCriterion = true;
    int m_phiDistanceExponent = 50;
};

} // namespace OpenSim

#endif // OPENSIM_LATINHYPERCUBEDESIGN_H
