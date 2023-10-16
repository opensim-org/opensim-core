/* -------------------------------------------------------------------------- *
*                      OpenSim:  LatinHypercubeDesign.cpp                     *
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

#include "LatinHypercubeDesign.h"

#include "Assertion.h"
#include "Exception.h"
#include <numeric>
#include <random>

#include <SimTKcommon/internal/Random.h>
#include <SimTKcommon/internal/VectorMath.h>

#include <OpenSim/Common/Logger.h>

using namespace OpenSim;

static const std::vector<std::string> ValidDistanceCriteria =
        {"maximin", "phi_p"};

//=============================================================================
// CONSTRUCTION
//=============================================================================
LatinHypercubeDesign::LatinHypercubeDesign() = default;

LatinHypercubeDesign::LatinHypercubeDesign(
        const LatinHypercubeDesign&) = default;

LatinHypercubeDesign::LatinHypercubeDesign(
        LatinHypercubeDesign&&) = default;

LatinHypercubeDesign& LatinHypercubeDesign::operator=(
        const LatinHypercubeDesign&) = default;

LatinHypercubeDesign& LatinHypercubeDesign::operator=(
        LatinHypercubeDesign&&) = default;

//=============================================================================
// GET AND SET
//=============================================================================
void LatinHypercubeDesign::setNumSamples(int numSamples) {
    m_numSamples = numSamples;
}

int LatinHypercubeDesign::getNumSamples() const {
    return m_numSamples;
}

void LatinHypercubeDesign::setNumVariables(int numVariables) {
    m_numVariables = numVariables;
}

int LatinHypercubeDesign::getNumVariables() const {
    return m_numVariables;
}

void LatinHypercubeDesign::setDistanceCriterion(std::string distanceCriterion) {
    m_distanceCriterion = std::move(distanceCriterion);
    m_useMaximinDistanceCriterion = m_distanceCriterion == "maximin";
}

const std::string& LatinHypercubeDesign::getDistanceCriterion() const {
    return m_distanceCriterion;
}

void LatinHypercubeDesign::setPhiPDistanceExponent(int exponent) {
    m_phiDistanceExponent = exponent;
}

int LatinHypercubeDesign::getPhiPDistanceExponent() const {
    return m_phiDistanceExponent;
}

//=============================================================================
// LATIN HYPERCUBE DESIGN
//=============================================================================
SimTK::Matrix LatinHypercubeDesign::generateRandomDesign(int iterations) const {
    checkConfiguration();
    log_info("Generating a random Latin hypercube design with {} "
             "iterations...", iterations);
    SimTK::Matrix design = computeRandomDesign(m_numSamples, m_numVariables,
            iterations);
    log_info("The final design has score {} using the '{}' distance "
             "criterion (lower is better).",
            evaluateDesign(design), m_distanceCriterion);
    return design;
}

SimTK::Matrix LatinHypercubeDesign::generateTranslationalPropagationDesign(
        int numSeedPoints) const {
    checkConfiguration();

    SimTK::Matrix design(m_numSamples, m_numVariables);
    if (numSeedPoints == -1) {
        int minPts = (int)std::ceil(0.05 * m_numSamples);
        int maxPts = (int)std::ceil(0.25 * m_numSamples);
        int deltaPts = (int)std::ceil(0.05 * m_numSamples);
        log_info("Generating Latin hypercube designs using the "
                 "translational propagation algorithm between {} "
                 "and {} seed points...", minPts, maxPts);

        SimTK::Matrix currentDesign;
        double currentScore, score = SimTK::Infinity;
        for (int pts = minPts; pts <= maxPts; pts += deltaPts) {
            SimTK::Matrix seed = computeRandomDesign(pts, m_numVariables, 25);
            seed *= pts;
            currentDesign = computeTranslationalPropagationDesign(seed);
            currentScore = evaluateDesign(currentDesign);
            if (currentScore < score) {
                design = currentDesign;
                score = currentScore;
            }
            log_info("Score for {} seed points = {}", pts, currentScore);
        }
    } else {
        OPENSIM_THROW_IF(numSeedPoints < 1, Exception,
                "The number of seed points must be greater than zero.")
        log_info("Generating a Latin hypercube design using the "
                 "translational propagation algorithm with {} seed points...",
                numSeedPoints);
        SimTK::Matrix seed = computeRandomDesign(
                numSeedPoints, m_numVariables, 25);
        seed *= numSeedPoints;
        design = computeTranslationalPropagationDesign(seed);
    }

    log_info("The final design has score {} using the '{}' distance "
             "criterion (lower is better).",
            evaluateDesign(design), m_distanceCriterion);
    return design;
}

SimTK::Matrix LatinHypercubeDesign::generateStochasticEvolutionaryDesign(
        int iterations, const SimTK::Matrix& initialDesign) const {
    checkConfiguration();
    log_info("Generating a Latin hypercube design using the enhanced "
             "stochastic evolutionary algorithm with {} iterations...",
            iterations);

    // A function the smoothly transitions between two values.
    auto smoothBoundedFunction = [](double x, double min, double max,
                                         double transition) {
        return min + (max - min) * (1 - std::exp(-transition * x)) /
                             (1 + std::exp(-transition * x));
    };

    // The maximum number of column exchanges (50) and inner iterations (100)
    // are set based on the Jin et al. (2005) paper. We arbitrarily set the
    // minimum number of column exchanges and inner iterations to 10% of their
    // maximum values.
    int numColumnExchanges = (int)smoothBoundedFunction(
            m_numSamples, 5, 50, 0.01);
    int numInnerIterations = (int)smoothBoundedFunction(
            m_numSamples, 10, 100, 0.005);
    log_info("Number of outer iterations = {} (provided by user)",
            iterations);
    log_info("Number of column exchanges = {} (computed based on the "
             "number of samples)", numColumnExchanges);
    log_info("Number of inner iterations = {} (computed based on the "
             "number of samples)", numInnerIterations);

    SimTK::Matrix initialDesignToUse(m_numSamples, m_numVariables);
    if (!initialDesign.nrow()) {
        initialDesignToUse =
                computeRandomDesign(m_numSamples, m_numVariables, 100);
    } else {
        OPENSIM_THROW_IF(initialDesign.nrow() != m_numSamples, Exception,
                "Expected the initial design to have {} rows, but received {}.",
                m_numSamples, initialDesign.nrow())
        OPENSIM_THROW_IF(initialDesign.ncol() != m_numVariables, Exception,
                "Expected the initial design to have {} columns, but received "
                "{}.",
                m_numVariables, initialDesign.ncol())
        initialDesignToUse = initialDesign;
    }
    log_info("Initial design score = {}",
            evaluateDesign(initialDesignToUse));

    // Perform the stochastic evolutionary algorithm.
    SimTK::Matrix design = computeStochasticEvolutionaryDesign(
            initialDesignToUse, numColumnExchanges,
            numInnerIterations, iterations);

    log_info("The final design has score {} using the '{}' distance "
                 "criterion (lower is better).",
            evaluateDesign(design), m_distanceCriterion);
    return design;
}

double LatinHypercubeDesign::evaluateDesign(
        const SimTK::Matrix& design) const {
    if (m_useMaximinDistanceCriterion) {
        return computeMaximinDistanceCriterion(design);
    } else {
        return computePhiDistanceCriterion(design);
    }
}

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================
double LatinHypercubeDesign::computeMaximinDistanceCriterion(
        const SimTK::Matrix& design) {
   SimTK::Matrix neighbors = computeKNearestNeighbors(design, design, 2);
   return -SimTK::min(neighbors.col(1));
}

double LatinHypercubeDesign::computePhiDistanceCriterion(
        const SimTK::Matrix& design) const {
    double sumInverseDistancesP = 0;
    const int numRows = design.nrow();
    for (int i = 0; i < numRows - 1; ++i) {
        const SimTK::RowVectorView row_i = design.row(i);
        for (int j = i + 1; j < numRows; ++j) {
            const SimTK::RowVectorView row_j = design.row(j);
            double distance = (row_j - row_i).abs().sum();
            sumInverseDistancesP += pow(1.0 / distance,
                    m_phiDistanceExponent);
        }
    }
    return pow(sumInverseDistancesP, 1.0 / m_phiDistanceExponent);
}

SimTK::Matrix LatinHypercubeDesign::computeTranslationalPropagationDesign(
        SimTK::Matrix seed) const {

    // Sorting helper function.
    // ------------------------
    // stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
    auto sort_indexes = [](const SimTK::Vector& v) -> std::vector<int> {
       std::vector<int> idx(v.size());
       std::iota(idx.begin(), idx.end(), 0);
       std::stable_sort(idx.begin(), idx.end(),
               [&v](int i1, int i2) {return v[i1] < v[i2];});
       return idx;
    };

    // Define the size of the TPLHD to be created first.
    // -------------------------------------------------
    int numSeedPoints = (int)seed.nrow();
    OPENSIM_THROW_IF(numSeedPoints > m_numSamples, Exception,
            "Expected the number of seed points to be less than or equal to "
            "the number of samples, but received {} seed points and {} "
            "samples.", numSeedPoints, m_numSamples)

    int numVariables = (int)seed.ncol();
    double numDivisions =
           std::pow(m_numSamples / numSeedPoints, 1.0 / numVariables);
    int numDivisionsRounded = (int)std::ceil(numDivisions);
    int numBlocks;
    if (numDivisionsRounded > numDivisions) {
        numBlocks = (int)std::pow(numDivisionsRounded, numVariables);
    } else {
       OPENSIM_ASSERT_ALWAYS(m_numSamples % numSeedPoints == 0);
       numBlocks = m_numSamples / numSeedPoints;
    }
    int numSamplesRounded = numBlocks * numSeedPoints;

    // Reshape the seed to create the first design.
    // --------------------------------------------
    if (numSeedPoints == 1) {
       seed = SimTK::Matrix(1, numVariables, 1.0);
    } else {
       SimTK::RowVector uf(numVariables, numSeedPoints);
       SimTK::RowVector ut(numVariables,
               ((double)numSamplesRounded / numDivisionsRounded) -
                       numDivisionsRounded * (numVariables - 1) + 1);
       SimTK::RowVector a = (ut - 1).elementwiseDivide(uf - 1);
       SimTK::RowVector b = ut - a.elementwiseMultiply(uf);
       for (int i = 0; i < numSeedPoints; ++i) {
           seed.updRow(i).elementwiseMultiplyInPlace(a);
           seed.updRow(i) += b;

           // Round all the elements of the seed to the nearest integer.
           for (int j = 0; j < numVariables; ++j) {
               seed[i][j] = std::round(seed[i][j]);
           }
       }
    }

    // If the seed already has the desired number of samples, return it as the
    // final design.
    if (seed.nrow() == m_numSamples) {
       seed /= m_numSamples;
       return seed;
    }

    // Create the TPLHD.
    // -----------------
    SimTK::Matrix tplhd(numSamplesRounded, numVariables);

    // Fill in the first seed.
    for (int i = 0; i < numSeedPoints; ++i) {
       tplhd.updRow(i) = seed.row(i);
    }

    SimTK::RowVector seedUpdate(numVariables, 1.0);
    for (int i = 0; i < numVariables; ++i) {
       // Update the seed with the most recently added points.
       seed = tplhd.block(0, 0, numSeedPoints, numVariables);

       // Prepare the seed update.
       for (int j = 0; j < i; ++j) {
           seedUpdate[j] = std::pow(numDivisionsRounded, i - 1);
       }
       seedUpdate[i] = (double)numSamplesRounded / numDivisionsRounded;
       for (int j = i + 1; j < numVariables; ++j) {
           seedUpdate[j] = std::pow(numDivisionsRounded, i);
       }

       // Fill in each of the divisions.
       for (int j = 1; j < numDivisionsRounded; ++j) {

           // Update the seed.
           for (int k = 0; k < (int)seed.nrow(); ++k) {
               seed.updRow(k) += seedUpdate;
           }

           // Update the TPLHD.
           tplhd.updBlock(
                   numSeedPoints, 0, (int)seed.nrow(), numVariables) = seed;
           numSeedPoints += (int)seed.nrow();
       }
    }

    // If necessary, resize the TPLHD.
    // -------------------------------
    if (numSamplesRounded > m_numSamples) {
       SimTK::Matrix tplhdResized(m_numSamples, numVariables);

       // Center the design space.
       SimTK::RowVector center(numVariables, 0.5*numSamplesRounded);

       // Compute the distance between each point and the center.
       SimTK::Vector distance(numSamplesRounded, 0.0);
       for (int i = 0; i < numSamplesRounded; ++i) {
           distance.set(i, (tplhd.row(i) - center).norm());
       }

       // Resize the TPLHD by taking the 'numSamples' points closest to the
       // center.
       int isample = 0;
       for (int irow : sort_indexes(distance)) {
           if (isample >= m_numSamples) {
               break;
           }
           tplhdResized.updRow(isample) = tplhd.row(irow);
           ++isample;
       }

       // Re-establish the Latin hypercube conditions.
       SimTK::RowVector tplhdMinimums = SimTK::min(tplhdResized);
       for (int ivar = 0; ivar < numVariables; ++ivar) {

           // Place the current design at the origin.
           std::vector<int> sortedColumnIndexes =
                   sort_indexes(tplhdResized.col(ivar));
           SimTK::Matrix tplhdTemp = tplhdResized;
           for (int irow = 0; irow < m_numSamples; ++irow) {
               tplhdResized.updRow(irow) =
                       tplhdTemp.row(sortedColumnIndexes[irow]);
           }
           tplhdResized.updCol(ivar) -= tplhdMinimums[ivar];
           tplhdResized.updCol(ivar) += 1;

           // Eliminate empty coordinates.
           for (int irow = 0; irow < m_numSamples; ++irow) {
               if (tplhdResized[irow][ivar] != irow + 1) {
                   tplhdResized[irow][ivar] = irow + 1;
               }
           }
       }
       tplhd = tplhdResized;
    }

    tplhd /= m_numSamples;
    return tplhd;
}

SimTK::Matrix LatinHypercubeDesign::computeStochasticEvolutionaryDesign(
        const SimTK::Matrix& initialDesign, int numColumnExchanges,
        int numInnerIterations, int numOuterIterations) const {

    // Validate inputs.
    // ----------------
    OPENSIM_THROW_IF(numOuterIterations < 1, Exception,
            "Expected the number of outer loop iterations must be greater than "
            "zero, but received {}.", numOuterIterations)
    OPENSIM_THROW_IF(numInnerIterations < 1, Exception,
            "Expected the number of inner loop iterations must be greater than "
            "zero, but received {}.", numInnerIterations)
    OPENSIM_THROW_IF(numColumnExchanges < 1, Exception,
            "Expected the number of column exchanges must be greater than "
            "zero, but received {}.", numColumnExchanges)
    int numSamples = initialDesign.nrow();
    int numVariables = initialDesign.ncol();

    // Initialize the algorithm parameters.
    // ------------------------------------
    // Initialize the design and their initial distance criterion values.
    SimTK::Matrix designCurrent = initialDesign;
    SimTK::Matrix designBest = initialDesign;
    double distanceCurrent = evaluateDesign(designCurrent);
    double distanceBest = distanceCurrent;
    double distanceOldBest;

    // Initialize the initial threshold and the warming/cooling schedule
    // parameters.
    double threshold = 5e-3 * std::abs(evaluateDesign(designCurrent));
    double alpha1 = 0.8;
    double alpha2 = 0.9;
    double alpha3 = 0.7;
    SimTK::Random::Uniform random;

    // Perform the stochastic evolutionary algorithm.
    // ----------------------------------------------
    SimTK::Matrix randomMatrix(2*numColumnExchanges, numInnerIterations);
    int numAccepted, numImproved, inner, outer = 0;
    bool improving;
    double acceptanceRatio, improvementRatio;
    while (outer < numOuterIterations) {
        // Store the current best design criterion value.
        distanceOldBest = distanceBest;

        // Pre-compute values for the random row exchanges.
        for (int i = 0; i < 2*numColumnExchanges; ++i) {
           for (int j = 0; j < numInnerIterations; ++j) {
               randomMatrix[i][j] = random.getValue();
           }
        }

        // Perform the inner loop.
        inner = 0;
        numAccepted = 0;
        numImproved = 0;
        while (inner < numInnerIterations) {
            // Select the column to exchange variables in.
            int col = inner % numVariables;

            // Iterate through random column exchanges.
            SimTK::Matrix designTry = designCurrent;
            double distanceTry = distanceCurrent;
            double temp;
            for (int i = 0; i < numColumnExchanges; ++i) {
                // Select two random rows for the element exchange.
                int row1 = (int)std::floor(
                        randomMatrix[2*i][inner] * numSamples);
                int row2 = (int)std::floor(
                        randomMatrix[2*i-1][inner] * numSamples);

                // Exchange the elements in the current column for each row.
                temp = designCurrent[row1][col];
                designCurrent.set(row1, col, designCurrent[row2][col]);
                designCurrent.set(row2, col, temp);

                // Check if the exchange is an improvement.
                double newDistance = evaluateDesign(designCurrent);
                if (newDistance < distanceTry) {
                    designTry = designCurrent;
                    distanceTry = newDistance;
                }

                // Undo the element exchange in the current.
                designCurrent.set(row2, col, designCurrent[row1][col]);
                designCurrent.set(row1, col, temp);
            }

            // Check for acceptance.
            if ((distanceTry - distanceCurrent) <=
                    threshold * random.getValue()) {
                designCurrent = designTry;
                distanceCurrent = distanceTry;
                ++numAccepted;

                // Check for improvement.
                if (distanceTry < distanceBest) {
                    designBest = designCurrent;
                    distanceBest = distanceTry;
                    ++numImproved;
                }
            }

            ++inner;
        }

        // Check for improvement.
        improving = (distanceOldBest - distanceBest) > threshold;

        // Compute the acceptance and improvement ratios.
        acceptanceRatio = (double)numAccepted / numInnerIterations;
        improvementRatio = (double)numImproved / numInnerIterations;

        // Update the threshold parameter.
        if (improving) {
            if (acceptanceRatio > 0.1) {
               if (improvementRatio < acceptanceRatio) {
                   threshold *= alpha1;
               }
            } else {
               threshold /= alpha1;
            }
        } else {
            if (acceptanceRatio < 0.1) {
                threshold /= alpha3;
            } else if (acceptanceRatio > 0.8) {
                threshold *= alpha2;
            }
        }

        ++outer;
        log_info("Iteration {}/{} score = {}", outer, numOuterIterations,
                 distanceBest);
    }

    return designBest;
}

SimTK::Matrix LatinHypercubeDesign::computeRandomMatrix(int numSamples,
        int numVariables) {
    SimTK::Random::Uniform random(0, 1);
    SimTK::Matrix matrix(numSamples, numVariables);
    for (int i = 0; i < numSamples; ++i) {
        for (int j = 0; j < numVariables; ++j) {
            matrix[i][j] = random.getValue();
        }
    }
    return matrix;
}

SimTK::Matrix LatinHypercubeDesign::computeRandomHypercube(
        int numSamples, int numVariables) {
    SimTK::Matrix hypercube(numSamples, numVariables);

    for (int icol = 0; icol < numVariables; ++icol) {
        // Creating a starting column of integers from 1 to numSamples. Without
        // randomization, this creates a valid hypercube with elements along the
        // diagonal (in the general sense) of the hypercube space.
        std::vector<int> column;
        column.reserve(numSamples);
        for (int irow = 0; irow < numSamples; ++irow) {
            column.push_back(irow + 1);
        }

        // Randomly permute the column.
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(column.begin(), column.end(), gen);

        // Fill in the column of the hypercube.
        for (int irow = 0; irow < numSamples; ++irow) {
            hypercube[irow][icol] = column[irow];
        }
    }

    return hypercube;
}

SimTK::Matrix LatinHypercubeDesign::computeRandomDesign(
        int numSamples, int numVariables, int iterations) const {
    // Iterate through the number of random designs and save the best one.
    SimTK::Matrix design;
    SimTK::Matrix designBest;
    double distanceBest = SimTK::Infinity;
    for (int i = 0; i < iterations; ++i) {
        // Generate a random hypercube and normalize it.
        design = computeRandomHypercube(numSamples, numVariables);
        design -= computeRandomMatrix(numSamples, numVariables);
        design /= numSamples;

        // Compute the distance criterion and save the best design.
        double distance = evaluateDesign(design);
        if (distance < distanceBest) {
            designBest = design;
            distanceBest = distance;
        }
    }

    return designBest;
}

void LatinHypercubeDesign::checkConfiguration() const {
    OPENSIM_THROW_IF(m_numSamples < 1, Exception,
            "Expected the number of samples to be greater than zero, but "
            "received {}.", m_numSamples)
    OPENSIM_THROW_IF(m_numVariables < 1, Exception,
            "Expected the number of variables to be greater than zero, but "
            "received {}.", m_numVariables)

    bool distanceCriterionIsValid =
            std::find(ValidDistanceCriteria.begin(),
                    ValidDistanceCriteria.end(),
                    m_distanceCriterion) ==
            ValidDistanceCriteria.end();
    OPENSIM_THROW_IF(distanceCriterionIsValid, Exception,
            "Invalid distance criterion. You must choose be one of the "
            "following: 'maximin', 'phi_p'.")

    if (!m_useMaximinDistanceCriterion) {
        OPENSIM_THROW_IF(m_phiDistanceExponent < 1, Exception,
                "Expected the 'phi-p' distance exponent to be greater than "
                "zero, but received {}.", m_phiDistanceExponent)
    }

    log_info("LatinHypercubeDesign");
    log_info("--------------------");
}
