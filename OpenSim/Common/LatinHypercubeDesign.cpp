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
#include "Exception.h"
#include <numeric>

#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/VectorMath.h>

using namespace OpenSim;

static const std::vector<std::string> ValidDistanceCriteria =
        {"maximin", "phi_p"};

void LatinHypercubeDesign::checkConfiguration() const {
    OPENSIM_THROW_IF(m_numSamples < 1, Exception,
            "The number of samples must be greater than zero.");
    OPENSIM_THROW_IF(m_numVariables < 1, Exception,
            "The number of variables must be greater than zero.");

    bool distanceCriterionIsValid =
            std::find(ValidDistanceCriteria.begin(),
                      ValidDistanceCriteria.end(),
                      m_distanceCriterion) ==
            ValidDistanceCriteria.end();
    OPENSIM_THROW_IF(distanceCriterionIsValid, Exception,
            "Invalid distance criterion. You must choose be one of the "
            "following: 'maximin', 'phi_p'.");
}

SimTK::Matrix LatinHypercubeDesign::generateTranslationalPropagationDesign(
        int numSeedPoints) const {
    checkConfiguration();

    if (numSeedPoints != -1) {
        // Generate a seed with 'numSeedPoints' based on a seed with a single
        // point.
        OPENSIM_THROW_IF(numSeedPoints > m_numSamples, Exception,
                "The number of seed points must be less than or equal to the "
                "number of samples.")
        SimTK::Matrix seed = computeTranslationalPropagationDesign(numSeedPoints,
                SimTK::Matrix(1, m_numVariables, 1.0));
        return computeTranslationalPropagationDesign(m_numSamples, seed);

    } else {
        // If the number of seed points was not specified, then iterate through
        // several seed designs and choose the one with the best distance
        // criterion. We arbitrarily choose the maximum number of seed points
        // to be 10% of the number of samples.
        int maxNumSeedPoints = (int)std::ceil(m_numSamples / 10);
        SimTK::Matrix bestDesign;
        double bestDistance = SimTK::Infinity;
        int nSeedPoints = 1;
        while (nSeedPoints <= maxNumSeedPoints) {
            // Create the seed.
            SimTK::Matrix seed =
                    computeTranslationalPropagationDesign(nSeedPoints,
                            SimTK::Matrix(1, m_numVariables, 1.0));

            // Create the Latin hypercube design.
            SimTK::Matrix design = computeTranslationalPropagationDesign(
                    m_numSamples, seed);

            // Compute the distance criterion.
            double distance = computeDistanceCriterion(design);
            if (distance < bestDistance) {
                bestDesign = design;
                bestDistance = distance;
            }
            ++nSeedPoints;
        }

        return bestDesign;
    }
}

double LatinHypercubeDesign::computeDistanceCriterion(
        const SimTK::Matrix& design) const {
    if (m_useMaximinDistanceCriterion) {
        return computeMaximinDistanceCriterion(design);
    } else {
        return computePhiDistanceCriterion(design);
    }
}

double LatinHypercubeDesign::computeMaximinDistanceCriterion(
        const SimTK::Matrix& design) const {
    SimTK::Matrix nearestNeighbors =
            computeKNearestNeighbors(design, design, 2);
    return -SimTK::min(nearestNeighbors.col(1));
}

double LatinHypercubeDesign::computePhiDistanceCriterion(
        const SimTK::Matrix& design) const {
    double sumInverseDistancesP = 0;
    for (int i = 0; i < design.nrow(); ++i) {
        for (int j = 0; j < design.nrow(); ++j) {
            if (i < j) {
                double distance = (design.row(j) - design.row(i)).norm();
                sumInverseDistancesP +=
                        1.0 / pow(distance, m_phiDistanceExponent);
            }
        }
    }

    return pow(sumInverseDistancesP, 1.0 / m_phiDistanceExponent);
}

SimTK::Matrix LatinHypercubeDesign::computeTranslationalPropagationDesign(
        int numSamples, SimTK::Matrix seed) const {

    // Sorting helper function.
    // ------------------------
    // https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
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
    int numVariables = (int)seed.ncol();
    double numDivisions =
            std::pow(getNumSamples() / numSeedPoints, 1.0 / getNumVariables());
    int numDivisionsRounded = (int)std::ceil(numDivisions);
    int numSeeds;
    if (numDivisionsRounded > numDivisions) {
        numSeeds = (int)std::pow(numDivisionsRounded, numVariables);
    } else {
        OPENSIM_ASSERT_ALWAYS(getNumSamples() % numSeedPoints == 0);
        numSeeds = numSamples / numSeedPoints;
    }
    int numSamplesRounded = numSeeds * numSeedPoints;

    // Reshape the seed to create the first design.
    // --------------------------------------------
    if (numSeedPoints == 1) {
        seed = SimTK::Matrix(1, numVariables, 1.0);
    } else {
        SimTK::RowVector uf(numVariables, numSeedPoints);
        SimTK::RowVector ut(numVariables,
                (numSamplesRounded / numDivisionsRounded) -
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
        seedUpdate[i] = numSamplesRounded / numDivisionsRounded;
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
            tplhd.updBlock(numSeedPoints, 0, (int)seed.nrow(), numVariables) =
                    seed;
            numSeedPoints += (int)seed.nrow();
        }
    }

    // If necessary, resize the TPLHD.
    // -------------------------------
    if (numSamplesRounded > numSamples) {
        SimTK::Matrix tplhdResized(numSamples, numVariables);

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
        for (auto i : sort_indexes(distance)) {
            if (isample >= numSamples) {
                break;
            }
            tplhdResized.updRow(isample) = tplhd.row(i);
            ++isample;
        }

        // Re-establish the Latin hypercube conditions.
        SimTK::RowVector tplhdMinimums = SimTK::min(tplhdResized);
        for (int i = 0; i < numVariables; ++i) {

            // Place the current design at the origin.
            std::vector<int> sortedColumnIndexes =
                    sort_indexes(tplhdResized.col(i));
            SimTK::Matrix tplhdTemp = tplhdResized;
            for (int j = 0; j < numSamples; ++j) {
                tplhdResized.updRow(j) = tplhdTemp.row(sortedColumnIndexes[j]);
            }
            tplhdResized.updCol(i) -= tplhdMinimums[i];
            tplhdResized.updCol(i) += 1;

            // Eliminate empty coordinates.
            bool flag = false;
            while (!flag) {
                std::vector<bool> mask;
                mask.reserve(numSamples);
                SimTK::Vector maskVec(numSamples, 0.0);
                for (int j = 0; j < numSamples; ++j) {
                    mask.push_back(tplhdResized(j, i) != j+1);
                    maskVec[j] = mask.back();
                }

                // If all elements in mask are false, then set flag to true.
                flag = std::all_of(mask.begin(), mask.end(),
                        [](bool b) {return !b;});

                tplhdResized.updCol(i) -= maskVec;
            }
        }
        return tplhdResized;
    } else {
        return tplhd;
    }
}
