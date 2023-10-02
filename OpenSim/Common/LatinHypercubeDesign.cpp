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
#include <numeric>

#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/VectorMath.h>

using namespace OpenSim;

double LatinHypercubeDesign::computeMaximinDistance(
        const SimTK::Matrix& x) {
    SimTK::Matrix nearestNeighbors = computeKNearestNeighbors(x, x, 2);
    return SimTK::min(nearestNeighbors.col(1));
}

double LatinHypercubeDesign::computePhiPDistanceCriterion(
        const SimTK::Matrix& x, const SimTK::Matrix& distances, int p) {

    double sumInverseDistancesP = 0;
    for (int i = 0; i < x.nrow(); ++i) {
        for (int j = 0; j < x.nrow(); ++j) {
            if (i < j) {
                sumInverseDistancesP += 1.0 / pow(distances(i, j), p);
            }
        }
    }

    return pow(sumInverseDistancesP, 1.0 / p);
}

SimTK::Matrix LatinHypercubeDesign::computeIntersiteDistanceMatrix(
        const SimTK::Matrix& x) {

    // Compute a matrix of distance values, where each element is the distance
    // between the corresponding rows in 'x' and 'y'. The elements should be
    // d_ij where 1 <= i, j <= n, i != j, where n is the number of rows in
    // 'x' and 'y'.
    SimTK::Matrix distances(x.nrow(), x.nrow(), 0.0);
    for (int i = 0; i < x.nrow(); ++i) {
        for (int j = 0; j < x.nrow(); ++j) {
            if (i != j) {
                distances(i, j) = (x.row(i) - x.row(j)).normSqr();
            }
        }
    }

    return distances;
}

SimTK::Matrix LatinHypercubeDesign::computeTranslationalPropagationDesign(
        int numSamples, int numVariables, SimTK::Matrix seed, int numSeedPoints)
{
    // Sorting helper functions.
    // -------------------------
    // https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
    auto sort_indexes = [](const std::vector<double>& v) -> std::vector<size_t> {
        std::vector<size_t> idx(v.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::stable_sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
        return idx;
    };

    // Same as above, but for SimTK::Vector.
    auto sort_indexes_simtk = [](const SimTK::Vector& v) -> std::vector<size_t> {
        std::vector<size_t> idx(v.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::stable_sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
        return idx;
    };

    // Define the size of the TPLHD to be created first.
    // ------------------------------------------------
    double numDivisions =
            std::pow(numSamples / numSeedPoints, 1.0 / numVariables);
    int numDivisionsRounded = (int)std::ceil(numDivisions);
    int numSeeds;
    if (numDivisionsRounded > numDivisions) {
        numSeeds = (int)std::pow(numDivisionsRounded, numVariables);
    } else {
        OPENSIM_ASSERT_ALWAYS(numSamples % numSeedPoints == 0);
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
        std::cout << "seed: " << seed << std::endl;

        // Prepare the seed update.
        for (int j = 0; j < i; ++j) {
            seedUpdate[j] = std::pow(numDivisionsRounded, i - 1);
        }
        seedUpdate[i] = numSamplesRounded / numDivisionsRounded;
        for (int j = i + 1; j < numVariables; ++j) {
            seedUpdate[j] = std::pow(numDivisionsRounded, i);
        }

        // Fill in each of the divisions.
        int numNewSeedPoints = 0;
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

    std::cout << "tplhd: " << tplhd << std::endl;

    // If necessary, resize the TPLHD.
    // -------------------------------
    if (numSamplesRounded > numSamples) {
        SimTK::Matrix tplhdResized(numSamples, numVariables);

        // Center the design space.
        SimTK::RowVector center(numVariables, 0.5*numSamplesRounded);

        // Compute the distance between each point and the center.
        std::vector<double> distance;
        distance.reserve(numSamplesRounded);
        for (int i = 0; i < numSamplesRounded; ++i) {
            distance.push_back((tplhd.row(i) - center).norm());
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
            std::vector<size_t> sortedColumnIndexes =
                    sort_indexes_simtk(tplhdResized.col(i));
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