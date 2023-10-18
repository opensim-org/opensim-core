/* -------------------------------------------------------------------------- *
*                 OpenSim:  testLatinHypercubeDesign.cpp                      *
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

#include <OpenSim/Common/LatinHypercubeDesign.h>

#define CATCH_CONFIG_MAIN
#include "OpenSim/Auxiliary/catch/catch.hpp"

using namespace OpenSim;

TEST_CASE("Translational propagation algorithm, 2-by-2 design") {
    LatinHypercubeDesign lhs;
    lhs.setNumSamples(2);
    lhs.setNumVariables(2);
    // Create design with 1 seed point.
    SimTK::Matrix design = lhs.generateTranslationalPropagationDesign(1);
    // Validate the design size.
    REQUIRE(design.nrow() == 2);
    REQUIRE(design.ncol() == 2);
    // Validate against the design values taken from the Matlab implementation
    // provided by Viana et al. (2009).
    double tol = 1e-9;
    REQUIRE_THAT(design[0][0], Catch::Matchers::WithinAbs(1.0, tol));
    REQUIRE_THAT(design[0][1], Catch::Matchers::WithinAbs(0.5, tol));
    REQUIRE_THAT(design[1][0], Catch::Matchers::WithinAbs(0.5, tol));
    REQUIRE_THAT(design[1][1], Catch::Matchers::WithinAbs(1.0, tol));
}

TEST_CASE("Translational propagation algorithm, 5-by-3 design") {
    LatinHypercubeDesign lhs;
    lhs.setNumSamples(5);
    lhs.setNumVariables(3);
    // Create design with 1 seed point.
    SimTK::Matrix design = lhs.generateTranslationalPropagationDesign(1);
    // Validate the design size.
    REQUIRE(design.nrow() == 5);
    REQUIRE(design.ncol() == 3);

    // Validate against the design values taken from the Matlab implementation
    // provided by Viana et al. (2009):
    //            __             __
    //           | 0.6   0.2   0.2 |
    //           | 0.2   0.8   0.4 |
    //  design = | 0.8   1.0   0.6 |
    //           | 0.4   0.4   0.8 |
    //           | 1.0   0.6   1.0 |
    //            ‾‾             ‾‾
    double tol = 1e-9;
    REQUIRE_THAT(design[0][0], Catch::Matchers::WithinAbs(0.6, tol));
    REQUIRE_THAT(design[0][1], Catch::Matchers::WithinAbs(0.2, tol));
    REQUIRE_THAT(design[0][2], Catch::Matchers::WithinAbs(0.2, tol));
    REQUIRE_THAT(design[1][0], Catch::Matchers::WithinAbs(0.2, tol));
    REQUIRE_THAT(design[1][1], Catch::Matchers::WithinAbs(0.8, tol));
    REQUIRE_THAT(design[1][2], Catch::Matchers::WithinAbs(0.4, tol));
    REQUIRE_THAT(design[2][0], Catch::Matchers::WithinAbs(0.8, tol));
    REQUIRE_THAT(design[2][1], Catch::Matchers::WithinAbs(1.0, tol));
    REQUIRE_THAT(design[2][2], Catch::Matchers::WithinAbs(0.6, tol));
    REQUIRE_THAT(design[3][0], Catch::Matchers::WithinAbs(0.4, tol));
    REQUIRE_THAT(design[3][1], Catch::Matchers::WithinAbs(0.4, tol));
    REQUIRE_THAT(design[3][2], Catch::Matchers::WithinAbs(0.8, tol));
    REQUIRE_THAT(design[4][0], Catch::Matchers::WithinAbs(1.0, tol));
    REQUIRE_THAT(design[4][1], Catch::Matchers::WithinAbs(0.6, tol));
    REQUIRE_THAT(design[4][2], Catch::Matchers::WithinAbs(1.0, tol));
}

TEST_CASE("Distance criterion values") {
    LatinHypercubeDesign lhs;
    lhs.setNumSamples(5);
    lhs.setNumVariables(5);
    SimTK::Matrix design = lhs.generateRandomDesign();

    // "maximin" distance criterion values should be non-positive.
    lhs.setDistanceCriterion("maximin");
    REQUIRE(lhs.evaluateDesign(design) <= 0.0);

    // "phi-p" distance criterion values should be non-negative.
    lhs.setDistanceCriterion("phi_p");
    REQUIRE(lhs.evaluateDesign(design) >= 0.0);
}

TEST_CASE("Invalid configurations") {
    LatinHypercubeDesign lhs;

    SECTION("Invalid number of samples") {
        lhs.setNumSamples(-1);
        lhs.setNumVariables(5);
        REQUIRE_THROWS_WITH(lhs.generateRandomDesign(),
                Catch::Contains("Expected the number of samples"));
    }

    SECTION("Invalid number of variables") {
        lhs.setNumVariables(-1);
        lhs.setNumSamples(5);
        REQUIRE_THROWS_WITH(lhs.generateRandomDesign(),
                Catch::Contains("Expected the number of variables"));
    }

    SECTION("Invalid distance criterion") {
        lhs.setNumSamples(5);
        lhs.setNumVariables(5);
        lhs.setDistanceCriterion("invalid");
        REQUIRE_THROWS_WITH(lhs.generateRandomDesign(),
            Catch::Contains("Invalid distance criterion"));
    }

    SECTION("Invalid 'phi-p' exponent") {
        lhs.setNumSamples(5);
        lhs.setNumVariables(5);
        lhs.setDistanceCriterion("phi_p");
        lhs.setPhiPDistanceExponent(-1);
        REQUIRE_THROWS_WITH(lhs.generateRandomDesign(),
                Catch::Contains("Expected the 'phi-p' distance exponent"));
    }
}
