/* -------------------------------------------------------------------------- *
*                         OpenSim:  testArray.cpp                            *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2024 Stanford University and the Authors                *
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

#include <OpenSim/Common/Array.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;

constexpr double tol = std::numeric_limits<double>::epsilon() * 10;

namespace
{
    template<typename T>
    int SearchBinaryLegacyImplementation(const Array<T> ary, const T& aValue, int aLo=-1, int aHi = -1)
    {
        if(ary.size()<=0) return(-1);
        int lo = aLo;  if(lo<0) lo = 0;
        int hi = aHi;  if((hi<0)||(hi>=ary.size())) hi = ary.size() - 1;
        int mid = -1;

        // CHECK lo AND hi
        if(lo>hi) return(-1);

        // SEARCH
        while(lo <= hi) {
            mid = (lo + hi) / 2;
            if(aValue < ary[mid]) {
                hi = mid - 1;
            } else if(ary[mid] < aValue) {
                lo = mid + 1;
            } else {
                break;
            }
        }

        // MAKE SURE LESS THAN
        if(aValue < ary[mid]) mid--;
        if(mid<=0) {
            return(mid);
        }

        // FIND FIRST
        bool aFindFirst = true;  // always true in newer implementation
        if(aFindFirst) {
            if(ary[mid-1]<ary[mid]) {
                return(mid);
            }
            lo = aLo;  if(lo<0) lo = 0;
            hi = mid;
            int mid2 = mid;
            T value2 = ary[mid];
            while(lo <= hi) {
                mid2 = (lo + hi) / 2;
                if(ary[mid2] == value2) {
                    hi = mid2 - 1;
                } else if(ary[mid2] < value2) {
                    lo = mid2 + 1;
                }
            }
            if(ary[mid2]<value2) mid2++;
            if(mid2<mid) mid = mid2;
        }

        return(mid);
    }
}

TEST_CASE("Array searchBinary Behaves Similarly to Legacy Implementation")
{
    constexpr int arraySize = 16;

    // create sorted range
    Array<int> vals;
    for (int i = -arraySize; i < arraySize; i+=2) {
        vals.append(i);
    }

    // probe sorted range with rng and compare
    for (int probe = -arraySize-1; probe < arraySize+1; ++probe) {
        int legacyOutput = SearchBinaryLegacyImplementation(vals, probe);
        int newOutput = vals.searchBinary(probe);
        INFO("index = " << probe << ", legacyOutput = " << legacyOutput << ", newOutput = " << newOutput);
        REQUIRE(legacyOutput == newOutput);
    }
}

TEST_CASE("Array rFindIndex returns expected results")
{
    Array<int> vals;
    vals.append(0);
    vals.append(1);
    vals.append(2);
    vals.append(2);
    vals.append(3);

    REQUIRE(vals.rfindIndex(3) == 4);
    REQUIRE(vals.rfindIndex(2) == 3);
    REQUIRE(vals.findIndex(2) == 2);  // sanity check for duplicate case
    REQUIRE(vals.rfindIndex(1) == 1);
    REQUIRE(vals.rfindIndex(0) == 0);
    REQUIRE(vals.rfindIndex(1337) == -1);
}

TEST_CASE("Array set keeps size if within bounds")
{
    Array<int> vals;
    vals.append(0);

    REQUIRE(vals.size() == 1);
    vals.set(0, 4);
    REQUIRE(vals.size() == 1);
    REQUIRE(vals.get(0) == 4);  
}

TEST_CASE("Array initializer list works with integers")
{
    Array<int> vals{-1, 2, 4};

    REQUIRE(vals.get(0) == -1);
    REQUIRE(vals.get(1) == 2);
    REQUIRE(vals.get(2) == 4);
}

TEST_CASE("Array constructor initialization works")
{
    Array<int> vals(-1, 2);

    REQUIRE(vals.get(0) == -1);
    REQUIRE(vals.get(1) == -1);
}

TEST_CASE("Array initializer list works with doubles")
{
    Array<double> vals{-1.5, 2.2, 4.6};

    REQUIRE_THAT(vals.get(0), Catch::Matchers::WithinAbs(-1.5,tol));
    REQUIRE_THAT(vals.get(1), Catch::Matchers::WithinAbs(2.2,tol));
    REQUIRE_THAT(vals.get(2), Catch::Matchers::WithinAbs(4.6,tol));
}

TEST_CASE("Array initializer list works with doubles and alternate syntax")
{
    Array<double> vals = {-1.5, 2.2, 4.6};

    REQUIRE_THAT(vals.get(0), Catch::Matchers::WithinAbs(-1.5,tol));
    REQUIRE_THAT(vals.get(1), Catch::Matchers::WithinAbs(2.2,tol));
    REQUIRE_THAT(vals.get(2), Catch::Matchers::WithinAbs(4.6,tol));
}

TEST_CASE("Array initializer list works with booleans")
{
    Array<bool> vals{true, false, true};

    REQUIRE(vals.get(0) == true);
    REQUIRE(vals.get(1) == false);
    REQUIRE(vals.get(2) == true);
}