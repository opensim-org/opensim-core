/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WrapResult.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "WrapResult.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

WrapResult::WrapResult()
    : startPoint{-1},
      endPoint{-1},
      wrap_pts{SimTK::Vec3(SimTK::NaN), 0, 1},
      wrap_path_length{SimTK::NaN},
      r1{SimTK::Vec3(SimTK::NaN)},
      r2{SimTK::Vec3(SimTK::NaN)},
      c1{SimTK::Vec3(SimTK::NaN)},
      sv{SimTK::Vec3(SimTK::NaN)} {}

WrapResult::WrapResult(const WrapResult& other)
    : startPoint{other.startPoint},
      endPoint{other.endPoint},
      wrap_pts{other.wrap_pts},
      wrap_path_length{other.wrap_path_length},
      r1{other.r1},
      r2{other.r2},
      c1{other.c1},
      sv{other.sv},
      factor{SimTK::NaN},
      singleWrap{other.singleWrap} {}

WrapResult& WrapResult::operator=(const WrapResult& other) {
    if (this != &other) {
        startPoint = other.startPoint;
        endPoint = other.endPoint;
        wrap_pts = other.wrap_pts;
        wrap_path_length = other.wrap_path_length;
        r1 = other.r1;
        r2 = other.r2;
        c1 = other.c1;
        sv = other.sv;
        factor = SimTK::NaN;
        singleWrap = other.singleWrap;
    }

    return *this;
}
