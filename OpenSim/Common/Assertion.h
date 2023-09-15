#ifndef OPENSIM_ASSERTION_H_
#define OPENSIM_ASSERTION_H_

/* ------------------------------------------------------------------------- *
*                           OpenSim:  Assertion.h                            *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2023 Stanford University and the Authors                *
* Author(s): Adam Kewley                                                     *
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

namespace OpenSim {

    class Object;

    // a function that is called whenever an assertion fails
    //
    // callers may assume that the implementation halts forward progression of
    // the caller by either throwing an exception or terminating the process
    [[noreturn]] OSIMCOMMON_API void OnAssertionError(
        char const* failingCode,
        char const* failingFile,
        char const* failingFunction,
        unsigned int failingLine,
        Object const* maybeSourceObject = nullptr
    );
}

/**
 * @name Macros to assert that expressions are true
 *
 * The purpose of these macros is to assert that preconditions are met within the
 * source code of OpenSim. If the precondition is not met, and the macros are enabled,
 * then the implementation will halt forward progression of the calling thread, either
 * by throwing an exception or terminating the process.
 */

// always ensures `expr` converts to `true` or otherwise halts forward progression of the calling thread
#define OPENSIM_ASSERT_ALWAYS(expr) \
    static_cast<bool>(expr) ? static_cast<void>(0) : OpenSim::OnAssertionError(#expr, __FILE__, __func__, __LINE__)

// always ensures `expr` converts to `true` or otherwise halts forward progression of the calling thread
// and passes `this` to the error handler (for better error messages)
#define OPENSIM_ASSERT_FRMOBJ_ALWAYS(expr) \
    static_cast<bool>(expr) ? static_cast<void>(0) : OpenSim::OnAssertionError(#expr, __FILE__, __func__, __LINE__, this)

// these macros behave identially to their `_ALWAYS` counterparts, but may be conditionally
// toggled by compile-time flags
#if defined(NDEBUG)
#define OPENSIM_ASSERT(expr)
#define OPENSIM_ASSERT_FRMOBJ(expr)
#else
#define OPENSIM_ASSERT(expr) OPENSIM_ASSERT_ALWAYS(expr)
#define OPENSIM_ASSERT_FRMOBJ(expr) OPENSIM_ASSERT_FRMOBJ_ALWAYS(expr)
#endif

#endif // OPENSIM_ASSERTION_H_
