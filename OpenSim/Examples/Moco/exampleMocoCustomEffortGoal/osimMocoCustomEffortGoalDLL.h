#ifndef OPENSIM_OSIMMOCOCUSTOMEFFORTGOALDLL_H
#define OPENSIM_OSIMMOCOCUSTOMEFFORTGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoCustomEffortGoalDLL.h                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOCUSTOMEFFORTGOAL_API
#else
    #ifdef OSIMMOCOCUSTOMEFFORTGOAL_EXPORTS
        #define OSIMMOCOCUSTOMEFFORTGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOCUSTOMEFFORTGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOCUSTOMEFFORTGOALDLL_H
