#ifndef OPENSIM_PARSE_ARGUMENTS_H_
#define OPENSIM_PARSE_ARGUMENTS_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  parse_arguments.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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

#include <docopt.h>

#include <OpenSim/OpenSim.h>
#include <OpenSim/Moco/osimMoco.h>

namespace OpenSim {

// This implementation was copied from docopt.cpp; the only difference is that
// this function can throw exceptions (whereas `docopt::docopt()` is
// `noexcept`).
std::map<std::string, docopt::value>
parse_arguments(std::string const& doc,
            std::vector<std::string> const& argv,
            bool help = true,
            std::string const& version = {},
            bool options_first = false)
{
    using namespace docopt;
    try {
        return docopt_parse(doc, argv, help, !version.empty(), options_first);
    } catch (DocoptExitHelp const&) {
        log_cout(doc);
        std::exit(0);
    } catch (DocoptExitVersion const&) {
        log_cout(version);
        std::exit(0);
    } catch (DocoptLanguageError const& error) {
        log_error("Docopt usage string could not be parsed.");
        log_error(error.what());
        std::exit(-1);
    } catch (DocoptArgumentError const& error) {
        log_error(error.what());
        log_error("Use --help to get more information.");
        std::exit(-1);
    }
}

} // namespace OpenSim

#endif // OPENSIM_PARSE_ARGUMENTS_H_
