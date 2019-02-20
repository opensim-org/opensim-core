/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxCasADiParallelMap                                     *
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

#include <SimTKcommon.h>

#include <casadi/casadi.hpp>

using namespace casadi;

int main() {

    auto x = MX::sym("x", 2, 1);
    auto u = MX::sym("u", 1, 1);

    auto xdot = MX(2, 1);
    xdot(0) = x(1);
    xdot(1) = u(0);

    Function ode("ode", {x, u}, {xdot});
    auto odeTraj = ode.map(100, "openmp");

    auto xtraj = DM::zeros(2, 100); // MX::sym("xtraj", 2, 100);
    auto utraj = DM::zeros(1, 100); // MX::sym("utraj", 1, 100);

    auto start = SimTK::realTimeInNs();
    std::vector<DM> xdottraj = odeTraj(std::vector<DM>{xtraj, utraj});
    std::cout << SimTK::realTimeInNs() - start << std::endl;

    std::cout << xdottraj.at(0) << std::endl;


    return 0;
}
