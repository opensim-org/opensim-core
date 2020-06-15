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

class MyFunction : public Callback {
public:
    MyFunction(const std::string& name) { construct(name); }
    ~MyFunction() {

    }
    casadi_int get_n_in() override { return 2; }
    casadi_int get_n_out() override { return 1; }
    Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0)
            return Sparsity::dense(2, 1);
        else if (i == 1)
            return Sparsity::dense(1, 1);
        else
            return Sparsity();
    }
    Sparsity get_sparsity_out(casadi_int) override {
        return Sparsity::dense(2, 1);
    }
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        auto x = arg.at(0);
        auto u = arg.at(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        std::cout << std::this_thread::get_id() << std::endl;
        auto xdot = DM::ones(2, 1);
        xdot(0) = x(1);
        xdot(1) = u(0);
        return {xdot};
    }
};

int main() {

    auto x = MX::sym("x", 2, 1);
    auto u = MX::sym("u", 1, 1);

    MyFunction ode("ode");

    int N = 100;
    auto odeTrajSerial = ode.map(N, "serial");

    // TODO: Threading results in memory issues. Something isn't threadsafe.
    // OpenMP isn't actually running the code in parallel.
    auto odeTrajParallel = ode.map(N, "thread", 8);

    auto xtraj = DM::rand(2, N); // MX::sym("xtraj", 2, 100);
    auto utraj = DM::rand(1, N); // MX::sym("utraj", 1, 100);

    double timeSerial;
    {
        auto start = SimTK::realTimeInNs();
        std::vector<DM> xdottraj = odeTrajSerial(std::vector<DM>{xtraj, utraj});
        timeSerial = SimTK::realTimeInNs() - start;
        std::cout << timeSerial << std::endl;
        std::cout << xdottraj.at(0) << std::endl;
    }
    double timeParallel;
    {
        auto start = SimTK::realTimeInNs();
        std::vector<DM> xdottraj =
                odeTrajParallel(std::vector<DM>{xtraj, utraj});
        timeParallel = SimTK::realTimeInNs() - start;
        std::cout << timeParallel << std::endl;
        std::cout << xdottraj.at(0) << std::endl;
    }
    std::cout << timeSerial / timeParallel << std::endl;

    return 0;
}
