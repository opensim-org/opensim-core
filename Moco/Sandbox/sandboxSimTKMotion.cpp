/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSimTKMotion.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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

#include "Simbody.h"

using namespace SimTK;
using std::cout;
using std::endl;

class MyReporter : public PeriodicEventReporter {
public:
    MyReporter(const MultibodySystem& system,
            Real interval)
            : PeriodicEventReporter(interval), system(system) {}

    // Show x-y position of the pendulum weight as a function of time.
    void handleEvent(const State& state) const override {
        system.realize(state, Stage::Acceleration);
        Vector mm;
        Vector mf;
        mm = system.getMatterSubsystem().getMotionMultipliers(state);
        system.getMatterSubsystem().findMotionForces(state, mf);
        std::cout << state.getTime() << "\t" << mm << "\t" << mf
                  << std::endl;
    }

private:
    const MultibodySystem& system;
};

int main() {
    // Much of this is copied from Simbody's ExamplePendulum.cpp.
    // Create the system, with subsystems for the bodies and some forces.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);

    // Add gravity as a force element.
    Force::UniformGravity gravity(forces, matter, Vec3(0, Real(-9.8), 0));
    // Create the body and some artwork for it.
    Body::Rigid pendulumBody(MassProperties(1.0, Vec3(0), Inertia(1)));
    pendulumBody.addDecoration(
            Transform(), DecorativeSphere(Real(0.1)).setColor(Red));

    // Add an instance of the body to the multibody system by connecting
    // it to Ground via a pin mobilizer.
    MobilizedBody::Pin pendulum1(matter.updGround(), Transform(Vec3(0, -1, 0)),
            pendulumBody, Transform(Vec3(0, 1, 0)));
    MobilizedBody::Pin pendulum1b(pendulum1, Transform(Vec3(0, -1, 0)),
            pendulumBody, Transform(Vec3(0, 1, 0)));

    Motion::Sinusoid motion(pendulum1, Motion::Level::Position, 1.0, 2.0, 0);

    // Visualize with default options; ask for a report every 1/30 of a second
    // to match the Visualizer's default 30 frames per second rate.
    Visualizer viz(system);
    system.addEventReporter(new Visualizer::Reporter(viz, Real(1. / 30)));

    system.addEventReporter(new MyReporter(system, 0.01));

    // Initialize the system and state.

    system.realizeTopology();
    State state = system.getDefaultState();

    system.realize(state);

    std::cout << "DEBUG " << state.getNY() << std::endl;
    std::cout << "DEBUG " << state.getNMultipliers() << std::endl;

    viz.report(state);
    // Simulate it.
    cout << "Hit ENTER to run a short simulation ...";
    getchar();

    RungeKuttaMersonIntegrator integ(system);
    TimeStepper ts(system, integ);
    ts.initialize(state);
    ts.stepTo(30.0);
    state = integ.getState();
    system.realize(state);

    return EXIT_SUCCESS;
}
