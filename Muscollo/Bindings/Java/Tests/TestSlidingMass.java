/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: TestSlidingMass.java                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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
import org.opensim.modeling.*;

class TestSlidingMass {

  public static Model createSlidingMassModel() {
    Model model = new Model();
    model.setName("sliding_mass");
    model.set_gravity(new Vec3(0, 0, 0));
    Body body = new Body("body", 2.0, new Vec3(0), new Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    SliderJoint joint = new SliderJoint("slider", model.getGround(), body);
    Coordinate coord = joint.updCoordinate();
    coord.setName("position");
    model.addComponent(joint);

    CoordinateActuator actu = new CoordinateActuator();
    actu.setCoordinate(coord);
    actu.setName("actuator");
    actu.setOptimalForce(1);
    model.addComponent(actu);

    return model;
  }

  public static void testSlidingMass() {

    MucoTool muco = new MucoTool();
    muco.setName("sliding_mass");

    // Define the optimal control problem.
    // ===================================
    MucoProblem mp = muco.updProblem();

    // Model (dynamics).
    // -----------------
    mp.setModel(createSlidingMassModel());

    // Bounds.
    // -------
    // Initial time must be 0, final time can be within [0, 5].
    // TODO support "initializer list" in MATLAB.
    // TODO "using" does not work in SWIG.
    mp.setTimeBounds(new MucoInitialBounds(0.), new MucoFinalBounds(0., 5.));

    // Initial position must be 0, final position must be 1.
    mp.setStateInfo("slider/position/value", new MucoBounds(-5, 5),
        new MucoInitialBounds(0), new MucoFinalBounds(1));
    // Initial and final speed must be 0. Use compact syntax.
    // TODO mp.setStateInfo("slider/position/speed", {-50, 50}, 0, 0);
    mp.setStateInfo("slider/position/speed", new MucoBounds(-50, 50),
        new MucoInitialBounds(0), new MucoFinalBounds(0));

    // Applied force must be between -50 and 50.
    mp.setControlInfo("actuator", new MucoBounds(-50, 50));

    // Cost.
    // -----
    MucoFinalTimeCost ftCost = new MucoFinalTimeCost();
    mp.addCost(ftCost);

    // Configure the solver.
    // =====================
    MucoTropterSolver ms = muco.initSolver();
    ms.set_num_mesh_points(50);

    // Now that we've finished setting up the tool, print it to a file.
    muco.print("sliding_mass.omuco");

    // TODO upd_optim_solver().

    // Solve the problem.
    // ==================
    // TODO MucoSolution solution = muco.solve();

    // TODO solution.write("sliding_mass_solution.sto");
  }
  public static void main(String[] args) {
    testSlidingMass();
    System.out.println("Test finished!");
  }
}
