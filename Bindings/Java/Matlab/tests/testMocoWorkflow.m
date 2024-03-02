% -------------------------------------------------------------------------- %
% OpenSim Moco: testWorkflow.m                                               %
% -------------------------------------------------------------------------- %
% Copyright (c) 2017 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Christopher Dembia                                              %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0          %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

function result = testWorkflow
    tests = functiontests(localfunctions);
    result = run(tests);
    for i = 1:length(result)
        if result(i).Failed
            error('Test failed :(');
        end
    end
    disp('Test passed :)');
end

function model = createSlidingMassModel()
    import org.opensim.modeling.*;
    model = Model();
    model.setName('sliding_mass');
    model.set_gravity(Vec3(0, 0, 0));
    body = Body('body', 10.0, Vec3(0), Inertia(0));
    model.addComponent(body);

    % Allows translation along x.
    joint = SliderJoint('slider', model.getGround(), body);
    coord = joint.updCoordinate();
    coord.setName('position');
    model.addComponent(joint);

    actu = CoordinateActuator();
    actu.setCoordinate(coord);
    actu.setName('actuator');
    actu.setOptimalForce(1);
    actu.setMinControl(-10);
    actu.setMaxControl(10);
    model.addComponent(actu);

end

function testDefaultBounds(testCase)
    import org.opensim.modeling.*;
    study = MocoStudy();
    problem = study.updProblem();
    model = createSlidingMassModel();
    model.finalizeFromProperties();
    coord = Coordinate.safeDownCast(model.updComponent('slider/position'));
    coord.setRangeMin(-10);
    coord.setRangeMax(15);
    actu = ScalarActuator.safeDownCast(model.updComponent('actuator'));
    actu.setMinControl(35);
    actu.setMaxControl(56);
    problem.setModel(model);
    phase0 = problem.getPhase(0);
    
    rep = problem.createRep();
    info = rep.getStateInfo('/slider/position/value');
    testCase.assertEqual(info.getBounds().getLower(), -10);
    testCase.assertEqual(info.getBounds().getUpper(),  15);
    
    % Default speed bounds.
    info = rep.getStateInfo('/slider/position/speed');
    testCase.assertEqual(info.getBounds().getLower(), -50);
    testCase.assertEqual(info.getBounds().getUpper(),  50);
    
    % Obtained from controls.
    info = rep.getControlInfo('/actuator');
    testCase.assertEqual(info.getBounds().getLower(), 35);
    testCase.assertEqual(info.getBounds().getUpper(), 56);
    
    problem.setControlInfo('/actuator', [12, 15]);
    probinfo = phase0.getControlInfo('/actuator');
    testCase.assertEqual(probinfo.getBounds().getLower(), 12);
    testCase.assertEqual(probinfo.getBounds().getUpper(), 15);
    
    rep = problem.createRep();
    info = rep.getControlInfo('/actuator');
    testCase.assertEqual(info.getBounds().getLower(), 12);
    testCase.assertEqual(info.getBounds().getUpper(), 15);
end

function testChangingTimeBounds(testCase)
    import org.opensim.modeling.*;
    study = MocoStudy();
    problem = study.updProblem();
    problem.setModel(createSlidingMassModel());
    problem.setTimeBounds(0, [0, 10]);
    problem.setStateInfo('/slider/position/value', [0, 1], 0, 1);
    problem.setStateInfo('/slider/position/speed', [-100, 100], 0, 0);
    problem.setControlInfo('/actuator', [-10, 10]);
    problem.addGoal(MocoFinalTimeGoal());

    if MocoTropterSolver.isAvailable()
        solver = study.initTropterSolver();
        solver.set_transcription_scheme('trapezoidal')
        solver.set_num_mesh_intervals(19);
        guess = solver.createGuess('random');
        guess.setTime(opensimCommon.createVectorLinspace(20, 0.0, 3.0));
        solver.setGuess(guess);
        solution0 = study.solve();

        problem.setTimeBounds(0, [5.8, 10]);
        % Editing the problem does not affect information in the Solver; the
        % guess still exists.
        assert(~solver.getGuess().empty());

        solution = study.solve();
        testCase.assertEqual(solution.getFinalTime(), 5.8);
    end

end

function testChangingModel(testCase)
    import org.opensim.modeling.*;
    study = MocoStudy();
    problem = study.updProblem();
    model = createSlidingMassModel();
    problem.setModel(model);
    problem.setTimeBounds(0, [0, 10]);
    problem.setStateInfo('/slider/position/value', [0, 1], 0, 1);
    problem.setStateInfo('/slider/position/speed', [-100, 100], 0, 0);
    problem.addGoal(MocoFinalTimeGoal());
    if MocoTropterSolver.isAvailable()
        solver = study.initTropterSolver();
        solver.set_num_mesh_intervals(20);
        finalTime0 = study.solve().getFinalTime();

        testCase.assertEqual(finalTime0, 2.00, 'AbsTol', 0.01);

        body = Body.safeDownCast(model.updComponent('body'));
        body.setMass(2 * body.getMass());
        finalTime1 = study.solve().getFinalTime();
        assert(finalTime1 > 1.1 * finalTime0);
    end
end

function testOrder(testCase)
    import org.opensim.modeling.*;
    % Can set the cost and model in any order.
    study = MocoStudy();
    problem = study.updProblem();
    problem.setTimeBounds(0, [0, 10]);
    problem.setStateInfo('/slider/position/value', [0, 1], 0, 1);
    problem.setStateInfo('/slider/position/speed', [-100, 100], 0, 0);
    problem.addGoal(MocoFinalTimeGoal());
    problem.setModel(createSlidingMassModel());
    if MocoTropterSolver.isAvailable()
        solver = study.initTropterSolver();
        solver.set_num_mesh_intervals(20);
        finalTime =  study.solve().getFinalTime();

        testCase.assertEqual(finalTime, 2.0, 'AbsTol', 0.01);
    end
end

function testChangingGoals(testCase)
    import org.opensim.modeling.*;
    % Changes to the costs are obeyed.
    study = MocoStudy();
    problem = study.updProblem();
    problem.setModel(createSlidingMassModel());
    problem.setTimeBounds(0, [0, 10]);
    problem.setStateInfo('/slider/position/value', [0, 1], 0, 1);
    problem.setStateInfo('/slider/position/speed', [-100, 100], 0, 0);
    problem.updPhase().addGoal(MocoFinalTimeGoal());
    effort = MocoControlGoal('effort');
    problem.updPhase().addGoal(effort);
    if MocoCasADiSolver.isAvailable()
        finalTime0 = study.solve().getFinalTime();

        % Change the weights of the costs.
        effort.setWeight(0.1);
        assert(study.solve().getFinalTime() < 0.8 * finalTime0);
    end
end
