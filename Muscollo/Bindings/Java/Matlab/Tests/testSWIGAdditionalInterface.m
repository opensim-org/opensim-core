% -------------------------------------------------------------------------- %
% OpenSim Muscollo: testSWIGAdditionalInterface.m                            %
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
import org.opensim.modeling.*;

%% Bounds.

model = Model();
model.setName('sliding_mass');
model.set_gravity(Vec3(0, 0, 0));
body = Body('body', 2.0, Vec3(0), Inertia(0));
model.addComponent(body);

joint = SliderJoint('slider', model.getGround(), body);
coord = joint.updCoordinate();
coord.setName('position');
model.addComponent(joint);

actu = CoordinateActuator();
actu.setCoordinate(coord);
actu.setName('actuator');
model.addComponent(actu);

muco = MucoTool();
muco.setName('sliding_mass');

mp = muco.updProblem();
ph0 = mp.getPhase();

mp.setModel(model);

almostEqual = @(x, y) abs(x - y) < 1e-15;

mp.setTimeBounds(MucoInitialBounds(0.), MucoFinalBounds(0.1, 5.));
assert(ph0.getTimeInitialBounds().getLower() == 0);
assert(ph0.getTimeInitialBounds().getUpper() == 0);
assert(almostEqual(ph0.getTimeFinalBounds().getLower(), 0.1));
assert(almostEqual(ph0.getTimeFinalBounds().getUpper(), 5.0));

mp.setTimeBounds([0.2, 0.3], [3.5]);
assert(almostEqual(ph0.getTimeInitialBounds().getLower(), 0.2));
assert(almostEqual(ph0.getTimeInitialBounds().getUpper(), 0.3));
assert(almostEqual(ph0.getTimeFinalBounds().getLower(), 3.5));
assert(almostEqual(ph0.getTimeFinalBounds().getUpper(), 3.5));

% Use setter on MucoPhase.
ph0.setTimeBounds([2.2, 2.3], [4.5]);
assert(almostEqual(ph0.getTimeInitialBounds().getLower(), 2.2));
assert(almostEqual(ph0.getTimeInitialBounds().getUpper(), 2.3));
assert(almostEqual(ph0.getTimeFinalBounds().getLower(), 4.5));
assert(almostEqual(ph0.getTimeFinalBounds().getUpper(), 4.5));


mp.setStateInfo('slider/position/value', MucoBounds(-5, 5), ...
    MucoInitialBounds(0));
assert(-5 == ph0.getStateInfo('slider/position/value').getBounds().getLower());
assert( 5 == ph0.getStateInfo('slider/position/value').getBounds().getUpper());
assert(isnan(ph0.getStateInfo('slider/position/value').getFinalBounds().getLower()));
assert(isnan(ph0.getStateInfo('slider/position/value').getFinalBounds().getUpper()));
mp.setStateInfo('slider/position/speed', [-50, 50], [-3], 1.5);
assert(-50 == ph0.getStateInfo('slider/position/speed').getBounds().getLower());
assert( 50 == ph0.getStateInfo('slider/position/speed').getBounds().getUpper());
assert(-3 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getLower());
assert(-3 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getUpper());
assert(almostEqual(1.5, ...
    ph0.getStateInfo('slider/position/speed').getFinalBounds().getLower()));
assert(almostEqual(1.5, ...
    ph0.getStateInfo('slider/position/speed').getFinalBounds().getUpper()));

% Use setter on MucoPhase.
ph0.setStateInfo('slider/position/speed', [-6, 10], [-4, 3], [0]);
assert(-6 == ph0.getStateInfo('slider/position/speed').getBounds().getLower());
assert(10 == ph0.getStateInfo('slider/position/speed').getBounds().getUpper());
assert(-4 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getLower());
assert( 3 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getUpper());
assert(0 == ph0.getStateInfo('slider/position/speed').getFinalBounds().getLower());
assert(0 == ph0.getStateInfo('slider/position/speed').getFinalBounds().getUpper());

% Controls.
mp.setControlInfo('actuator', MucoBounds(-50, 50));
assert(-50 == ph0.getControlInfo('actuator').getBounds().getLower());
assert( 50 == ph0.getControlInfo('actuator').getBounds().getUpper());
mp.setControlInfo('actuator', [18]);
assert(18 == ph0.getControlInfo('actuator').getBounds().getLower());
assert(18 == ph0.getControlInfo('actuator').getBounds().getUpper());


%% MucoIterate
time = Vector(3, 0);
time.set(0, 0);
time.set(1, 0.1);
time.set(2, 0.2);
sn = StdVectorString();
sn.add('s0');
sn.add('s1');
cn = StdVectorString();
cn.add('c0');
cn.add('c1');
cn.add('c2');
pn = StdVectorString();
pn.add('p0');
pn.add('p1');
st = Matrix(3, 2);
ct = Matrix(3, 3);
p = RowVector(2, 0.0);
it = MucoIterate(time, sn, cn, pn, st, ct, p);

it.setTime([15, 25, 35]);
assert(it.getTime().get(0) == 15);
assert(it.getTime().get(1) == 25);
assert(it.getTime().get(2) == 35);

it.setState('s0', [5, 3, 10]);
s0traj = it.getState('s0');
assert(s0traj.get(0) == 5);
assert(s0traj.get(1) == 3);
assert(s0traj.get(2) == 10);
it.setState('s1', [2, 6, 1]);
s1traj = it.getState('s1');
assert(s1traj.get(0) == 2);
assert(s1traj.get(1) == 6);
assert(s1traj.get(2) == 1);

it.setControl('c0', [10, 46, -5]);
c0traj = it.getControl('c0');
assert(c0traj.get(0) == 10);
assert(c0traj.get(1) == 46);
assert(c0traj.get(2) == -5);
it.setControl('c2', [5, 12, -1]);
c2traj = it.getControl('c2');
assert(c2traj.get(0) == 5);
assert(c2traj.get(1) == 12);
assert(c2traj.get(2) == -1);

it.setParameter('p0', 25);
it.setParameter('p1', 30);
p = it.getParameters();
assert(p.get(0) == 25);
assert(p.get(1) == 30);
p0 = it.getParameter('p0');
assert(p0 == 25);
