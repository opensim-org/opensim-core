% -------------------------------------------------------------------------- %
% OpenSim Muscollo: testStateAndControlInfo.m                                %
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


