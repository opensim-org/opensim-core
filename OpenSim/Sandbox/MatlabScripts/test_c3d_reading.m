% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2017 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
%                                                                         %
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         %
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %

% Author: James Dunne, Tom Uchida, Shrinidhi K. Lakshmikanth, Chris Dembia, 
% Ajay Seth, Ayman Habib, Jen Hicks.

import org.opensim.modeling.*


%% Test script for utility function 
c3d_reading('filepath', 'test_walking.c3d');

%% Test if trc and mot file were printed. 
assert( exist('test_walking.trc', 'file') == 2, 'TRC was not printed ')
assert( exist('test_walking.mot', 'file') == 2, 'TRC was not printed ')

%% Test rotations
% test if roations about X, Y, and Z, respectively, maintain the correct
% components. ie if rotatiing about X, X values remain the same. 

%% Get the unrotated reference values
mkr = TRCFileAdapter.read('test_walking.trc');
ana = STOFileAdapter().read('test_walking.mot');

mkr_Xvalue_ref = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(0);
mkr_Yvalue_ref = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(1);
mkr_Zvalue_ref = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(2);

ana_Xvalue_ref = ana.getDependentColumnAtIndex(0).getElt(0,0);
ana_Yvalue_ref = ana.getDependentColumnAtIndex(1).getElt(0,0);
ana_Zvalue_ref = ana.getDependentColumnAtIndex(2).getElt(0,0);

%% Rotate about X
c3d_reading('filepath', 'test_walking.c3d', 'firstrotation', 90, 'axis', 'x');
mkr = TRCFileAdapter.read('test_walking.trc');
ana = STOFileAdapter().read('test_walking.mot');
% Set the new marker and force values
mkr_Xvalue = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(0);
mkr_Yvalue = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(1);
mkr_Zvalue = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(2);
ana_Xvalue = ana.getDependentColumnAtIndex(0).getElt(0,0);
ana_Yvalue = ana.getDependentColumnAtIndex(1).getElt(0,0);
ana_Zvalue = ana.getDependentColumnAtIndex(2).getElt(0,0);
% Assess if the rotation is correct 
assert(mkr_Xvalue_ref ==  mkr_Xvalue, 'X axis marker rotation is incorrect ')
assert(mkr_Yvalue_ref ~=  mkr_Yvalue, 'Y axis marker rotation is incorrect ')
assert(mkr_Zvalue_ref ~=  mkr_Zvalue, 'Z axis marker rotation is incorrect ')
assert(abs(mkr_Zvalue_ref) ==  abs(mkr_Yvalue), 'Y axis marker rotation is incorrect')
assert(abs(mkr_Yvalue_ref) ==  abs(mkr_Zvalue), 'Z axis marker rotation is incorrect')
assert(ana_Xvalue_ref ==  ana_Xvalue, 'X axis force rotation is incorrect ')
assert(ana_Yvalue_ref ~=  ana_Yvalue, 'Y axis force rotation is incorrect ')
assert(ana_Zvalue_ref ~=  ana_Zvalue, 'Z axis force rotation is incorrect ')
assert(abs(ana_Zvalue_ref) ==  abs(ana_Yvalue), 'Y axis force rotation is incorrect')
assert(abs(ana_Yvalue_ref) ==  abs(ana_Zvalue), 'Z axis force rotation is incorrect')

%% Roate about Y
c3d_reading('filepath', 'test_walking.c3d', 'firstrotation', 90, 'axis', 'y');
mkr = TRCFileAdapter.read('test_walking.trc');
ana = STOFileAdapter().read('test_walking.mot');
% Set the new marker and force values
mkr_Xvalue = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(0);
mkr_Yvalue = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(1);
mkr_Zvalue = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(2);
ana_Xvalue = ana.getDependentColumnAtIndex(0).getElt(0,0);
ana_Yvalue = ana.getDependentColumnAtIndex(1).getElt(0,0);
ana_Zvalue = ana.getDependentColumnAtIndex(2).getElt(0,0);
% Assess if the rotation is correct 
assert(mkr_Yvalue_ref ==  mkr_Yvalue, 'Y axis marker rotation is incorrect ')
assert(mkr_Xvalue_ref ~=  mkr_Xvalue, 'X axis marker rotation is incorrect ')
assert(mkr_Zvalue_ref ~=  mkr_Zvalue, 'Z axis marker rotation is incorrect ')
assert(abs(mkr_Zvalue_ref) ==  abs(mkr_Xvalue), 'X axis marker rotation is incorrect')
assert(abs(mkr_Xvalue_ref) ==  abs(mkr_Zvalue), 'Z axis marker rotation is incorrect')
assert(ana_Yvalue_ref ==  ana_Yvalue, 'Y axis force rotation is incorrect ')
assert(ana_Xvalue_ref ~=  ana_Xvalue, 'X axis force rotation is incorrect ')
assert(ana_Zvalue_ref ~=  ana_Zvalue, 'Z axis force rotation is incorrect ')
assert(abs(ana_Zvalue_ref) ==  abs(ana_Xvalue), 'X axis force rotation is incorrect')
assert(abs(ana_Xvalue_ref) ==  abs(ana_Zvalue), 'Z axis force rotation is incorrect')

%% Roate about Z
c3d_reading('filepath', 'test_walking.c3d', 'firstrotation', 90, 'axis', 'z');
mkr = TRCFileAdapter.read('test_walking.trc');
ana = STOFileAdapter().read('test_walking.mot');
% Set the new marker and force values
mkr_Xvalue = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(0);
mkr_Yvalue = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(1);
mkr_Zvalue = mkr.getDependentColumnAtIndex(0).getElt(0,0).get(2);
ana_Xvalue = ana.getDependentColumnAtIndex(0).getElt(0,0);
ana_Yvalue = ana.getDependentColumnAtIndex(1).getElt(0,0);
ana_Zvalue = ana.getDependentColumnAtIndex(2).getElt(0,0);
% Assess if the rotation is correct 
assert(mkr_Zvalue_ref ==  mkr_Zvalue, 'Z axis marker rotation is incorrect ')
assert(mkr_Yvalue_ref ~=  mkr_Yvalue, 'Y axis marker rotation is incorrect ')
assert(mkr_Xvalue_ref ~=  mkr_Xvalue, 'X axis marker rotation is incorrect ')
assert(abs(mkr_Xvalue_ref) ==  abs(mkr_Yvalue), 'Y axis marker rotation is incorrect')
assert(abs(mkr_Yvalue_ref) ==  abs(mkr_Xvalue), 'X axis marker rotation is incorrect')
assert(ana_Zvalue_ref ==  ana_Zvalue, 'Z axis force rotation is incorrect ')
assert(ana_Yvalue_ref ~=  ana_Yvalue, 'Y axis force rotation is incorrect ')
assert(ana_Xvalue_ref ~=  ana_Xvalue, 'X axis force rotation is incorrect ')
assert(abs(ana_Xvalue_ref) ==  abs(ana_Yvalue), 'Y axis force rotation is incorrect')
assert(abs(ana_Yvalue_ref) ==  abs(ana_Xvalue), 'X axis force rotation is incorrect')



