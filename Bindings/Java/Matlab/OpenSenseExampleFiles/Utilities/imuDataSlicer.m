% IMUDataSlicer()
%   Class to extract sub trials from IMU data for OpenSense Tracking.
%   Inputs:
%   trialName   String of the IMU orientation files. This must also have an
%               acompanying accelerations file (used for static analysis)
%               e.g. 'MT_012005D6_009-001-quaternions.sto' & a
%               corresponding MT_012005D6_009-001-accelerations.sto file. 

classdef imuDataSlicer < matlab.mixin.SetGet
% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2019 Stanford University and the Authors             %
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
    
    properties (Access = private)
        orientationsFilePath
        oData
        accFilePath
        accData
        timeArray
        t0
        tn
        subtrial
        staticTrial
    end
    methods
        function obj = imuDataSlicer(trialName)
            import org.opensim.modeling.*
            currentFolder = cd;
            
            % Read orientation Data
            orientationsFilePath = fullfile(currentFolder,trialName);
            if ~exist(orientationsFilePath,'file')
                error('Orientations file does not exist')
            else
                obj.orientationsFilePath = orientationsFilePath;
                obj.oData = STOFileAdapterQuaternion().readFile(orientationsFilePath);
            end
            
            % 
            obj.accFilePath = [];
            obj.accData = [];
            
            % Get the time information from the sto file
            timeVec = obj.oData.getIndependentColumn();
            for i = 0 : timeVec.size() - 1
                timeArray(i+1,1) = round(str2num(sprintf('%.2f',timeVec.get(i)))*1000)/1000;
            end
            obj.timeArray = timeArray;
            
            % Initialize the data times to empty arrays
            obj.t0 = [];
            obj.tn = [];   
            
            % Initialize static trial
            staticTrial = [];
            subtrialTrial = [];
        end
        function tv = getTimeVector(obj)
            tv = obj.timeArray;
        end
        function st = getStartTime(obj)
            st = obj.timeArray(1);
        end
        function et = getEndTime(obj)
            format long g
            et = obj.timeArray(end);
            mins=fix(et/60); secs = rem(et,60);
            disp([num2str(mins) ' minutes, ' num2str(secs) ' seconds']);
        end
        function r = getRate(obj)
           r =  1/(obj.timeArray(2) - obj.timeArray(1));
        end
        function setDataTimeInterval(obj, t0, tn)
            
            if   obj.getStartTime() > t0 
                error('Intial time is out of range');
            elseif t0 > tn 
                error('Final time must be larger than initial time');
            elseif tn > obj.getEndTime()
                error('Final time is out of range');
            end
            
            obj.t0 = t0;
            obj.tn = tn;
        end
        function setDataTimeIntervalInMinutes(obj, stime, etime)
            t0 = seconds(minutes(fix((stime)))) + ((stime - fix((stime)))*100);
            tn = seconds(minutes(fix((etime)))) + ((etime - fix((etime)))*100);
            
            obj.t0 = t0;
            obj.tn = tn;
        end
        function t0 = getDataTimeIntervalStart(obj)
            t0 = obj.t0;
        end
        function tn = getDataTimeIntervalEnd(obj)
            tn = obj.tn;
        end
        function generateSubTrial(obj)
            import org.opensim.modeling.*
            % Check the time interval has been set.
            obj.isTimeIntervalSet()
            % Take a slice out of the bigger file
            disp('Getting Data Slice using Time Intervals.....');
            dataTableSlice = obj.getSlice(obj.oData, obj.t0,obj.tn);
            subtrial = TimeSeriesTableQuaternion(dataTableSlice);
            for i = 0 : obj.oData.getTableMetaDataKeys().size() - 1
                metakey = char(obj.oData.getTableMetaDataKeys().get(i));
                value = char(obj.oData.getTableMetaDataAsString(metakey));
                % Add meta data to the new trial
                subtrial.addTableMetaDataString(metakey,value);
            end
            
            obj.subtrial = subtrial;
            disp('Sub trial data added. Use getSubTrial() to return.')
        end
        function isStatic(obj, accelerationsFilePath)
            import org.opensim.modeling.*
            % Check the time interval has been set.
            obj.isTimeIntervalSet()
            
            % Read acceleration data
            accFilePath = fullfile(cd,accelerationsFilePath);
            if ~exist(accFilePath,'file')
               error(['File not found ' accFilePath])
            else
                obj.accFilePath = accFilePath;
                obj.accData = STOFileAdapterVec3().readFile(accFilePath);
            end
            
            % For the trial to be static, the data must be relatively
            % constant, i.e. no movement. We will set some arbitary level
            % of change for the sensors and compute if, at the time
            % interval given, if the it is below this level. 
            disp('Getting Data Slice using Time Intervals.....');
            dataTableSlice = obj.getSlice(obj.accData ,obj.t0, obj.tn);
            
            % Convert the Data table to a Matlab Struct
            disp('Converting OpenSim Table to Matlab table for analysis...')
            dataStruct = obj.convertToMatlabArray(dataTableSlice);
            
            % Find the biggest chunk of non-moving data in the time
            % interval
            disp('analyzing')
            [f0,f1] = obj.whenNotMoving(dataStruct);
            ta = dataTableSlice.getIndependentColumn().get(f0-1);
            tb = dataTableSlice.getIndependentColumn().get(f1-1);
            disp(['Static frames found between ' num2str(ta) ' & ' num2str(tb) ' seconds']);
            staticTrial = TimeSeriesTableQuaternion( obj.getSlice(obj.oData, ta,tb) );
            for i = 0 : obj.oData.getTableMetaDataKeys().size() - 1
                metakey = char(obj.oData.getTableMetaDataKeys().get(i));
                value = char(obj.oData.getTableMetaDataAsString(metakey));
                % Add meta data to the new trial
                staticTrial.addTableMetaDataString(metakey,value);
            end
            obj.staticTrial = staticTrial;
            
            disp('Static trial data added. Use getStaticTrial() to return.')
        end
        function staticTrial = getStaticTrial(obj)
            if isempty(obj.staticTrial())
                error('No static trial defined')
            else
                staticTrial = obj.staticTrial();
            end
        end
        function staticTrial = getSubTrial(obj)
            if isempty(obj.subtrial)
                error('No sub trial found.')
            else
                staticTrial = obj.subtrial();
            end
        end
        function writeStaticTrial(obj, postfix)
            import org.opensim.modeling.*
            [path, name, ext] = fileparts(obj.orientationsFilePath);
            trialName = fullfile(path, [name '_' postfix ext]);
            STOFileAdapterQuaternion.write(obj.staticTrial,  trialName);
            disp(['File ' trialName ' printed to file']);
        end
        function writeSubTrial(obj, postfix)
            import org.opensim.modeling.*
            [path, name, ext] = fileparts(obj.orientationsFilePath);
            trialName = fullfile(path, [name '_' postfix ext]);
            STOFileAdapterQuaternion.write(obj.subtrial,  trialName);
            disp(['File ' trialName ' printed to file']);
        end
    end
    methods (Access = private)
       function dataTableSlice = getSlice(obj, dataTable ,t0, tn)
            import org.opensim.modeling.*
            
            if contains(char(dataTable.getClass), 'TimeSeriesTableQuaternion')
               dataTableSlice = DataTableQuaternion();
            else
               dataTableSlice = DataTableVec3();
            end
            
            % Set the labels on the new Table
            labels = dataTable.getColumnLabels();
            for i = 0 : labels.size() - 1
                labels.set(i, [char(labels.get(i)) '_imu'])
            end
            dataTableSlice.setColumnLabels( labels );
            
            % Get all the row indices to use
            R0 = dataTable.getNearestRowIndexForTime(t0);
            Rn = dataTable.getNearestRowIndexForTime(tn);
            rowArray = (R0:Rn)';
            
            for u = 1 : length(rowArray)
                i = rowArray(u);
                rowi = dataTable.getRowAtIndex(i);
                dataTableSlice.appendRow(i,rowi);
                dataTableSlice.setIndependentValueAtIndex(u-1, obj.timeArray(i+1));
            end
       end
       function [f0,f1] = whenNotMoving(obj,dataStruct)
            cutOff = 2;
            nIMUs = length(fieldnames(dataStruct));
            % Get the Data
            for i = 1 : nIMUs
               labels = fieldnames(dataStruct);
               dataArray = dataStruct.(labels{i});
               % get the vertical axis 
               [v,vertAxis] = max(abs(mean(dataArray)));
               % Remove bias and absolute
               % zAxis = abs(dataArray(:,vertAxis) - mean(dataArray(:,vertAxis)));
               zAxis = abs(detrend(dataArray(:,vertAxis)));
               % Make NaN any values above the cut off value.  
               zAxis(find(zAxis > cutOff) ) = nan;
               % Assign a 1 or 0 if the frame has a NaN
               isMoving(:,i) = isnan(zAxis);
 
               % Plotting for debugging
               % subplot(4,2,i)
               % plot(zAxis)
            end
            % Compute the largest chunk of time in the trial when the
            % person is standing 'still'. Sum if moving (1 or 0) for 
            % each IMU, across each frame 
            g = sum(isMoving,2);
            % Any frame that has a moving IMU is changed to 1
            g(find(g~= 0)) = 1;
            % is Standing
            x = ~g;
            d = [true; diff(x) ~= 0];   % TRUE if values change
            k = find([d', true]);       % Indices of changes
            n = diff(k);   
            [j,i] = max(n);
            f0 = k(i);
            f1 = k(i+1) - 1;
            stop = 0;
            while stop == 0
                if mean(x(f0:f1)) ~= 1
                    n(i) = [];
                    [j,i] = max(n);
                    f0 = k(i);
                    f1 = k(i+1) - 1;
                else
                    stop = 1;
                end
            end
       end
       function dataStruct = convertToMatlabArray(obj, dataTable)
           import org.opensim.modeling.*
           
           dataStruct = struct;
           
           if contains(char(dataTable.getClass()),'Vec3')
               dataArray = zeros(dataTable.getNumRows(),3);
           elseif contains(char(dataTable.getClass()),'Quat')
               dataArray = zeros(dataTable.getNumRows(),5);
           end
           
           for u = 1 : dataTable.getNumColumns()
               for i = 1 : dataTable.getNumRows()
                    dataArray(i,:) = [dataTable.getRowAtIndex(i-1).get(u-1).get(0)...
                                      dataTable.getRowAtIndex(i-1).get(u-1).get(1)...
                                      dataTable.getRowAtIndex(i-1).get(u-1).get(2)];
               end
               dataStruct.(char( dataTable.getColumnLabels().get(u-1))) = dataArray;
           end           
       end
       function isTimeIntervalSet(obj)
           if isempty(obj.t0) || isempty(obj.tn)
              error('Time intervals have not been set. Call method setDataTimeInterval(t0,tn)')
           end
       end
    end
end
