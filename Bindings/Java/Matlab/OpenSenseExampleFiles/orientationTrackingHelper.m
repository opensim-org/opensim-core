% orientationTrackingHelper()
%   Helper Class to perform OpenSense Tracking of Orienation data. 
%   Models can be calibrated and IK Tracking. 
%   There are no Inputs to this class, but updates must be made to settings
%   such as the setCalibratedModelName() and trackingOrientationsFileName()
classdef orientationTrackingHelper < matlab.mixin.SetGet

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
        modelCalibrationPoseFile 
        calibrationTrialName 
        baseImuName 
        baseHeadingAxis
        opensenseUtility
        visualizeCalibratedModel
        calibratedModel
        calibratedModelName
        ikSetupPath
        trackingOrientationsFileName
        visualizeTracking
        outputResultsFileName
        ikStartTime
        ikEndTime
        ikStudy
        ikResultsDir
    end
    methods
        function obj = orientationTrackingHelper
            import org.opensim.modeling.*
            obj.modelCalibrationPoseFile ='';
            obj.calibrationTrialName = '';
            obj.baseImuName = 'pelvis_imu';
            obj.baseHeadingAxis = CoordinateAxis(0);
            obj.opensenseUtility = OpenSenseUtilities();
            obj.visualizeCalibratedModel = 0;
            outputResultsFileName = [];
            obj.ikStartTime = [];
            obj.ikEndTime = [];
        end
        function generateCalibratedModel(obj)
            import org.opensim.modeling.*
            model = obj.opensenseUtility.calibrateModelFromOrientations(obj.modelCalibrationPoseFile,...
                                                obj.calibrationTrialName,...
                                                obj.baseImuName,...
                                                obj.baseHeadingAxis,...
                                                obj.visualizeCalibratedModel);
            model.initSystem();
            obj.calibratedModel = model;
        end
        function writeCalibratedModel2File(obj)
            import org.opensim.modeling.*
            obj.calibratedModel.print(obj.calibratedModelName())
        end
        function setCalibratedModelOutputName(obj,fileName)
            obj.calibratedModelName = fileName;
        end
        function o = getCalibratedModelOutputName(obj)
            o = obj.calibratedModelName;
        end
        function setModelCalibrationPoseFile(obj, fileName)
            obj.modelCalibrationPoseFile = fileName;
        end
        function o = getModelCalibrationPoseFile(obj)
            o = obj.modelCalibrationPoseFile;
        end
        function setCalibrationTrialName(obj, fileName)
            obj.calibrationTrialName = fileName;
        end
        function o = getCalibrationTrialName(obj)
            o = obj.calibrationTrialName;
        end
        function setBaseHeadingAxis(obj, heading)
            import org.opensim.modeling.*
            obj.baseHeadingAxis = CoordinateAxis(heading);
        end
        function o = getBaseHeadingAxis(obj)
            o = obj.baseHeadingAxis();
        end
        function setVisualizeCalibratedModel(obj, b)
            obj.visualizeCalibratedModel = b;
        end
        function o = getVisualizeCalibratedModel(obj)
            o = obj.visualizeCalibratedModel ();
        end
        function setIkSetupPath(obj, fileName)
            obj.ikSetupPath = fileName;
        end
        function o = getIkSetupPath(obj)
            o = obj.ikSetupPath;
        end
        function setTrackingOrientationsFileName(obj,fileName)
            obj.trackingOrientationsFileName = fileName;
        end
        function getTrackingOrientationsFileName(obj)
            o = obj.trackingOrientationsFileName ;
        end
        function setVisualizeTracking(obj,b)
            obj.visualizeTracking = b;
        end
        function o = getVisualizeTracking(obj)
            o = obj.visualizeTracking();
        end
        function setOutputResultsFileName(obj, fileName)
            obj.outputResultsFileName = fileName;
        end
        function o = getOutputResultsFileName(obj)
            o = obj.outputResultsFileName;
        end
        function setIKTimeIntervalInMinutes(obj, stime, etime)
            t0 = seconds(minutes(fix((stime)))) + ((stime - fix((stime)))*100);
            tn = seconds(minutes(fix((etime)))) + ((etime - fix((etime)))*100);
            
            obj.ikStartTime = t0;
            obj.ikEndTime = tn;
        end
        function o = getIKStartTime(obj)
            o = obj.ikStartTime;
        end
        function o = getIKEndTime(obj)
            o = obj.ikEndTime;
        end
        function o = writeIKStudySetup2File(obj,ikStudyFileName)
           import org.modeling.opensim.*
           
           if isempty(obj.ikStudy)
              error('No IK solution has been generated, run runOrientationTracking()')
           end
           % Write the IK Study Setup to file. 
           obj.ikStudy.print(ikStudyFileName);
        end
        function setIKResultsDir(obj, dirName)
           obj.ikResultsDir = dirName;
        end
        function runOrientationTracking(obj)
            import org.opensim.modeling.*
            % Instantiate an IK Study
            ik = InverseKinematicsStudy();
            ik.set_orientations_file_name(obj.trackingOrientationsFileName)
            % Set time range
            imuData = STOFileAdapterQuaternion().readFile(obj.trackingOrientationsFileName);
            % Set the start and end times
            ik.setStartTime( obj.ikStartTime)
            ik.setEndTime(obj.ikEndTime)
            
            % Set the axis heading
            if obj.baseHeadingAxis().isXAxis
                trackingHeadingAxis = 'x';
            elseif obj.baseHeadingAxis().isYAxis
                trackingHeadingAxis = 'y';
            elseif obj.baseHeadingAxis().isZAxis
                trackingHeadingAxis = 'z';
            end
            % Set the base IMU
            ik.set_base_imu_label(obj.baseImuName)
            % set the heading axis
            ik.set_base_heading_axis(trackingHeadingAxis);
            % Change the model file
            ik.setModel(obj.calibratedModel);
            % Set the output directory Name
            ik.set_results_directory(obj.ikResultsDir)
            % Run IK
            ik.run(obj.visualizeTracking)
            % Save IKStudy Object for writing to file
            obj.ikStudy = ik;
        end
    end
end
    