% TODO: add documentation
% reportFilepath = self.output;
% https://www.ghostscript.com/download/gsdnld.html      
% Refer to report.py (located in Moco/Bindings/Python in the source code)
% for additional documentation.
% TODO: adding a legend
% TODO reduce margin around figure.
% TODO: parameters
% Based on report.py.
% TODO use tiledlayout
% (https://www.mathworks.com/help/matlab/ref/tiledlayout.html).

% -------------------------------------------------------------------------- %
% OpenSim Moco: osimMocoTrajectoryReport.m                                   %
% -------------------------------------------------------------------------- %
% Copyright (c) 2019 Stanford University and the Authors                     %
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
classdef osimMocoTrajectoryReport < handle
    methods
        function self = osimMocoTrajectoryReport(model, trajectoryFilepath, ...
                varargin)
            import org.opensim.modeling.*;
            args = inputParser();
            args.addRequired('model', @(x) isa(x, 'org.opensim.modeling.Model'));
            args.addRequired('trajectoryFilepath', @ischar);
            args.addParameter('output', '', @ischar);
            args.addParameter('bilateral', false, @islogical);
            args.addParameter('refFiles', {}, @iscell);
            args.parse(model, trajectoryFilepath, varargin{:});
            self.model = args.Results.model;
            self.model.initSystem();
            self.trajectoryFilepath = args.Results.trajectoryFilepath;
            self.trajectory = MocoTrajectory(self.trajectoryFilepath);
            self.bilateral = args.Results.bilateral;

            self.time = self.trajectory.getTimeMat();
            [trajDir, self.trajectoryFname, ~] = ...
                fileparts(self.trajectoryFilepath);
            if isempty(args.Results.output)
                self.output = sprintf('%s_report.pdf', ...
                    self.trajectoryFname);
            else
                if ~endsWith(args.Result.output, '.pdf')
                    error('Expected output to end with .pdf');
                end
                self.output = args.Results.output;
            end
            if isempty(trajDir)
                trajDir = './';
            end
            self.output_ps = sprintf('%s.ps', tempname(trajDir));
            
            self.numRows = floor(self.plotsPerPage / 3) + 1;
            
            
            derivNames = self.trajectory.getDerivativeNames();
            for id = 0:(derivNames.size()-1)
                derivName = derivNames.get(id);
                if endsWith(derivName, '/accel')
                    self.accels = true;
                end
            end
            
            self.refs = {};
            for ir = 1:length(args.Results.refFiles)
                refFile = args.Results.refFiles{ir};
                [~, ~, ext] = fileparts(refFile);
                refTable = STOFileAdapter().read(refFile);
                ext = lower(ext);
                if strcmp(ext, '.sto') || strcmp(ext, '.mot')
                    if (refTable.hasTableMetaDataKey('inDegrees') && ...
                            strcmp(refTable.getTableMetaDataAsString('inDegrees'), 'yes'))
                        simEngine = self.model.getSimbodyEngine();
                        simEngine.convertDegreesToRadians(refTable);
                    end
                end
                ref = osimMocoTableToStruct(refTable);
                self.refs = [self.refs, {ref}];
            end
            
            allFiles = {};
            if ~isempty(args.Results.refFiles)
                allFiles = args.Results.refFiles;
            end
            allFiles = [allFiles, {self.trajectoryFilepath}];
            colormap parula;
            cmapSamples = linspace(0.7, 0, length(allFiles));
            cmapOrig = colormap();
            cmapCount = size(cmapOrig, 1);
            cmapOrigX = (0:(cmapCount - 1)) / (cmapCount - 1);
            self.cmap = interp1(cmapOrigX, cmapOrig, cmapSamples);

%         for sample, file in zip(self.cmap_samples, all_files):
%             color = self.cmap(sample)
%             import matplotlib.lines as mlines
%             if bilateral:
%                 r = mlines.Line2D([], [], ls='-', color=color, linewidth=3)
%                 self.legend_handles.append(r)
%                 self.legend_labels.append(file + ' (right leg)')
%                 l = mlines.Line2D([], [], ls='--', color=color, linewidth=3)
%                 self.legend_handles.append(l)
%                 self.legend_labels.append(file + ' (left leg)')
%             else:
%                 h = mlines.Line2D([], [], ls='-', color=color, linewidth=3)
%                 self.legend_handles.append(h)
%                 self.legend_labels.append(file)
%                 
        end
        function reportFilepath = generate(self)
            self.plotKinematics();
            self.plotActivations();
            self.plotNormalizedTendonForces();
            self.plotAuxiliaryDerivatives();
            self.plotControls();
            self.plotMultipliers();
            self.plotParameters();
            [status, ~] = system(sprintf('ps2pdf %s %s', self.output_ps, self.output));
            if status == 0
                delete(self.output_ps);
                reportFilepath = self.output;
            else
                reportFilepath = replace(self.output, '.pdf', '.ps');
                movefile(self.output_ps, reportFilepath);
            end
        end
    end
    methods (Access = private)
        function plotKinematics(self)
            % TODO: Use ordered dictionary.
            stateMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            stateLsMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            stateLabelMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            accelMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            accelLsMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            accelLabelMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            coordSet = self.model.getCoordinateSet();

            for i = 0:(coordSet.getSize() - 1)
                coord = coordSet.get(i);
                coordName = coord.getName();
                coordPath = coord.getAbsolutePathString();
                coordMotType = coord.getMotionType();
                % Append suffixes to create names for position and speed state
                % variables.
                valueName = sprintf('%s/value', coordName);
                speedName = sprintf('%s/speed', coordName);
                if self.accels
                    accelName = sprintf('%s/accel', coordName);
                end
                
                if self.bilateral
                    [valueName, stateLsMap] = ...
                        self.bilateralize(valueName, stateLsMap);
                    [speedName, stateLsMap] = ...
                        self.bilateralize(speedName, stateLsMap);
                    if self.accels
                        [accelName, accelLsMap] = ...
                            self.bilateralize(accelName, accelLsMap);
                    end
                else
                    if ~stateLsMap.isKey(valueName)
                        stateLsMap(valueName) = {};
                    end
                    if ~stateLsMap.isKey(speedName)
                        stateLsMap(speedName) = {};
                    end
                    stateLsMap(valueName) = [stateLsMap(valueName), {'-'}];
                    stateLsMap(speedName) = [stateLsMap(speedName), {'-'}];
                    if self.accels
                        if ~accelLsMap.isKey(accelName)
                            accelLsMap(accelName) = {};
                        end
                        accelLsMap(accelName) = [accelLsMap(accelName), {'-'}];
                    end
                end

                if ~stateMap.isKey(valueName)
                    stateMap(valueName) = {};
                end
                stateMap(valueName) = ...
                    [stateMap(valueName), sprintf('%s/value', coordPath)];
                stateLabelMap(valueName) = ...
                    self.getLabelFromMotionType(coordMotType, 'value');

                if ~stateMap.isKey(speedName)
                    stateMap(speedName) = {};
                end
                stateMap(speedName) = ...
                    [stateMap(speedName), sprintf('%s/speed', coordPath)];
                stateLabelMap(speedName) = ...
                    self.getLabelFromMotionType(coordMotType, 'speed');
                
                if self.accels
                    if ~accelMap.isKey(accelName)
                        accelMap(accelName) = {};
                    end
                    accelMap(accelName) = ...
                        [accelMap(accelName), sprintf('%s/accel', coordPath)];
                    accelLabelMap(accelName) = ...
                        self.getLabelFromMotionType(coordMotType, 'accel');
                end
            end

            self.plotVariables('state', stateMap, stateLsMap, stateLabelMap);
            if self.accels
                self.plotVariables('derivative', accelMap, accelLsMap, ...
                    accelLabelMap);
            end
        end
        function plotActivations(self)
            activMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            activLsMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            activLabelMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            stateNames = self.trajectory.getStateNames();
            for istate = 0:(stateNames.size()-1)
                stateName = char(stateNames.get(istate));
                if endsWith(stateName, '/activation')
                    title = stateName;
                    if self.bilateral
                        [title, activLsMap] = ...
                            self.bilateralize(title, activLsMap);
                    else
                        if ~activLsMap.isKey(title)
                            activLsMap(title) = {};
                        end
                        activLsMap(title) = ...
                            [activLsMap(title), {'-'}];
                    end
                    if ~activMap.isKey(title)
                        activMap(title) = {};
                    end
                    activMap(title) = ...
                        [activMap(title), stateName];
                    activLabelMap(title) = '';
                end
            end
            self.plotVariables('state', activMap, activLsMap, activLabelMap);
        end
        function plotNormalizedTendonForces(self)
            map = containers.Map('KeyType', 'char', 'ValueType', 'any');
            lsMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            labelMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            stateNames = self.trajectory.getStateNames();
            for istate = 0:(stateNames.size()-1)
                stateName = char(stateNames.get(istate));
                if endsWith(stateName, '/normalized_tendon_force')
                    title = stateName;
                    if self.bilateral
                        [title, lsMap] = self.bilateralize(title, lsMap);
                    else
                        if ~lsMap.isKey(title)
                            lsMap(title) = {};
                        end
                        lsMap(title) = [lsMap(title), {'-'}];
                    end
                    if ~map.isKey(title)
                        map(title) = {};
                    end
                    map(title) = [map(title), stateName];
                    labelMap(title) = '';
                end
            end
            self.plotVariables('state', map, lsMap, labelMap);
        end
        function plotAuxiliaryDerivatives(self)
            map = containers.Map('KeyType', 'char', 'ValueType', 'any');
            lsMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            labelMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            derivNames = self.trajectory.getDerivativeNames();
            auxDerivNames = {};
            for id = 0:(derivNames.size()-1)
                derivName = char(derivNames.get(id));
                if ~endsWith(derivName, '/accel')
                    auxDerivNames = [auxDerivNames, {derivName}];
                end
            end
            
            for iaux = 1:length(auxDerivNames)
                auxDerivName = auxDerivNames{iaux};
                title = auxDerivName;
                if self.bilateral
                    [title, lsMap] = self.bilateralize(title, lsMap);
                else
                    if ~lsMap.isKey(title)
                        lsMap(title) = {};
                    end
                    lsMap(title) = [lsMap(title), {'-'}];
                end
                if ~map.isKey(title)
                    map(title) = {};
                end
                map(title) = [map(title), auxDerivName];
                labelMap(title) = '';
            end
            self.plotVariables('derivative', map, lsMap, labelMap);
        end
        function plotControls(self)
            map = containers.Map('KeyType', 'char', 'ValueType', 'any');
            lsMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            labelMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            names = self.trajectory.getControlNames();
            for ic = 0:(names.size() - 1)
                name = char(names.get(ic));
                title = name;
                if self.bilateral
                    [title, lsMap] = self.bilateralize(title, lsMap);
                else
                    if ~lsMap.isKey(title)
                        lsMap(title) = {};
                    end
                    lsMap(title) = [lsMap(title), {'-'}];
                end
                if ~map.isKey(title)
                    map(title) = {};
                end
                map(title) = [map(title), name];
                labelMap(title) = '';
            end
            self.plotVariables('control', map, lsMap, labelMap);
        end
        function plotMultipliers(self)
            map = containers.Map('KeyType', 'char', 'ValueType', 'any');
            lsMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            labelMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            names = self.trajectory.getMultiplierNames();
            for ic = 0:(names.size() - 1)
                name = char(names.get(ic));
                title = name;
                if self.bilateral
                    [title, lsMap] = self.bilateralize(title, lsMap);
                else
                    if ~lsMap.isKey(title)
                        lsMap(title) = {};
                    end
                    lsMap(title) = [lsMap(title), {'-'}];
                end
                if ~map.isKey(title)
                    map(title) = {};
                end
                map(title) = [map(title), name];
                labelMap(title) = '';
            end
            self.plotVariables('multiplier', map, lsMap, labelMap);
        end
        function plotParameters(self)
            % TODO
        end
        function plotVariables(self, varType, varMap, lsMap, labelMap)
            nPagesPrinted = 0;
            
            %  Number of plots on the current page.
            p = 1;
            varMapKeys = varMap.keys();
            for ivar = 1:varMap.Count
                key = varMapKeys{ivar};
                paths = varMap(key);
                linestyles = lsMap(key);
                if mod(p, self.plotsPerPage) == 1
                    fig = self.createFigure();
                end
                % Skip the first row, which is used for the legend.
                ax = self.createSubplot(p);
                ymin =  inf;
                ymax = -inf;

                numConds = length(varMap(key));
                for icond = 1:numConds
                    path = paths{icond};
                    var = self.getVariable(varType, path);
                    ymin = min(ymin, min(var));
                    ymax = max(ymax, max(var));
                    for ir = 1:length(self.refs)
                        ref = self.refs{ir};
                        colLabel = path;
                        % Copied from osimMocoTableToStruct.
                        if ~isvarname(colLabel)
                            % Find any non-alphanumeric characters and replace with '_'
                            colLabel(~(isstrprop(colLabel, 'alphanum'))) = '_';
                            % Check if first character is a letter, and append 'unlabeled' if not
                            if ~(isletter(colLabel(1)))
                                colLabel = colLabel(2:end);
                            end
                        end
                        if ~isempty(find(strcmp(fieldnames(ref), colLabel), 1))
                            init = self.getIndexForNearestValue(ref.time, self.time(1));
                            final = self.getIndexForNearestValue(ref.time, self.time(end));
                            y = ref.(colLabel)(init:final);
                            plot(ref.time(init:final), y, linestyles{icond}, ...
                                'LineWidth', 2.5, 'Color', self.cmap(ir, :)); 
                            ymin = min(ymin, min(y));
                            ymax = max(ymax, max(y));
                        end
                    end
                    linestyle = linestyles{icond};
                    plot(self.time, var, linestyle, 'LineWidth', 1.5, ...
                            'Color', self.cmap(end, :));
                end

                set(ax, 'fontsize', 6);
                htitle = title(key, 'Interpreter', 'none', 'fontsize', 10);
                if length(key) > 45
                    htitle.FontSize = 6;
                elseif length(key) > 40
                    htitle.FontSize = 7;
                elseif length(key) > 35
                    htitle.FontSize = 8;
                end
                
                if ivar > self.plotsPerPage * ...
                        floor(double(varMap.Count) / self.plotsPerPage)
                    % This is the last page.
                    plotsThisPage = varMap.Count - ...
                        nPagesPrinted * self.plotsPerPage;
                else
                    plotsThisPage = self.plotsPerPage;
                end
                if p > (plotsThisPage - self.numCols)
                    % Add xlabels for the bottom numCols plots on this
                    % page.
                    xlabel('time (s)', 'fontsize', 8);
                end
                ylabel(labelMap(key), 'fontsize', 8);
                xlim([self.time(1), self.time(end)]);
                if 0 <= ymin && ymax <= 1
                    ylim([0, 1]);
                end

                if (mod(p, self.plotsPerPage) == 0 || ivar == varMap.Count)
                    print(fig, '-dpsc', '-append', self.output_ps);
                    close(fig);
                    p = 1;
                    nPagesPrinted = nPagesPrinted + 1;
                else
                    p = p + 1;
                end

            end

        end
        function var = getVariable(self, type, path)
            if strcmp(type, 'state')
                var = self.trajectory.getStateMat(path);
            elseif strcmp(type, 'control')
                var = self.trajectory.getControlMat(path);
            elseif strcmp(type, 'multiplier')
                var = self.trajectory.getMultiplierMat(path);
            elseif strcmp(type, 'derivative')
                % var = self.trajectory.getDerivativeMat(path);
                derivsTraj = self.trajectory.getDerivativesTrajectory();
                derivNames = self.trajectory.getDerivativeNames();
                icol = 0;
                for id = 0:(derivNames.size() - 1)
                    if strcmp(char(derivNames.get(id)), path)
                        icol = id;
                    end
                end
                n = length(self.time);
                var = zeros(n, 1);
                for irow = 0:(n - 1)
                    var(irow + 1) = derivsTraj.get(irow, icol);
                end
            elseif strcmp(type, 'slack')
                var = self.trajectory.getSlackMat(path);
            elseif strcmp(type, 'parameter')
                var = self.trajectory.getParameter(path);
            end
        end
        function ax = createSubplot(self, p)
            ax = subplot(self.numRows, self.numCols, p + self.numCols);
            hold on;
            % pos = get(gca, 'Position');
            % pos(1) = 0.055;
            % pos(3) = 0.9;
            % set(gca, 'Position', pos)
        end
    end
    methods (Static)
        function [newName, lsMap] = bilateralize(name, lsMap)
            newName = regexprep(name, '_r(/|_|$)', '$1');
            if ~strcmp(newName, name)
                if ~lsMap.isKey(newName)
                    lsMap(newName) = {};
                end
                lsMap(newName) = [lsMap(newName), {'-'}];
            else
                newName = regexprep(name, '_l(/|_|$)', '$1');
                if ~lsMap.isKey(newName)
                    lsMap(newName) = {};
                end
                lsMap(newName) = [lsMap(newName), {'--'}];
            end
        end
        function label = getLabelFromMotionType(motionType, level)
            if strcmp(motionType, 'Rotational')
                if strcmp(level, 'value')
                    label = 'angle (rad)';
                elseif strcmp(level, 'speed')
                    label = 'speed (rad/s)';
                elseif strcmp(level, 'accel')
                    label = 'accel. (rad/s^2)';
                else
                    label = 'rotate';
                end
            elseif strcmp(motionType, 'Translational')
                if strcmp(level, 'value')
                    label = 'position (m)';
                elseif strcmp(level, 'speed')
                    label = 'speed (m/s)';
                elseif strcmp(level, 'accel')
                    label = 'accel. (m/s^2)';
                else
                    label = 'translate';
                end
            elseif strcmp(motionType, 'Coupled')
                label = 'coupled';
            else
                label = 'undefined';
            end
        end
        function fig = createFigure()
            fig = figure('Visible', 'off');
            set(fig, 'PaperUnits', 'inches');
            set(fig, 'PaperSize', [8.5 11]);
            set(fig, 'PaperPositionMode', 'manual');
            set(fig, 'PaperPosition', [0 0 8.5 11]);
        end
        function index = getIndexForNearestValue(vec, val)
            [~, index] = min(abs(vec - val));
        end
    end
    properties (SetAccess = private)
        % Input.
        model
        trajectoryFilepath
        trajectory
        bilateral
        trajectoryFname
        output
        output_ps
        accels
        refs

        time
        plotsPerPage = 15.0;
        numCols = 3;
        numRows
        cmap
    end
end
