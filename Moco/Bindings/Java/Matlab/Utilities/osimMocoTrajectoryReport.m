classdef osimMocoTrajectoryReport < handle
    methods
        function self = osimMocoTrajectoryReport(model, trajectoryFilepath, varargin)
            args = inputParser();
            args.addRequired('model');
            args.addRequired('trajectoryFilepath');
            args.addParameter('output', '');
            args.parse(model, trajectoryFilepath, varargin{:});
            import org.opensim.modeling.*;
            self.model = args.Results.model;
            self.model.initSystem();
            self.trajectoryFilepath = args.Results.trajectoryFilepath;
            self.trajectory = MocoTrajectory(self.trajectoryFilepath);

            self.time = self.trajectory.getTimeMat();
            [~, self.trajectoryFname, ~] = ...
                fileparts(self.trajectoryFilepath);
            if isempty(args.Results.output)
                self.output = sprintf('%s_report.pdf', ...
                    self.trajectoryFname);
                self.output_ps = sprintf('%s_report.ps', ...
                    self.trajectoryFname);
            else
                self.output = args.Results.output;
                self.output_ps = sprintf('%s.ps', args.Result.output);
            end
            if exist(self.output_ps, 'file')
                delete(self.output_ps);
            end

            self.numRows = floor(self.plotsPerPage / 3) + 1;

        end
        function pdfFilepath = generate(self)
            self.plotKinematics();
            pdfFilepath = self.output;
        end
    end
    methods (Access = private)
        function plotKinematics(self)
            % TODO: Use ordered dictionary.
            stateMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            stateLsMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            stateLabelMap = containers.Map('KeyType', 'char', 'ValueType', 'any');

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
                % TODO accels
                if ~stateLsMap.isKey(valueName)
                    stateLsMap(valueName) = {};
                end
                if ~stateLsMap.isKey(speedName)
                    stateLsMap(speedName) = {};
                end
                % TODO bilateral
                stateLsMap(valueName) = [stateLsMap(valueName), {'-'}];
                stateLsMap(speedName) = [stateLsMap(speedName), {'-'}];
                % stateLsMap(accelName) = [stateLsMap(accelName), {'-'}];

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
            end

            self.plotVariables('state', stateMap, stateLsMap, stateLabelMap);
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
                ymin = -inf;
                ymax =  inf;

                numConds = length(varMap(key));
                for icond = 1:numConds
                    path = paths{icond};
                    var = self.getVariable(varType, path);
                    ymin = min(ymin, min(var));
                    ymax = max(ymax, max(var));
                    % TODO handle refs

                    linestyle = linestyles{icond};
                    plot(self.time, var, linestyle, 'LineWidth', 1.5);
                end

                title(key, 'Interpreter', 'none');
                
                if ivar > self.plotsPerPage * floor(double(varMap.Count) / self.plotsPerPage)
                    % This is the last page.
                    plotsThisPage = varMap.Count - ...
                        nPagesPrinted * self.plotsPerPage;
                else
                    plotsThisPage = self.plotsPerPage;
                end
                if p > (plotsThisPage - self.numCols)
                    % Add xlabels for the bottom numCols plots on this
                    % page.
                    xlabel('time (s)');
                end
                ylabel(labelMap(key));
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
                error('TODO');
            elseif strcmp(type, 'slack')
                var = self.trajectory.getSlackMat(path);
            elseif strcmp(type, 'parameter')
                var = self.trajectory.getParameter(path);
            end
        end
        function ax = createSubplot(self, p)
            ax = subplot(self.numRows, self.numCols, p + self.numCols);
            pos = get(gca, 'Position');
            %pos(1) = 0.055;
            %pos(3) = 0.9;
            set(gca, 'Position', pos)
        end
    end
    methods (Static)
        function label = getLabelFromMotionType(motionType, level)
            label = '';
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
    end
    properties (SetAccess = private)
        % Input.
        model
        trajectoryFilepath
        trajectory
        % TODO bilateral
        % TODO refFiles
        trajectoryFname
        output
        output_ps

        time
        plotsPerPage = 15.0;
        numCols = 3;
        numRows
    end
end
