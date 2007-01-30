function varargout = ExcitationGUI(varargin)

% EXCITATIONGUI Application M-file for ExcitationGUI.fig

% -------------------------------------------------------------------------
% 

if nargin == 0  % LAUNCH GUI
    
	fig = openfig(mfilename,'reuse');
    
	% Use system color scheme for figure:
	set(fig,'Color',get(0,'defaultUicontrolBackgroundColor'));

	% Generate a structure of handles to pass to callbacks, and store it. 
	handles = guihandles(fig);
	guidata(fig, handles);
    
	if nargout > 0
		varargout{1} = fig;
	end
    
elseif ischar(varargin{1}) % INVOKE NAMED SUBFUNCTION OR CALLBACK

	try
		if (nargout)
			[varargout{1:nargout}] = feval(varargin{:}); % FEVAL switchyard
		else
			feval(varargin{:}); % FEVAL switchyard
		end
	catch
		disp(lasterr);
	end

end

% ------------------------------------------------------------------------- 

function varargout = logoPushbutton_Callback(h, eventdata, handles, varargin)

% Open the "About ExcitationGUI" message box window when the lab logo push
% button is used.

% Call the imageOfLabLogo function to use as custom message box icon.
logo = imageOfLabLogo;

% Define the message box window contents.
CreateStruct.WindowStyle='modal';
CreateStruct.Interpreter='tex';
msgbox({'Developed by Jeff Reinbolt at the Neuromuscular Biomechanics Laboratory at Stanford University.  Contains software licensed from The Mathworks, Inc. (Natick, MA).'
        ''
        'Copyright \copyright 2006.  All rights reserved.'
        ''
        'Warning: This computer program is protected by copyright law and international treaties.  Unauthorized reproduction or distribution of this program, or any portion of it, may result in severe civil and criminal penalties, and will be prosecuted to the maximum extent possible under the law.'}, ...
        'About ExcitationGUI', ...
        'custom', ...
        logo, ...
        [], ...
        CreateStruct)

% ------------------------------------------------------------------------- 

function varargout = initializePushbutton_Callback(h, eventdata, handles, varargin)

% Open a question dialog box to determine initial input from a saved file
% or default when the initialize push button is used.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes
global m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global t tt tmax NumControl FPS ForceZeroSlope Range default_nodes exc_range

% Define node and spline parameters.
tmax = 100 ;         % final time in seconds
NumControl = 11 ;    % number of control nodes
FPS = 1 ;            % frames/second
ForceZeroSlope = 0 ; % force spline slope to zero
Range = [0 1] ;      % y-axis limits

exc_range = 0.35;

% Define the node and spline time vectors.
t = linspace(0, tmax, NumControl)';
tt = linspace(0, tmax, (tmax+1)*FPS)';

% Determine intial input from saved file or default using a question dialog
% box.
init_reply = questdlg('How would you like to initialize the "Excitations"?', ...
                      'Initialize Confirmation',...
                      'Default', 'File', 'Cancel', 'Default');

% Switch among several cases based on user response for initialization.
switch init_reply,
    
    case 'Default'
        
        % Define cubic splines
        default_value = 0;
        default_nodes = default_value * ones(length(t), 12);
        m1Nodes = default_nodes(:, 1); % Initial curve for m1
        m2Nodes = default_nodes(:, 2); % Initial curve for m2
        m3Nodes = default_nodes(:, 3); % Initial curve for m3
        m4Nodes = default_nodes(:, 4); % Initial curve for m4
        m5Nodes = default_nodes(:, 5); % Initial curve for m5
        m6Nodes = default_nodes(:, 6); % Initial curve for m6
        m7Nodes = default_nodes(:, 7); % Initial curve for m7
        m8Nodes = default_nodes(:, 8); % Initial curve for m8
        m9Nodes = default_nodes(:, 9); % Initial curve for m9
        m10Nodes = default_nodes(:, 10); % Initial curve for m10
        m11Nodes = default_nodes(:, 11); % Initial curve for m11
        m12Nodes = default_nodes(:, 12); % Initial curve for m12
        
        default_nodes = [  0.99545454545455   0.99230769230769   0.94615384615385   0.96153846153846   0.96153846153846   0.00769230769231   0.99545454545455   0.99230769230769   0.94615384615385   0.96153846153846   0.96153846153846   0.00769230769231
                           0.72272727272727   0.96153846153846   0.94615384615385   0.71538461538461   0.73076923076923   0.14615384615385   0.72272727272727   0.96153846153846   0.94615384615385   0.71538461538461   0.73076923076923   0.14615384615385
                                          0   0.43846153846154   0.53076923076923                  0                  0   0.74615384615385                  0   0.43846153846154   0.53076923076923                  0                  0   0.74615384615385
                                          0                  0                  0                  0                  0   0.96153846153846                  0                  0                  0                  0                  0   0.96153846153846
                           0.25909090909091                  0                  0                  0                  0   0.77692307692308   0.25909090909091                  0                  0                  0                  0   0.77692307692308
                           0.91538461538462                  0                  0                  0                  0   0.19230769230769   0.91538461538462                  0                  0                  0                  0   0.19230769230769
                           0.23181818181818                  0                  0                  0   0.40769230769231                  0   0.23181818181818                  0                  0                  0   0.40769230769231                  0
                                          0                  0                  0   0.16153846153846   0.83846153846154                  0                  0                  0                  0   0.16153846153846   0.83846153846154                  0
                                          0   0.16153846153846   0.25384615384615   0.62307692307692   0.96153846153846                  0                  0   0.16153846153846   0.25384615384615   0.62307692307692   0.96153846153846                  0
                           0.27727272727273   0.59230769230769   0.66923076923077   0.86923076923077   0.94615384615385                  0   0.27727272727273   0.59230769230769   0.66923076923077   0.86923076923077   0.94615384615385                  0
                           0.97727272727273   0.93076923076923   0.94615384615385   0.96153846153846   0.96153846153846                  0   0.97727272727273   0.93076923076923   0.94615384615385   0.96153846153846   0.96153846153846                  0
                        ];
        m1Nodes = default_nodes(:, 1); % Initial curve for m1
        m2Nodes = default_nodes(:, 2); % Initial curve for m2
        m3Nodes = default_nodes(:, 3); % Initial curve for m3
        m4Nodes = default_nodes(:, 4); % Initial curve for m4
        m5Nodes = default_nodes(:, 5); % Initial curve for m5
        m6Nodes = default_nodes(:, 6); % Initial curve for m6
        m7Nodes = default_nodes(:, 7); % Initial curve for m7
        m8Nodes = default_nodes(:, 8); % Initial curve for m8
        m9Nodes = default_nodes(:, 9); % Initial curve for m9
        m10Nodes = default_nodes(:, 10); % Initial curve for m10
        m11Nodes = default_nodes(:, 11); % Initial curve for m11
        m12Nodes = default_nodes(:, 12); % Initial curve for m12
        
        default_value = 0;
        default_nodes = default_value * ones(length(t), 12);
        
        set(handles.symmetryCheckbox, 'Value', 0)
        set(handles.quadsCheckbox, 'Value', 0)
        
    case 'File'
        d = dir('*excitation_nodes.txt');
        fname = [];
        if ~isempty(d),
            fname = d.name;
        end
        if exist(fname) == 2,
            
            default_value = 0;
            default_nodes = default_value * ones(length(t), 12);
        
            % Load nodal points.
            nodes = load(fname);
            m1Nodes = nodes(:, 1);
            m2Nodes = nodes(:, 2);
            m3Nodes = nodes(:, 3);
            m4Nodes = nodes(:, 4);
            m5Nodes = nodes(:, 5);
            m6Nodes = nodes(:, 6);
            m7Nodes = nodes(:, 7);
            m8Nodes = nodes(:, 8);
            m9Nodes = nodes(:, 9);
            m10Nodes = nodes(:, 10);
            m11Nodes = nodes(:, 11);
            m12Nodes = nodes(:, 12);
            
            set(handles.symmetryCheckbox, 'Value', 0)
            set(handles.quadsCheckbox, 'Value', 0)
        
        else,
            beep
            errordlg({'The initialization input file does not exist!'
                      ''
                      '   1. Initialize "Excitations" using the "Default" button.'
                      '   2. Change "Excitations" using the mouse.'
                      '   3. Record "Excitations" using the "Save" button.'
                      '   4. Initialize "Excitations" using the "File" button.'}, ...
                      'File Error', 'modal')
            return
        end

    case 'Cancel'
        return
    
end

% Define and plot the m1 curve.
axes(handles.m1Axes)
plotSpline(m1Nodes)

% Define and plot the m2 curve.
axes(handles.m2Axes)
plotSpline(m2Nodes)

% Define and plot the m3 curve.
axes(handles.m3Axes)
plotSpline(m3Nodes)

% Define and plot the m4 curve.
axes(handles.m4Axes)
plotSpline(m4Nodes)

% Define and plot the m5 curve.
axes(handles.m5Axes)
plotSpline(m5Nodes)

% Define and plot the m6 curve.
axes(handles.m6Axes)
plotSpline(m6Nodes)

% Define and plot the m7 curve.
axes(handles.m7Axes)
plotSpline(m7Nodes)

% Define and plot the m8 curve.
axes(handles.m8Axes)
plotSpline(m8Nodes)

% Define and plot the m9 curve.
axes(handles.m9Axes)
plotSpline(m9Nodes)

% Define and plot the m10 curve.
axes(handles.m10Axes)
plotSpline(m10Nodes)

% Define and plot the m11 curve.
axes(handles.m11Axes)
plotSpline(m11Nodes)

% Define and plot the m12 curve.
axes(handles.m12Axes)
plotSpline(m12Nodes)

% ------------------------------------------------------------------------- 

function varargout = savePushbutton_Callback(h, eventdata, handles, varargin)

% Save the user inputs to disk.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m4Axes_ButtonDownFcn,
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes
global m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global tmax NumControl FPS ForceZeroSlope Range default_nodes exc_range

CurveFlag = 1;

% Check if m1Nodes, m2Nodes, m3Nodes, m4Nodes, m5Nodes, and m6Nodes are not
% empty. If they are empty, then present the user with an error dialog box
% (see below).
if ~isempty(m1Nodes) & ~isempty(m2Nodes) & ~isempty(m3Nodes) & ~isempty(m4Nodes) & ~isempty(m5Nodes) & ~isempty(m6Nodes) & ~isempty(m7Nodes) & ~isempty(m8Nodes) & ~isempty(m9Nodes) & ~isempty(m10Nodes) & ~isempty(m11Nodes) & ~isempty(m12Nodes),
    
    % Define matrix of nodes (size = numNodes x 12).
	nodes = [m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes];
         
    % Read subject parameters
    d = dir('*_params.txt');
    fname = [];
    if ~isempty(d),
        fname = d.name;
    end
    if exist(fname) == 2,
        fid = fopen(fname, 'r');
        [labels, parameters] = textread(fname, '%s %s');
        subjectID = char(parameters(1));
        cycletimes = str2double(parameters(2:3));
        limb = char(parameters(4));
        step1_OTO = str2double(parameters(5));
        step1_OIC = str2double(parameters(6));
        fclose(fid);
    else,
        subjectID = [];
        cycletimes = [100 100];
        limb = 'L';
        step1_OTO = 100;
        step1_OIC = 100;
    end
    index0sh_2 = tmax - (step1_OIC/100)*tmax + 1;   
    index100sh_2 = tmax - (step1_OIC/100)*tmax; 
    index100_2 = tmax+1;
    
    % Define the node and spline time vectors.
    t = linspace(0, tmax, NumControl)';
    tt = linspace(0, tmax, (tmax+1)*FPS)';
    
    % Save the matrix of nodes to disk.         
	save([subjectID '_excitation_nodes.txt'], 'nodes' ,'-ascii', '-double')
    
    % Save step function of nodes to disk for input into cmcparams.txt
    muscles = {'rect_fem_l'
               'vas_lat_l'
               'vas_int_l'
               'vas_med_l'
               'semimem_l'
               'semiten_l'
               'bifemlh_l'
               'bifemsh_l'
               'tib_ant_l'
               'med_gas_l'
               'lat_gas_l'
               'soleus_l'
               'rect_fem_r'
               'vas_lat_r'
               'vas_int_r'
               'vas_med_r'
               'semimem_r'
               'semiten_r'
               'bifemlh_r'
               'bifemsh_r'
               'tib_ant_r'
               'med_gas_r'
               'lat_gas_r'
               'soleus_r'};
    nmuscles = length(muscles);
    if CurveFlag,
        nnodes = length(tt);
        step_size = 100 / (nnodes - 1);
    else,
        nnodes = length(t);
        step_size = 100 / (nnodes - 1);
    end
    fid = fopen([subjectID '_excitation_ranges.txt'], 'wt');
    for m = 1:nmuscles,
        switch m
            case(1)
                points = nodes(:, 1);
                default_points = default_nodes(:, 1);
            case(2)
                points = nodes(:, 2);
                default_points = default_nodes(:, 2);
            case{3}
                points = (nodes(:, 2) + nodes(:, 3)) / 2;
            case(4)
                points = nodes(:, 3);
                default_points = default_nodes(:, 3);
            case{5, 6, 7, 8}
                points = nodes(:, 4);
                default_points = default_nodes(:, 4);
            case(9)
                points = nodes(:, 5);
                default_points = default_nodes(:, 5);
            case{10, 11, 12}
                points = nodes(:, 6);
                default_points = default_nodes(:, 6);
            case(13)
                points = nodes(:, 7);
                default_points = default_nodes(:, 7);
            case(14)
                points = nodes(:, 8);
                default_points = default_nodes(:, 8);
            case{15}
                points = (nodes(:, 8) + nodes(:, 9)) / 2;
            case(16)
                points = nodes(:, 9);
                default_points = default_nodes(:, 9);
            case{17, 18, 19, 20}
                points = nodes(:, 10);
                default_points = default_nodes(:, 10);
            case(21)
                points = nodes(:, 11);
                default_points = default_nodes(:, 11);
            case{22, 23, 24}
                points = nodes(:, 12);
                default_points = default_nodes(:, 12);
        end
        if ForceZeroSlope
            cs = spline(t,[0; points; 0]);
        else
            cs = spline(t,points);
        end
        curve = ppval(cs,tt);
        
        changes = points-default_points;
        
        if CurveFlag,
            points = curve;
        end
        
        if mean(points) ~= 0,
            if upper(limb) == 'L',
                if m <= (nmuscles/2),
                    tinitial = 0;
                    for n = 1:(nnodes-1),
                        minExc = points(n) - exc_range;
                        if minExc <= 0.01,
                            minExc = 0.01;
                        end
                        maxExc = points(n) + exc_range;
                        if maxExc <= minExc,
                            maxExc = minExc + exc_range;
                        end
                        if maxExc > 1,
                            maxExc = 1.0;
                        end
                        if minExc >= maxExc,
                            minExc = maxExc - exc_range;
                        end
                        minTime = tinitial;
                        maxTime = tinitial + step_size;
                        if maxTime > 100,
                            maxTime = 100;
                        end
                        fprintf(fid, '%s %.16f %.16f %.16f %.16f\n', char(muscles(m)), minExc, maxExc, (minTime*(cycletimes(1)/100)), (maxTime*(cycletimes(1)/100)));
                        tinitial = maxTime;
                    end
                elseif m > (nmuscles/2),
                    % Shift right leg data
                    
                    if CurveFlag,
                        curve_sync = [curve(index0sh_2:index100_2); curve(1:index100sh_2)];
                        points = curve_sync;
                    else
                        % Evaluate piecewise cubic spline
                        if ForceZeroSlope
                            cs = spline(t,[0; points; 0]);
                        else
                            cs = spline(t,points);
                        end
                        yy = ppval(cs,tt);

                        % Shift data for step 2
                        yy_sync = [yy(index0sh_2:index100_2); yy(1:index100sh_2)];
                        pp = spline(tt, yy_sync);
                        points = ppval(pp, t);
                    end
                    
                    tinitial = 0;
                    for n = 1:(nnodes-1),
                        minExc = points(n) - exc_range;
                        if minExc <= 0.01,
                            minExc = 0.01;
                        end
                        maxExc = points(n) + exc_range;
                        if maxExc <= minExc,
                            maxExc = minExc + exc_range;
                        end
                        if maxExc > 1,
                            maxExc = 1.0;
                        end
                        if minExc >= maxExc,
                            minExc = maxExc - exc_range;
                        end
                        minTime = tinitial;
                        maxTime = tinitial + step_size;
                        if maxTime > 100,
                            maxTime = 100;
                        end
                        fprintf(fid, '%s %.16f %.16f %.16f %.16f\n', char(muscles(m)), minExc, maxExc, (minTime*(cycletimes(1)/100)), (maxTime*(cycletimes(1)/100)));
                        tinitial = maxTime;
                    end                    
                end
            elseif upper(limb) == 'R'
                if m <= (nmuscles/2),
                    % Shift left leg data
                    
                    if CurveFlag,
                        curve_sync = [curve(index0sh_2:index100_2); curve(1:index100sh_2)];
                        points = curve_sync;
                    else
                        % Evaluate piecewise cubic spline
                        if ForceZeroSlope
                            cs = spline(t,[0; points; 0]);
                        else
                            cs = spline(t,points);
                        end
                        yy = ppval(cs,tt);

                        % Shift data for step 2
                        yy_sync = [yy(index0sh_2:index100_2); yy(1:index100sh_2)];
                        pp = spline(tt, yy_sync);
                        points = ppval(pp, t);
                    end
                    
                    tinitial = 0;
                    for n = 1:(nnodes-1),
                        minExc = points(n) - exc_range;
                        if minExc <= 0.01,
                            minExc = 0.01;
                        end
                        maxExc = points(n) + exc_range;
                        if maxExc <= minExc,
                            maxExc = minExc + exc_range;
                        end
                        if maxExc > 1,
                            maxExc = 1.0;
                        end
                        if minExc >= maxExc,
                            minExc = maxExc - exc_range;
                        end
                        minTime = tinitial;
                        maxTime = tinitial + step_size;
                        if maxTime > 100,
                            maxTime = 100;
                        end
                        fprintf(fid, '%s %.16f %.16f %.16f %.16f\n', char(muscles(m)), minExc, maxExc, (minTime*(cycletimes(1)/100)), (maxTime*(cycletimes(1)/100)));
                        tinitial = maxTime;
                    end
                elseif m > (nmuscles/2),
                    tinitial = 0;
                    for n = 1:(nnodes-1),
                        minExc = points(n) - exc_range;
                        if minExc <= 0.01,
                            minExc = 0.01;
                        end
                        maxExc = points(n) + exc_range;
                        if maxExc <= minExc,
                            maxExc = minExc + exc_range;
                        end
                        if maxExc > 1,
                            maxExc = 1.0;
                        end
                        if minExc >= maxExc,
                            minExc = maxExc - exc_range;
                        end
                        minTime = tinitial;
                        maxTime = tinitial + step_size;
                        if maxTime > 100,
                            maxTime = 100;
                        end
                        fprintf(fid, '%s %.16f %.16f %.16f %.16f\n', char(muscles(m)), minExc, maxExc, (minTime*(cycletimes(1)/100)), (maxTime*(cycletimes(1)/100)));
                        tinitial = maxTime;
                    end                    
                end                
            end
        end
    end
    fclose(fid);
    
% Open error dialog box if m1Nodes, m2Nodes, m3Nodes, m4Nodes, m5Nodes, and m6Nodes
% are empty. 
else
    beep
    errordlg({'The input excitations do not exist!'
              ''
              '   1. Generate "Excitations" using the "Initialize" button.'
              '   2. Change "Excitations" using the mouse.'
              '   3. Record "Excitations" using the "Save" button.'},...
              'Save Error', 'modal')
end % End of if m1Nodes, m2Nodes, m3Nodes, m4Nodes, m5Nodes, and m6Nodes are not empty.

% ------------------------------------------------------------------------- 

function varargout = quitPushbutton_Callback(h, eventdata, handles, varargin)

% Quit the ExcitationGUI program.

% Open a question dialog window to confirm the quit request.
quit_reply = questdlg('Do you really want to quit the application?', ...
                      'Quit Confirmation');
     
% Check if the quit request was confirmed.                      
if strcmp(quit_reply,'Yes'),
    
    % Close the ExcitationGUI window.
    close(gcf); 

% If the quit request was not confirmed, return to program.
else,
    return
end % End of if the quit request was confirmed. 

% ------------------------------------------------------------------------- 

function varargout = symmetryCheckbox_Callback(h, eventdata, handles, varargin)

% Accept change to the symmetry check box.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

% Check if inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    % Determine if symmetry box is being checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
    
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m1Nodes = mean([m1Nodes m2Nodes m3Nodes m7Nodes m8Nodes m9Nodes], 2); axes(handles.m1Axes); plotSpline(m1Nodes)
            m2Nodes = m1Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
            m3Nodes = m1Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
            m7Nodes = m1Nodes; axes(handles.m7Axes); plotSpline(m7Nodes)
            m8Nodes = m1Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
            m9Nodes = m1Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        else,
            m1Nodes = mean([m1Nodes m7Nodes], 2); axes(handles.m1Axes); plotSpline(m1Nodes)
            m2Nodes = mean([m2Nodes m8Nodes], 2); axes(handles.m2Axes); plotSpline(m2Nodes)
            m3Nodes = mean([m3Nodes m9Nodes], 2); axes(handles.m3Axes); plotSpline(m3Nodes)
            m7Nodes = m1Nodes; axes(handles.m7Axes); plotSpline(m7Nodes)
            m8Nodes = m2Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
            m9Nodes = m3Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        end

        m4Nodes = mean([m4Nodes m10Nodes], 2); axes(handles.m4Axes); plotSpline(m4Nodes)
        m5Nodes = mean([m5Nodes m11Nodes], 2); axes(handles.m5Axes); plotSpline(m5Nodes)
        m6Nodes = mean([m6Nodes m12Nodes], 2); axes(handles.m6Axes); plotSpline(m6Nodes)
        m10Nodes = m4Nodes; axes(handles.m10Axes); plotSpline(m10Nodes)
        m11Nodes = m5Nodes; axes(handles.m11Axes); plotSpline(m11Nodes)
        m12Nodes = m6Nodes; axes(handles.m12Axes); plotSpline(m12Nodes)
        
    end
    
else,
    
	set(handles.symmetryCheckbox, 'Value', 0);

end

% ------------------------------------------------------------------------- 

function varargout = quadsCheckbox_Callback(h, eventdata, handles, varargin)

% Accept change to the symmetry check box.
global m1Nodes m2Nodes m3Nodes m7Nodes m8Nodes m9Nodes

% Check if inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    % Determine if quads box is being checked
    if get(handles.quadsCheckbox, 'Value') == 1,

        % Determine if symmetry box is checked
        if get(handles.symmetryCheckbox, 'Value') == 1,
            m1Nodes = mean([m1Nodes m2Nodes m3Nodes m7Nodes m8Nodes m9Nodes], 2); axes(handles.m1Axes); plotSpline(m1Nodes)
            m2Nodes = m1Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
            m3Nodes = m1Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
            m7Nodes = m1Nodes; axes(handles.m7Axes); plotSpline(m7Nodes)
            m8Nodes = m1Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
            m9Nodes = m1Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        else,
            m1Nodes = mean([m1Nodes m2Nodes m3Nodes], 2); axes(handles.m1Axes); plotSpline(m1Nodes)
            m2Nodes = m1Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
            m3Nodes = m1Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
            m7Nodes = mean([m7Nodes m8Nodes m9Nodes], 2); axes(handles.m7Axes); plotSpline(m7Nodes)
            m8Nodes = m7Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
            m9Nodes = m7Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        end
        
    end
    
else,
    
    set(handles.quadsCheckbox, 'Value', 0);
    
end

% ------------------------------------------------------------------------- 

function replot1(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m1Axes); plotSpline(m1Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m7Nodes = m1Nodes; axes(handles.m7Axes); plotSpline(m7Nodes)
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m2Nodes = m1Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
            m3Nodes = m1Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
            m8Nodes = m1Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
            m9Nodes = m1Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        end
        
    else,
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m2Nodes = m1Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
            m3Nodes = m1Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
        end        
        
    end

% ------------------------------------------------------------------------- 

function varargout = m1PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m1Nodes = m1Nodes+0.05;
    m1Nodes(find(m1Nodes < Range(1))) = Range(1);
    m1Nodes(find(m1Nodes > Range(2))) = Range(2);
    replot1(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m1MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m1Nodes = m1Nodes-0.05;
    m1Nodes(find(m1Nodes < Range(1))) = Range(1);
    m1Nodes(find(m1Nodes > Range(2))) = Range(2);
    replot1(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m1Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m1Nodes m2Nodes m3Nodes m7Nodes m8Nodes m9Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m1 spline curve.
    m1Nodes = updateSpline(m1Nodes);
    replot1(handles)
    
end

% ------------------------------------------------------------------------- 

function replot2(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m2Axes); plotSpline(m2Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m8Nodes = m2Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m1Nodes = m2Nodes; axes(handles.m1Axes); plotSpline(m1Nodes)
            m3Nodes = m2Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
            m7Nodes = m2Nodes; axes(handles.m7Axes); plotSpline(m7Nodes)
            m9Nodes = m2Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        end
        
    else,
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m1Nodes = m2Nodes; axes(handles.m1Axes); plotSpline(m1Nodes)
            m3Nodes = m2Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
        end        
        
    end

% ------------------------------------------------------------------------- 

function varargout = m2PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m2Nodes = m2Nodes+0.05;
    m2Nodes(find(m2Nodes < Range(1))) = Range(1);
    m2Nodes(find(m2Nodes > Range(2))) = Range(2);
    replot2(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m2MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m2Nodes = m2Nodes-0.05;
    replot2(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m2Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m1Nodes m2Nodes m3Nodes m7Nodes m8Nodes m9Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m2 spline curve.
    m2Nodes = updateSpline(m2Nodes);
    replot2(handles)
    
end

% ------------------------------------------------------------------------- 

function replot3(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m3Axes); plotSpline(m3Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m9Nodes = m3Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m1Nodes = m3Nodes; axes(handles.m1Axes); plotSpline(m1Nodes)
            m2Nodes = m3Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
            m7Nodes = m3Nodes; axes(handles.m7Axes); plotSpline(m7Nodes)
            m8Nodes = m3Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
        end
        
    else,
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m1Nodes = m3Nodes; axes(handles.m1Axes); plotSpline(m1Nodes)
            m2Nodes = m3Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
        end        
        
    end

% ------------------------------------------------------------------------- 

function varargout = m3PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m3Nodes = m3Nodes+0.05;
    m3Nodes(find(m3Nodes < Range(1))) = Range(1);
    m3Nodes(find(m3Nodes > Range(2))) = Range(2);
    replot3(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m3MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m3Nodes = m3Nodes-0.05;
    m3Nodes(find(m3Nodes < Range(1))) = Range(1);
    m3Nodes(find(m3Nodes > Range(2))) = Range(2);
    replot3(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m3Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m1Nodes m2Nodes m3Nodes m7Nodes m8Nodes m9Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    % Update the m3 spline curve.
    m3Nodes = updateSpline(m3Nodes);
    replot3(handles);
    
end

% ------------------------------------------------------------------------- 

function replot4(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m4Axes); plotSpline(m4Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m10Nodes = m4Nodes;
        axes(handles.m10Axes);
        plotSpline(m10Nodes)
    end

% ------------------------------------------------------------------------- 

function varargout = m4PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m4Nodes = m4Nodes+0.05;
    m4Nodes(find(m4Nodes < Range(1))) = Range(1);
    m4Nodes(find(m4Nodes > Range(2))) = Range(2);
    replot4(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m4MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m4Nodes = m4Nodes-0.05;
    m4Nodes(find(m4Nodes < Range(1))) = Range(1);
    m4Nodes(find(m4Nodes > Range(2))) = Range(2);
    replot4(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m4Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m4Nodes m10Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m4 spline curve.
    m4Nodes = updateSpline(m4Nodes);
    replot4(handles)
    
end

% ------------------------------------------------------------------------- 

function replot5(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m5Axes); plotSpline(m5Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m11Nodes = m5Nodes;
        axes(handles.m11Axes);
        plotSpline(m11Nodes)
    end

% ------------------------------------------------------------------------- 

function varargout = m5PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m5Nodes = m5Nodes+0.05;
    m5Nodes(find(m5Nodes < Range(1))) = Range(1);
    m5Nodes(find(m5Nodes > Range(2))) = Range(2);
    replot5(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m5MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m5Nodes = m5Nodes-0.05;
    m5Nodes(find(m5Nodes < Range(1))) = Range(1);
    m5Nodes(find(m5Nodes > Range(2))) = Range(2);
    replot5(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m5Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m5Nodes m11Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m5 spline curve.
    m5Nodes = updateSpline(m5Nodes);
    replot5(handles)
    
end

% ------------------------------------------------------------------------- 

function replot6(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m6Axes); plotSpline(m6Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m12Nodes = m6Nodes;
        axes(handles.m12Axes);
        plotSpline(m12Nodes)
    end

% ------------------------------------------------------------------------- 

function varargout = m6PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m6Nodes = m6Nodes+0.05;
    m6Nodes(find(m6Nodes < Range(1))) = Range(1);
    m6Nodes(find(m6Nodes > Range(2))) = Range(2);
    replot6(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m6MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m6Nodes = m6Nodes-0.05;
    m6Nodes(find(m6Nodes < Range(1))) = Range(1);
    m6Nodes(find(m6Nodes > Range(2))) = Range(2);
    replot6(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m6Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m6Nodes m12Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m6 spline curve.
    m6Nodes = updateSpline(m6Nodes);
    replot6(handles)
    
end

% ------------------------------------------------------------------------- 

function replot7(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m7Axes); plotSpline(m7Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m1Nodes = m7Nodes; axes(handles.m1Axes); plotSpline(m1Nodes)
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m2Nodes = m7Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
            m3Nodes = m7Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
            m8Nodes = m7Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
            m9Nodes = m7Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        end
        
    else,
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m8Nodes = m7Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
            m9Nodes = m7Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        end        
        
    end

% ------------------------------------------------------------------------- 

function varargout = m7PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m7Nodes = m7Nodes+0.05;
    m7Nodes(find(m7Nodes < Range(1))) = Range(1);
    m7Nodes(find(m7Nodes > Range(2))) = Range(2);
    replot7(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m7MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m7Nodes = m7Nodes-0.05;
    m7Nodes(find(m7Nodes < Range(1))) = Range(1);
    m7Nodes(find(m7Nodes > Range(2))) = Range(2);
    replot7(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m7Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m1Nodes m2Nodes m3Nodes m7Nodes m8Nodes m9Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m7 spline curve.
    m7Nodes = updateSpline(m7Nodes);
    replot7(handles)
    
end

% ------------------------------------------------------------------------- 

function replot8(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m8Axes); plotSpline(m8Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m2Nodes = m8Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m1Nodes = m8Nodes; axes(handles.m1Axes); plotSpline(m1Nodes)
            m3Nodes = m8Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
            m7Nodes = m8Nodes; axes(handles.m7Axes); plotSpline(m7Nodes)
            m9Nodes = m8Nodes; axes(handles.m9Axes); plotSpline(m9Nodes)
        end
        
    else,
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m1Nodes = m8Nodes; axes(handles.m1Axes); plotSpline(m1Nodes)
            m3Nodes = m8Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
        end        
        
    end

% ------------------------------------------------------------------------- 

function varargout = m8PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m8Nodes = m8Nodes+0.05;
    m8Nodes(find(m8Nodes < Range(1))) = Range(1);
    m8Nodes(find(m8Nodes > Range(2))) = Range(2);
    replot8(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m8MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m8Nodes = m8Nodes-0.05;
    m8Nodes(find(m8Nodes < Range(1))) = Range(1);
    m8Nodes(find(m8Nodes > Range(2))) = Range(2);
    replot8(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m8Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m1Nodes m2Nodes m3Nodes m7Nodes m8Nodes m9Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m8 spline curve.
    m8Nodes = updateSpline(m8Nodes);
    replot8(handles)
    
end

% ------------------------------------------------------------------------- 

function replot9(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m9Axes); plotSpline(m9Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m3Nodes = m9Nodes; axes(handles.m3Axes); plotSpline(m3Nodes)
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m1Nodes = m9Nodes; axes(handles.m1Axes); plotSpline(m1Nodes)
            m2Nodes = m9Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
            m7Nodes = m9Nodes; axes(handles.m7Axes); plotSpline(m7Nodes)
            m8Nodes = m9Nodes; axes(handles.m8Axes); plotSpline(m8Nodes)
        end
        
    else,
        
        % Determine if quads box is checked
        if get(handles.quadsCheckbox, 'Value') == 1,
            m1Nodes = m9Nodes; axes(handles.m1Axes); plotSpline(m1Nodes)
            m2Nodes = m9Nodes; axes(handles.m2Axes); plotSpline(m2Nodes)
        end        
        
    end

% ------------------------------------------------------------------------- 

function varargout = m9PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m9Nodes = m9Nodes+0.05;
    m9Nodes(find(m9Nodes < Range(1))) = Range(1);
    m9Nodes(find(m9Nodes > Range(2))) = Range(2);
    replot9(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m9MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m9Nodes = m9Nodes-0.05;
    m9Nodes(find(m9Nodes < Range(1))) = Range(1);
    m9Nodes(find(m9Nodes > Range(2))) = Range(2);
    replot9(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m9Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m1Nodes m2Nodes m3Nodes m7Nodes m8Nodes m9Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m9 spline curve.
    m9Nodes = updateSpline(m9Nodes);
    replot9(handles)
   
end

% ------------------------------------------------------------------------- 

function replot10(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m10Axes); plotSpline(m10Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m4Nodes = m10Nodes;
        axes(handles.m4Axes);
        plotSpline(m4Nodes)
    end

% ------------------------------------------------------------------------- 

function varargout = m10PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m10Nodes = m10Nodes+0.05;
    m10Nodes(find(m10Nodes < Range(1))) = Range(1);
    m10Nodes(find(m10Nodes > Range(2))) = Range(2);
    replot10(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m10MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m10Nodes = m10Nodes-0.05;
    m10Nodes(find(m10Nodes < Range(1))) = Range(1);
    m10Nodes(find(m10Nodes > Range(2))) = Range(2);
    replot10(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m10Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m4Nodes m10Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m10 spline curve.
    m10Nodes = updateSpline(m10Nodes);
    replot10(handles)
    
end

% ------------------------------------------------------------------------- 

function replot11(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m11Axes); plotSpline(m11Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m5Nodes = m11Nodes;
        axes(handles.m5Axes);
        plotSpline(m5Nodes)
    end

% ------------------------------------------------------------------------- 

function varargout = m11PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m11Nodes = m11Nodes+0.05;
    m11Nodes(find(m11Nodes < Range(1))) = Range(1);
    m11Nodes(find(m11Nodes > Range(2))) = Range(2);
    replot11(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m11MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m11Nodes = m11Nodes-0.05;
    m11Nodes(find(m11Nodes < Range(1))) = Range(1);
    m11Nodes(find(m11Nodes > Range(2))) = Range(2);
    replot11(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m11Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m5Nodes m11Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m11 spline curve.
    m11Nodes = updateSpline(m11Nodes);
    replot11(handles)
    
end

% ------------------------------------------------------------------------- 

function replot12(handles)

global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

    axes(handles.m12Axes); plotSpline(m12Nodes)
    
    % Determine if symmetry box is checked
    if get(handles.symmetryCheckbox, 'Value') == 1,
        m6Nodes = m12Nodes;
        axes(handles.m6Axes);
        plotSpline(m6Nodes)
    end

% ------------------------------------------------------------------------- 

function varargout = m12PlusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1plus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m12Nodes = m12Nodes+0.05;
    m12Nodes(find(m12Nodes < Range(1))) = Range(1);
    m12Nodes(find(m12Nodes > Range(2))) = Range(2);
    replot12(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m12MinusPushbutton_Callback(h, eventdata, handles, varargin)

% Accept push on the m1minus button.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes
global Range

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,
    
    m12Nodes = m12Nodes-0.05;
    m12Nodes(find(m12Nodes < Range(1))) = Range(1);
    m12Nodes(find(m12Nodes > Range(2))) = Range(2);
    replot12(handles)
    
end

% ------------------------------------------------------------------------- 

function varargout = m12Axes_ButtonDownFcn(h, eventdata, handles, varargin)

% Given user changes to node points, update the spline curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m6Nodes m12Nodes

% Check if the inputs have been initialized
initFlag = checkIntialization;

% Proceed if inputs have been initialized
if initFlag == 1,

    % Update the m12 spline curve.
    m12Nodes = updateSpline(m12Nodes);
    replot12(handles)
    
end

% ------------------------------------------------------------------------- 

function plotSpline(Nodes)

% Plot the spline nodes and curve.
global t tt ForceZeroSlope Range exc_range

cla
Nodes(find(Nodes < Range(1))) = Range(1);
Nodes(find(Nodes > Range(2))) = Range(2);
y = Nodes;
if mean(y) ~= 0,
    if ForceZeroSlope
        cs = spline(t,[0; y; 0]);
    else
        cs = spline(t,y);
    end
    yy = ppval(cs,tt);

    plot(tt,yy,'b-',t,y,'or');
    hold on
    hstem = stem(tt, yy);
    plot(tt,(yy+exc_range),'r:',tt,(yy-exc_range),'r:');
    set(hstem, 'Color', 'b', 'Marker', 'none', 'HitTest', 'off');
    set(gca, 'ylim', [Range(1) Range(2)])
end

% ------------------------------------------------------------------------- 

function Nodes = updateSpline(Nodes)

% Given user mouse input, update the spline nodes and curve.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global tmax NumControl FPS ForceZeroSlope Range

% Clear the current axes.
cla

% Define temporary node vector.
Nodes(find(Nodes < Range(1))) = Range(1);
Nodes(find(Nodes > Range(2))) = Range(2);
lineY = Nodes;

% Define time vectors for number of nodes and number of time frames.
t = linspace(0, tmax, NumControl)' ;
tt = linspace(0, tmax, (tmax+1)*FPS)' ;

% Check if no nodes were input and use all zeros in this case.
if size(lineY) == [0 0],
	%start with all zeros in control points
	y = zeros(length(t), 1);
% If nodes were input, use them in this case.
else,
    y = lineY;
end

% Check if the spline should be forced to have zero slope at the ends.
if ForceZeroSlope
    cs = spline(t,[0 y 0]);
% Otherwise, do not force zero slope at the ends.
else
    cs = spline(t,y);
end

% Evaluate piecewise polynomial. 
yy = ppval(cs,tt);

% Hold the current plot and all axis properties.
hold on

% Plot the spline curve.
sp = plot(tt,yy);

% Plot the spline nodes.
line1 = plot(t,y,'or');

% Set the y-axis limits to hard coded values.
set(gca, 'ylim', [Range(1) Range(2)])

% Get the current mouse position.
mouse = get(gca,'currentpoint');

% Calculate the point nearest to the mouse click.
lineX = get(line1,'xdata')';
lineY = get(line1,'ydata')';
[val, pnt] = min(abs(lineX-mouse(1,1)));

% Move the control point on the plot.
lineY(pnt) = mouse(1,2);
lineY(find(lineY < Range(1))) = Range(1);
lineY(find(lineY > Range(2))) = Range(2);
set(line1, 'ydata', lineY);

% Make new spline curve.
if ForceZeroSlope
    cs = spline(t,[0 lineY 0]);
else
    cs = spline(t,lineY);
end
yy = ppval(cs,tt);
set(sp, 'ydata', yy);

% Plot the bars
% hbar = bar(tt(1:2:end), yy(1:2:end), 0.6, 'b');
% set(hbar, 'EdgeColor', 'none', 'HitTest', 'off');
hstem = stem(tt, yy);
set(hstem, 'Color', 'b', 'Marker', 'none', 'HitTest', 'off');

% Flush pending graphics events.
drawnow

% Pass back the new nodes.
Nodes = lineY;

% ------------------------------------------------------------------------- 

function initFlag = checkIntialization

% Determine if input torques are defined before allowing control panel
% changes.

% Define global variables for the functions: initializePushbutton_Callback,
% runPushbutton_Callback, animatePushbutton_Callback,
% savePushbutton_Callback, m1Axes_ButtonDownFcn, m2Axes_ButtonDownFcn,
% m3Axes_ButtonDownFcn, m4Axes_ButtonDownFcn, m5Axes_ButtonDownFcn, 
% m6Axes_ButtonDownFcn, checkIntialization, and updateSpline.
global m1Nodes m2Nodes m3Nodes m4Nodes m5Nodes m6Nodes
global m7Nodes m8Nodes m9Nodes m10Nodes m11Nodes m12Nodes

% Check if m1Nodes, m2Nodes, m3Nodes, m4Nodes, m5Nodes, and m6Nodes are not empty.
% If they are empty, then present the user with an error dialog box (see
% below).
if ~isempty(m1Nodes) & ~isempty(m2Nodes) & ~isempty(m3Nodes) & ~isempty(m4Nodes) & ~isempty(m5Nodes) & ~isempty(m6Nodes) & ~isempty(m7Nodes) & ~isempty(m8Nodes) & ~isempty(m9Nodes) & ~isempty(m10Nodes) & ~isempty(m11Nodes) & ~isempty(m12Nodes),
    
    initFlag = 1;
    
% Open error dialog box if m1Nodes, m2Nodes, m3Nodes, m4Nodes, m5Nodes, and m6Nodes
% are empty. 
else
    
    initFlag = 0;
    beep
    errordlg({'The input excitations do not exist!'
              ''
              '   1. Generate "Excitations" using the "Initialize" button.'
              '   2. Change "Excitations" using the mouse.'},...
              'Initialization Error', 'modal')
          
end % End of if m1Nodes, m2Nodes, m3Nodes, m4Nodes, m5Nodes, and m6Nodes  are not empty.


% ------------------------------------------------------------------------- 
% ------------------------------------------------------------------------- 

% Lines below contain GUI creation specifics.

% ------------------------------------------------------------------------- 
% ------------------------------------------------------------------------- 


% --- Creates and returns a handle to the GUI figure. 
function hfig = openfig(filename, policy, varargin)
hfig = ExcitationGUI_LayoutFcn(policy);

function hfig = ExcitationGUI_LayoutFcn(policy)
% policy - create a new figure or use a singleton. 'new' or 'reuse'.

persistent hsingleton;
if strcmpi(policy, 'reuse') & ishandle(hsingleton)
    hfig = hsingleton;
    return;
end

appdata = [];
appdata.GUIDEOptions = struct(...
    'active_h', [], ...
    'taginfo', struct(...
    'figure', 2, ...
    'text', 10, ...
    'axes', 14, ...
    'radiobutton', 3, ...
    'edit', 3, ...
    'pushbutton', 3), ...
    'override', 0, ...
    'resize', 'simple', ...
    'accessibility', 'callback', ...
    'mfile', 1, ...
    'callbacks', 1, ...
    'singleton', 1, ...
    'blocking', 0, ...
    'syscolorfig', 1, ...
    'lastSavedFile', 'C:\MATLAB6p1\work\ExcitationGUI.m', ...
    'release', 12);
appdata.lastValidTag = 'ExcitationGUI';
appdata.GUIDELayoutEditor = [];

set(0,'Units','pixels')
screenSize = get(0, 'ScreenSize');
leftMargin = (screenSize(3)-800)/2;
bottomMargin = (screenSize(4)-600)/2;

hfig = figure(...
'Color',get(0,'defaultUicontrolBackgroundColor'),...
'Colormap',[0 0 0.5625;0 0 0.625;0 0 0.6875;0 0 0.75;0 0 0.8125;0 0 0.875;0 0 0.9375;0 0 1;0 0.0625 1;0 0.125 1;0 0.1875 1;0 0.25 1;0 0.3125 1;0 0.375 1;0 0.4375 1;0 0.5 1;0 0.5625 1;0 0.625 1;0 0.6875 1;0 0.75 1;0 0.8125 1;0 0.875 1;0 0.9375 1;0 1 1;0.0625 1 1;0.125 1 0.9375;0.1875 1 0.875;0.25 1 0.8125;0.3125 1 0.75;0.375 1 0.6875;0.4375 1 0.625;0.5 1 0.5625;0.5625 1 0.5;0.625 1 0.4375;0.6875 1 0.375;0.75 1 0.3125;0.8125 1 0.25;0.875 1 0.1875;0.9375 1 0.125;1 1 0.0625;1 1 0;1 0.9375 0;1 0.875 0;1 0.8125 0;1 0.75 0;1 0.6875 0;1 0.625 0;1 0.5625 0;1 0.5 0;1 0.4375 0;1 0.375 0;1 0.3125 0;1 0.25 0;1 0.1875 0;1 0.125 0;1 0.0625 0;1 0 0;0.9375 0 0;0.875 0 0;0.8125 0 0;0.75 0 0;0.6875 0 0;0.625 0 0;0.5625 0 0],...
'IntegerHandle','off',...
'InvertHardcopy','off',...
...% 'MenuBar','none',...
'Name','CMC Excitation Range Editor',...
'NumberTitle','off',...
'PaperOrientation','landscape',...
'PaperPosition',[1.5 1.25 8 6],...
'PaperUnits','inches',...
'Position',[leftMargin bottomMargin 800 600],...
'Renderer','painters',...
'RendererMode','manual',...
'HandleVisibility','callback',...
'Tag','ExcitationGUI',...
'Units','characters',...
'UserData',[],...
'CreateFcn', {@local_CreateFcn, '', appdata} );

% ------------------------------------------------------------------------- 
% Control panel block.

buttonsX = 0.04875;
buttonsY = 0.895;
buttonsSpace = 0.120;

appdata = [];
appdata.lastValidTag = 'controlText';

hcontrol = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[0.02375 (buttonsY+0.06) 0.475 0.0233333333333333],...
'String','CONTROL PANEL',...
'Style','text',...
'Tag','controlText',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'initializePushbutton';

hinitialize = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''initializePushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[buttonsX buttonsY 0.075 0.05],...
'String','Initialize',...
'Tag','initializePushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );


appdata = [];
appdata.lastValidTag = 'symmetryCheckbox';

hsymmetry = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''symmetryCheckbox_Callback'',gcbo,[],guidata(gcbo))',...
'ListboxTop',0,...
'Position',[buttonsX+buttonsSpace buttonsY+0.025 0.09875 0.0333333333333333],...
'String','Symmetry',...
'Style','checkbox',...
'Value',0,...
'Tag','symmetryCheckbox',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'quadricepsCheckbox';

hquads = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''quadsCheckbox_Callback'',gcbo,[],guidata(gcbo))',...
'ListboxTop',0,...
'Position',[buttonsX+buttonsSpace buttonsY-0.005 0.09875 0.0333333333333333],...
'String','Quads',...
'Style','checkbox',...
'Value',0,...
'Tag','quadsCheckbox',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'savePushbutton';

hsave = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''savePushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[buttonsX+2*buttonsSpace buttonsY 0.075 0.05],...
'String','Save',...
'Tag','savePushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'quitPushbutton';

hquit = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''quitPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[buttonsX+3*buttonsSpace buttonsY 0.075 0.05],...
'String','Quit',...
'Tag','quitPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

% ------------------------------------------------------------------------- 
% Lab logo block.

topRightX = 0.92;
topRightY = 0.99;

sizeX = 0.31;
sizeY = 0.12;

logo = imageOfLabLogo;
    
appdata = [];
appdata.lastValidTag = 'logoPushbutton';

hlogo = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''logoPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','normal',...
'ListboxTop',0,...
'Position',[(topRightX-sizeX) (topRightY-sizeY) sizeX sizeY],...
'String','',...
'Style','pushbutton',...
'Tag','logoPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'CData', logo);

% ------------------------------------------------------------------------- 
% Inputs block.

plotsWidth = 0.4;
plotsXa = 0.067;
plotsXb = 0.555;
plotsY = 0.73;
plotsSpace = 0.13333333333333;

changesSpace = 0.108333333333333-0.025;

appdata = [];
appdata.lastValidTag = 'inputsText';

hinputs1 = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[(plotsXa-0.04375) (plotsY+0.11666666666667) 0.475 0.0233333333333333],...
'String','LEFT EXCITATIONS',...
'Style','text',...
'Tag','inputsText',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm1Axes';

haxes1 = axes(...
'Parent',hfig,...
'Position',[plotsXa plotsY plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m1Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m1Axes',...
'UserData',[]);

htitle1 = get(haxes1,'title');

set(htitle1,...
'Parent',haxes1,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel1 = get(haxes1,'xlabel');

set(hxlabel1,...
'Parent',haxes1,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 -0.161538461538462 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

hylabel1 = get(haxes1,'ylabel');

set(hylabel1,...
'Parent',haxes1,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615384 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'RF',...
'FontSize', 9);

hzlabel1 = get(haxes1,'zlabel');

set(hzlabel1,...
'Parent',haxes1,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 4.77692307692308 1.00005459937205],...
'HandleVisibility','off',...
'Visible','off');

appdata = [];
appdata.lastValidTag = 'm1PlusPushbutton';

hm1plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m1PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m1PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm1MinusPushbutton';

hm1minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m1MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY 0.02 0.025],...
'String','-',...
'Tag','m1MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm2Axes';

haxes2 = axes(...
'Parent',hfig,...
'Position',[plotsXa (plotsY-plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m2Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m2Axes',...
'UserData',[]);

htitle2 = get(haxes2,'title');

set(htitle2,...
'Parent',haxes2,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461539 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel2 = get(haxes2,'xlabel');

set(hxlabel2,...
'Parent',haxes2,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 -0.16153846153846 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

hylabel2 = get(haxes2,'ylabel');

set(hylabel2,...
'Parent',haxes2,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615386 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'VAS LAT',...
'FontSize', 9);

hzlabel2 = get(haxes2,'zlabel');

set(hzlabel2,...
'Parent',haxes2,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 6.00769230769231 1.00005459937205],...
'HandleVisibility','off',...
'Visible','off');

appdata = [];
appdata.lastValidTag = 'm2PlusPushbutton';

hm2plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m2PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m2PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm2MinusPushbutton';

hm2minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m2MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m2MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm3Axes';

haxes3 = axes(...
'Parent',hfig,...
'Position',[plotsXa (plotsY-2*plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m3Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m3Axes',...
'UserData',[]);

htitle3 = get(haxes3,'title');

set(htitle3,...
'Parent',haxes3,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel3 = get(haxes3,'xlabel');

set(hxlabel3,...
'Parent',haxes3,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 -0.161538461538463 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

hylabel3 = get(haxes3,'ylabel');

set(hylabel3,...
'Parent',haxes3,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615383 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'VAS MED',...
'FontSize', 9);

hzlabel3 = get(haxes3,'zlabel');

set(hzlabel3,...
'Parent',haxes3,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 7.23846153846154 1.00005459937205],...
'HandleVisibility','off',...
'Visible','off');

appdata = [];
appdata.lastValidTag = 'm3PlusPushbutton';

hm3plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m3PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-2*plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m3PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm3MinusPushbutton';

hm3minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m3MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-2*plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m3MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm4Axes';

haxes4 = axes(...
'Parent',hfig,...
'Position',[plotsXa (plotsY-3*plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m4Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m4Axes',...
'UserData',[]);

htitle4 = get(haxes4,'title');

set(htitle4,...
'Parent',haxes4,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel4 = get(haxes4,'xlabel');

set(hxlabel4,...
'Parent',haxes4,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[49.8529411764706 -0.330769230769231 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off',...
'Visible','off');

hylabel4 = get(haxes4,'ylabel');

set(hylabel4,...
'Parent',haxes4,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615385 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'HAM',...
'FontSize', 9);

hzlabel4 = get(haxes4,'zlabel');

set(hzlabel4,...
'Parent',haxes4,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 8.46923076923077 1.00005459937205],...
'HandleVisibility','off',...
'Visible','off');

appdata = [];
appdata.lastValidTag = 'm4PlusPushbutton';

hm4plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m4PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-3*plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m4PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm4MinusPushbutton';

hm4minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m4MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-3*plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m4MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm5Axes';

haxes5 = axes(...
'Parent',hfig,...
'Position',[plotsXa (plotsY-4*plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m5Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m5Axes',...
'UserData',[]);

htitle5 = get(haxes5,'title');

set(htitle5,...
'Parent',haxes5,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel5 = get(haxes5,'xlabel');

set(hxlabel5,...
'Parent',haxes5,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... % 'Position',[49.8529411764706 -0.161538461538463 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

hylabel5 = get(haxes5,'ylabel');

set(hylabel5,...
'Parent',haxes5,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615383 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'TIB ANT',...
'FontSize', 9);

hzlabel5 = get(haxes5,'zlabel');

set(hzlabel5,...
'Parent',haxes5,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 7.23846153846154 1.00005459937205],...
'HandleVisibility','off');

appdata = [];
appdata.lastValidTag = 'm5PlusPushbutton';

hm5plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m5PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-4*plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m5PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm5MinusPushbutton';

hm5minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m5MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-4*plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m5MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm6Axes';

haxes6 = axes(...
'Parent',hfig,...
'Position',[plotsXa (plotsY-5*plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m6Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m6Axes',...
'UserData',[]);

htitle6 = get(haxes6,'title');

set(htitle6,...
'Parent',haxes6,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel6 = get(haxes6,'xlabel');

set(hxlabel6,...
'Parent',haxes6,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... % 'Position',[49.8529411764706 -0.161538461538463 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off',...
'String', '% Gait Cycle',...
'FontSize', 9);

hylabel6 = get(haxes6,'ylabel');

set(hylabel6,...
'Parent',haxes6,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615383 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'GAS / SOL',...
'FontSize', 9);

hzlabel6 = get(haxes6,'zlabel');

set(hzlabel6,...
'Parent',haxes6,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 7.23846153846154 1.00005459937205],...
'HandleVisibility','off');

appdata = [];
appdata.lastValidTag = 'inputsText';

hinputs2 = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[(plotsXb-0.04375) (plotsY+0.11666666666667) 0.475 0.0233333333333333],...
'String','RIGHT EXCITATIONS',...
'Style','text',...
'Tag','inputsText',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm6PlusPushbutton';

hm6plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m6PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-5*plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m6PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm6MinusPushbutton';

hm6minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m6MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXa+plotsWidth plotsY-5*plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m6MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm7Axes';

haxes7 = axes(...
'Parent',hfig,...
'Position',[plotsXb plotsY plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m7Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m7Axes',...
'UserData',[]);

htitle7 = get(haxes7,'title');

set(htitle7,...
'Parent',haxes7,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel7 = get(haxes7,'xlabel');

set(hxlabel7,...
'Parent',haxes7,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... % 'Position',[49.8529411764706 -0.161538461538463 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

hylabel7 = get(haxes7,'ylabel');

set(hylabel7,...
'Parent',haxes7,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615383 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'RF',...
'FontSize', 9);

hzlabel7 = get(haxes7,'zlabel');

set(hzlabel7,...
'Parent',haxes7,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 7.23846153846154 1.00005459937205],...
'HandleVisibility','off');

appdata = [];
appdata.lastValidTag = 'm7PlusPushbutton';

hm7plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m7PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m7PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm7MinusPushbutton';

hm7minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m7MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY 0.02 0.025],...
'String','-',...
'Tag','m7MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm8Axes';

haxes8 = axes(...
'Parent',hfig,...
'Position',[plotsXb (plotsY-plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m8Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m8Axes',...
'UserData',[]);

htitle8 = get(haxes8,'title');

set(htitle8,...
'Parent',haxes8,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel8 = get(haxes8,'xlabel');

set(hxlabel8,...
'Parent',haxes8,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... % 'Position',[49.8529411764706 -0.161538461538463 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

hylabel8 = get(haxes8,'ylabel');

set(hylabel8,...
'Parent',haxes8,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615383 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'VAS LAT',...
'FontSize', 9);

hzlabel8 = get(haxes8,'zlabel');

set(hzlabel8,...
'Parent',haxes8,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 7.23846153846154 1.00005459937205],...
'HandleVisibility','off');

appdata = [];
appdata.lastValidTag = 'm8PlusPushbutton';

hm8plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m8PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m8PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm8MinusPushbutton';

hm8minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m8MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m8MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm9Axes';

haxes9 = axes(...
'Parent',hfig,...
'Position',[plotsXb (plotsY-2*plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m9Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m9Axes',...
'UserData',[]);

htitle9 = get(haxes9,'title');

set(htitle9,...
'Parent',haxes9,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel9 = get(haxes9,'xlabel');

set(hxlabel9,...
'Parent',haxes9,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... % 'Position',[49.8529411764706 -0.161538461538463 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

hylabel9 = get(haxes9,'ylabel');

set(hylabel9,...
'Parent',haxes9,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615383 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'VAS MED',...
'FontSize', 9);

hzlabel9 = get(haxes9,'zlabel');

set(hzlabel9,...
'Parent',haxes9,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 7.23846153846154 1.00005459937205],...
'HandleVisibility','off');

appdata = [];
appdata.lastValidTag = 'm9PlusPushbutton';

hm9plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m9PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-2*plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m9PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm9MinusPushbutton';

hm9minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m9MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-2*plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m9MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm10Axes';

haxes10 = axes(...
'Parent',hfig,...
'Position',[plotsXb (plotsY-3*plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m10Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m10Axes',...
'UserData',[]);

htitle10 = get(haxes10,'title');

set(htitle10,...
'Parent',haxes10,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel10 = get(haxes10,'xlabel');

set(hxlabel10,...
'Parent',haxes10,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... % 'Position',[49.8529411764706 -0.161538461538463 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

hylabel10 = get(haxes10,'ylabel');

set(hylabel10,...
'Parent',haxes10,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615383 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'HAM',...
'FontSize', 9);

hzlabel10 = get(haxes10,'zlabel');

set(hzlabel10,...
'Parent',haxes10,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 7.23846153846154 1.00005459937205],...
'HandleVisibility','off');

appdata = [];
appdata.lastValidTag = 'm10PlusPushbutton';

hm10plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m10PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-3*plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m10PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm10MinusPushbutton';

hm10minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m10MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-3*plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m10MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm11Axes';

haxes11 = axes(...
'Parent',hfig,...
'Position',[plotsXb (plotsY-4*plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m11Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'XTickLabel','',...
'XTickLabelMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m11Axes',...
'UserData',[]);

htitle11 = get(haxes11,'title');

set(htitle11,...
'Parent',haxes11,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel11 = get(haxes11,'xlabel');

set(hxlabel11,...
'Parent',haxes11,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... % 'Position',[49.8529411764706 -0.161538461538463 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

hylabel11 = get(haxes11,'ylabel');

set(hylabel11,...
'Parent',haxes11,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615383 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'TIB ANT',...
'FontSize', 9);

hzlabel11 = get(haxes11,'zlabel');

set(hzlabel11,...
'Parent',haxes11,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 7.23846153846154 1.00005459937205],...
'HandleVisibility','off');

appdata = [];
appdata.lastValidTag = 'm11PlusPushbutton';

hm11plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m11PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-4*plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m11PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm11MinusPushbutton';

hm11minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m11MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-4*plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m11MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm12Axes';

haxes12 = axes(...
'Parent',hfig,...
'Position',[plotsXb (plotsY-5*plotsSpace) plotsWidth 0.108333333333333],...
'Box','on',...
'ButtonDownFcn','ExcitationGUI(''m12Axes_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'CameraPosition',[50 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'FontSize',8,...
'NextPlot','replacechildren',...
'XColor',get(0,'defaultaxesXColor'),...
'XLim',[0 100],...
'XLimMode','manual',...
'YColor',get(0,'defaultaxesYColor'),...
'YLim',get(0,'defaultaxesYLim'),...
'YLimMode','manual',...
'ZColor',get(0,'defaultaxesZColor'),...
'CreateFcn', {@local_CreateFcn, '', appdata},...
'Tag','m12Axes',...
'UserData',[]);

htitle12 = get(haxes12,'title');

set(htitle12,...
'Parent',haxes12,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[49.8529411764706 1.08461538461538 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

hxlabel12 = get(haxes12,'xlabel');

set(hxlabel12,...
'Parent',haxes12,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... % 'Position',[49.8529411764706 -0.161538461538463 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off',...
'String', '% Gait Cycle',...
'FontSize', 9);

hylabel12 = get(haxes12,'ylabel');

set(hylabel12,...
'Parent',haxes12,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
... 'Position',[-7.5 0.484615384615383 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','baseline',...
'HandleVisibility','off',...
'String', 'GAS / SOL',...
'FontSize', 9);

hzlabel12 = get(haxes12,'zlabel');

set(hzlabel12,...
'Parent',haxes12,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-14.5588235294118 7.23846153846154 1.00005459937205],...
'HandleVisibility','off');

appdata = [];
appdata.lastValidTag = 'm12PlusPushbutton';

hm12plus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m12PlusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-5*plotsSpace+changesSpace 0.02 0.025],...
'String','+',...
'Tag','m12PlusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );

appdata = [];
appdata.lastValidTag = 'm12MinusPushbutton';

hm12minus = uicontrol(...
'Parent',hfig,...
'Units','normalized',...
'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'),...
'Callback','ExcitationGUI(''m12MinusPushbutton_Callback'',gcbo,[],guidata(gcbo))',...
'FontWeight','bold',...
'ListboxTop',0,...
'Position',[plotsXb+plotsWidth plotsY-5*plotsSpace 0.02 0.025],...
'String','-',...
'Tag','m12MinusPushbutton',...
'CreateFcn', {@local_CreateFcn, '', appdata} );


% ------------------------------------------------------------------------- 

hsingleton = hfig;


% ------------------------------------------------------------------------- 
% --- Set application data first then calling the CreateFcn. 
function local_CreateFcn(hObject, eventdata, createfcn, appdata)

if ~isempty(appdata)
   names = fieldnames(appdata);
   for i=1:length(names)
       name = char(names(i));
       setappdata(hObject, name, getfield(appdata,name));
   end
end

if ~isempty(createfcn)
   eval(createfcn);
end


% ------------------------------------------------------------------------- 
function logo = imageOfLabLogo

logoRed = [223	224	225	224	226	233	232	227	222	222	224	224	224	224	224	224	224	224	224	224	223	223	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	226	221	227	226	231	237	231	226	225	224	224	224	224	224	224	224	224	225	225	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	224	223	227	228	224	233	226	224	223	224	224	224	224	224	224	224	225	224	224	224	224	225	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	225	225	222	230	213	227	231	224	224	224	224	224	224	224	224	224	224	225	224	224	224	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
222	224	225	223	232	214	236	234	225	224	224	224	224	224	224	224	224	224	224	225	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	223	224	225	231	220	226	224	223	223	224	224	224	224	224	224	225	225	222	223	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
225	225	224	225	210	233	225	224	224	224	224	224	224	224	224	224	224	223	223	224	223	223	222	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	221	225	225	210	220	227	226	224	224	225	224	224	224	224	224	223	223	228	227	231	239	229	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	224	224	225	213	210	222	224	224	224	223	224	224	225	223	224	224	225	232	242	255	255	249	228	224	224	223	223	224	225	223	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	226	213	207	225	227	224	226	224	226	224	224	224	225	229	223	221	252	241	200	241	228	225	225	224	224	223	223	227	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	225	222	224	214	216	213	219	234	227	231	228	225	221	223	217	220	217	191	202	226	230	219	200	227	226	224	224	223	224	219	227	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	224	225	221	221	228	226	226	225	229	225	224	225	221	228	218	163	185	207	243	219	218	228	227	224	224	226	224	223	226	227	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	223	229	214	221	229	225	226	227	225	223	222	224	224	225	228	227	181	189	242	232	220	225	227	226	227	224	224	223	226	222	225	224	224	224	224	223	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	227	217	208	213	212	220	231	233	227	223	224	224	228	233	213	184	192	188	226	222	231	231	229	232	222	227	224	229	228	226	224	224	224	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	224	223	217	221	213	201	202	222	234	225	228	229	227	228	193	178	219	191	220	224	229	230	229	225	225	223	224	229	230	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	227	223	224	225	226	224	229	227	222	222	207	197	212	241	213	223	207	210	222	198	226	230	224	224	227	223	226	226	224	226	224	224	224	224	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	228	229	228	229	228	228	227	229	228	230	227	177	155	194	200	182	203	239	232	235	239	236	230	232	226	227	229	229	226	230	226	226	225	224	223	224	224	224	224	224	224	225	223	225	224	224	224	224	225	224	223	225	225	224	224	223	223	225	224	224	225	223	224	223	224	225	225	224	224	224	225	224	224	223	224	224	223	224	223	224	224	223	223	224	224	224	222	225	225	224	225	224	224	224	224	224	224	222	224	224	224	224	225	224	225	225	224	223	224	223	224	224	224	223	222	224	224	224	225	224	224	224	223	221	224	225	224	223	224	223	225	223	224	224	223	224	225	225	225	225	223	225	224	224	224	224	224	224	224	224	224	224	224	226	224	224	223	225	225	223	223	225	225	224	225	224	224	225	224	224	223	223	223	225	223	225	226	225	225	225	223	224	224	225	223	224	223	224	223	223	223	222	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	225	225	223	224	224	224	223	224	223	225	223	224	225	225	224	223	224	223	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	223	224	223	225	224	222	224	222	222	227	213	150	172	196	160	197	236	233	235	228	238	216	229	236	226	224	224	224	223	225	225	225	224	223	224	224	224	224	224	225	224	224	225	221	225	224	222	224	223	226	223	223	222	225	224	223	222	225	225	223	224	223	224	225	224	224	223	224	225	223	223	224	224	223	223	224	223	225	226	223	223	223	223	224	225	225	223	224	224	223	224	224	225	223	227	224	224	224	224	223	224	223	225	223	225	223	224	224	225	224	225	225	225	225	223	224	224	224	223	224	224	224	225	223	223	224	224	223	225	223	223	224	223	227	225	220	222	225	224	222	225	225	225	224	224	224	224	223	226	224	224	223	224	224	223	224	224	223	224	226	224	222	223	222	224	223	223	222	225	224	227	223	225	223	223	222	224	224	223	224	223	225	224	224	223	224	225	224	223	225	225	224	224	224	223	223	224	224	224	224	224	224	224	224	224	224	226	223	224	224	224	223	224	224	224	224	225	221	224	225	222	221	223	225	226	226	223	225	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	224	224	226	225	224	223	224	225	224	225	223	226	189	191	183	184	191	225	228	228	236	213	227	229	236	231	223	224	226	224	225	224	224	223	224	224	224	224	224	224	224	224	226	223	224	223	224	225	221	227	225	225	224	225	224	222	225	224	223	225	223	224	223	224	223	225	227	223	224	224	225	227	224	224	223	223	224	223	223	224	224	224	223	224	223	222	223	225	224	224	224	225	224	222	224	223	222	223	226	224	223	223	224	223	223	225	225	226	223	223	225	222	224	222	224	226	225	222	224	223	225	225	223	226	224	225	223	224	224	226	225	224	226	224	223	224	223	223	223	223	226	224	224	224	223	223	224	225	223	224	224	224	222	225	224	224	225	221	223	226	222	224	224	223	224	224	224	224	223	224	222	226	224	224	225	227	225	224	223	224	223	225	224	224	223	224	225	223	223	223	224	224	224	225	225	223	224	224	225	224	224	224	224	224	224	224	224	223	222	225	224	223	223	224	220	225	223	224	225	224	223	225	225	224	221	223	224	225	227	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	223	226	224	225	224	227	225	224	224	222	229	230	209	191	171	190	205	220	238	235	238	192	205	223	223	220	239	227	224	230	224	223	224	224	224	224	224	224	224	225	225	224	224	190	156	159	175	188	222	223	224	227	223	224	223	224	189	163	184	182	162	190	224	223	224	224	223	224	224	222	222	223	224	224	223	225	225	223	223	223	224	224	224	223	225	225	225	223	224	226	224	223	222	224	223	224	224	224	225	224	224	223	224	226	224	224	223	224	223	225	222	223	224	225	224	224	223	224	224	191	165	184	180	195	225	223	224	224	224	220	224	224	223	216	174	186	172	176	224	225	222	225	224	225	224	224	225	225	224	224	225	225	224	227	224	224	223	224	227	222	225	225	224	223	225	223	224	225	225	225	224	223	224	224	223	224	223	223	224	222	224	223	224	224	224	223	223	224	225	223	224	224	224	222	223	225	224	224	225	224	224	224	224	224	224	224	224	225	223	224	224	225	224	224	224	223	224	225	223	223	224	224	224	224	224	227	224	222	225	223	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	225	213	198	216	227	223	209	226	226	225	235	231	227	206	144	184	207	214	229	239	243	218	190	196	188	199	222	236	233	226	224	224	224	224	224	224	224	224	224	222	226	224	222	160	99	5	0	19	133	222	221	224	223	222	223	224	153	93	26	28	99	150	224	224	223	223	224	224	224	225	223	224	224	225	226	223	225	224	223	224	224	224	224	225	225	224	223	225	225	223	223	225	224	222	226	225	223	223	225	223	224	225	225	225	223	224	224	224	224	227	225	225	224	224	224	224	224	223	224	151	89	30	8	38	222	224	224	223	224	224	222	226	227	136	0	28	99	134	220	224	223	223	224	224	225	224	224	224	224	223	223	225	223	222	223	223	225	224	223	223	223	223	223	223	224	224	224	224	224	224	224	224	224	223	224	224	225	224	224	225	224	225	224	223	225	224	224	225	222	224	224	224	226	224	225	224	225	224	224	224	224	224	224	224	224	224	224	224	222	225	223	222	225	224	223	223	225	224	224	227	224	223	225	224	224	225	224	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	225	215	197	217	227	224	212	226	224	226	231	231	225	208	166	183	182	180	230	243	218	218	191	187	167	188	215	227	235	225	223	224	224	225	224	224	224	224	224	225	223	224	224	222	224	205	32	0	1	149	225	223	225	224	225	223	224	224	99	105	223	224	224	225	223	224	224	225	224	227	224	225	224	224	223	225	223	224	224	225	224	224	224	223	224	222	225	222	223	224	224	224	223	225	224	224	225	224	223	225	222	223	224	225	224	224	222	224	226	224	224	225	227	225	225	224	223	223	224	224	225	180	0	1	184	223	225	225	222	224	224	224	221	18	2	40	224	225	225	223	223	225	223	224	223	225	224	223	224	224	224	224	224	223	225	225	225	224	223	225	223	225	224	224	223	226	225	225	223	224	225	224	224	225	224	224	224	225	225	224	224	224	224	224	223	225	224	224	225	225	225	223	223	225	223	223	224	225	225	224	224	224	224	224	224	224	224	226	223	221	224	226	223	224	225	227	224	223	223	222	225	224	224	224	224	223	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
225	225	212	197	217	228	224	213	226	223	224	232	231	226	221	158	179	225	216	214	208	184	203	204	180	212	232	211	217	234	229	224	224	224	224	224	224	224	224	224	222	225	224	228	225	224	224	112	7	0	0	185	225	225	222	226	225	223	223	124	143	223	224	225	213	202	219	223	211	199	197	198	203	220	224	224	209	215	222	221	202	220	225	223	224	220	216	221	214	211	223	213	203	222	223	209	200	216	224	224	223	223	224	223	222	228	224	225	222	209	158	135	181	215	225	223	224	224	223	224	224	224	210	1	1	67	223	225	224	224	224	224	225	175	9	3	56	223	223	223	224	206	219	222	218	202	219	224	225	223	215	221	222	206	216	225	224	223	215	169	156	207	221	223	225	224	223	221	224	221	214	169	135	178	206	224	226	223	220	205	220	222	214	203	224	224	224	224	216	221	220	206	221	224	208	213	223	218	203	217	224	224	224	224	224	224	224	224	223	225	223	204	218	223	222	223	224	225	225	221	197	215	226	219	203	212	220	222	223	224	226	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	226	221	211	218	227	225	211	227	225	224	236	231	225	222	187	196	205	193	197	215	188	171	167	194	231	226	225	220	207	233	226	224	224	223	224	224	224	224	224	225	223	224	224	223	222	224	91	167	122	2	11	205	225	223	225	224	224	224	124	142	225	224	224	200	126	10	18	134	174	167	160	30	103	223	225	173	93	20	38	159	202	223	224	225	209	128	21	134	198	223	180	150	24	40	174	154	53	82	218	224	223	227	223	224	222	224	186	79	134	178	182	133	56	78	207	224	223	227	222	224	224	199	19	22	2	211	223	223	224	223	224	222	58	179	21	52	222	224	223	222	168	59	20	67	161	218	223	224	227	196	99	34	165	209	224	223	118	105	143	123	1	169	227	223	227	221	225	183	58	90	153	140	137	59	48	197	225	218	149	43	20	97	171	221	226	223	223	185	83	46	159	213	223	177	77	24	46	143	218	224	224	224	224	224	224	224	224	224	223	222	11	97	223	224	223	224	223	223	221	169	85	21	112	177	106	33	162	227	223	225	222	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	224	224	225	224	225	224	224	223	231	230	225	228	206	177	187	168	200	203	205	191	189	186	234	228	229	229	209	204	234	227	223	223	224	224	225	224	224	224	224	222	225	224	224	225	92	165	221	74	0	47	221	225	224	223	223	224	120	144	225	224	225	224	224	76	30	223	223	224	224	173	67	223	224	225	212	1	117	223	225	224	223	224	224	219	38	220	225	224	224	224	23	62	225	224	213	11	70	222	224	224	227	223	224	172	47	211	222	223	225	225	212	107	16	197	224	224	224	224	224	193	76	160	0	116	225	224	223	224	224	191	79	216	3	50	222	224	224	224	225	162	0	200	224	225	224	223	224	224	181	57	223	223	223	147	38	222	224	224	152	119	220	227	223	224	131	1	161	223	225	223	223	220	114	74	223	224	224	144	0	197	223	224	224	223	224	224	179	113	224	223	224	224	200	0	148	224	223	224	224	224	224	224	224	224	224	224	224	192	2	15	214	223	224	223	224	223	224	223	189	0	192	222	224	158	0	178	225	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	224	224	224	224	224	228	224	224	225	223	224	224	232	227	196	177	229	213	208	207	200	194	229	225	223	222	229	207	219	228	229	228	223	225	223	223	224	224	225	224	224	222	224	224	90	153	225	216	36	0	85	223	225	224	224	224	122	142	224	223	225	223	225	94	44	223	224	224	224	218	193	225	225	224	211	2	112	224	225	222	225	224	223	220	49	219	223	225	224	220	37	66	224	225	223	143	0	203	225	223	223	223	203	0	205	223	224	224	224	225	222	226	51	9	210	223	224	224	225	175	64	221	13	14	217	225	224	224	223	99	183	217	6	44	223	224	223	223	224	165	2	206	224	223	224	224	224	223	172	60	224	224	222	28	98	223	223	224	213	179	225	224	224	158	0	173	225	224	224	224	223	226	223	110	218	224	224	158	2	193	224	224	223	224	223	226	179	121	224	223	224	223	194	1	155	224	224	224	224	224	224	224	224	224	224	226	225	122	138	0	149	226	224	224	223	223	224	225	183	1	185	225	225	208	0	76	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	224	224	224	225	224	223	223	224	226	222	225	224	222	238	208	188	237	220	212	210	206	216	228	223	224	226	224	226	207	197	220	227	224	223	224	225	224	226	224	224	226	225	224	225	91	157	223	227	205	18	1	134	224	224	225	224	120	145	223	225	225	224	223	89	42	222	225	225	218	215	225	224	226	224	211	0	114	223	223	225	224	225	224	219	48	217	225	223	223	221	37	66	223	224	225	139	0	212	224	225	222	222	44	76	222	225	225	225	224	224	224	227	209	1	89	224	224	222	225	162	90	225	112	0	170	224	223	224	213	47	224	213	8	33	218	223	224	224	224	162	0	206	224	224	224	223	225	224	171	62	225	224	222	32	35	214	224	223	225	224	223	223	212	5	63	223	224	223	225	224	223	223	223	217	223	224	224	157	0	193	224	224	223	224	223	225	179	120	223	223	224	225	193	1	154	223	224	224	224	224	224	224	224	224	224	224	214	78	223	52	48	222	222	225	223	224	224	225	181	0	187	224	224	206	2	95	225	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	224	224	225	225	225	225	225	224	224	224	224	226	224	222	225	239	215	205	233	232	220	216	228	228	229	232	227	225	225	228	217	191	193	209	227	230	225	226	223	225	226	221	222	224	223	224	92	155	225	221	225	198	11	0	163	224	224	224	120	143	227	224	223	224	224	91	35	222	227	218	106	163	224	224	223	225	208	1	111	223	225	224	223	225	223	218	48	217	225	224	223	218	35	75	227	225	194	0	134	223	224	225	224	205	1	136	224	222	225	223	224	224	225	223	221	32	5	212	224	224	225	169	76	224	210	2	42	224	224	225	131	148	223	216	16	31	218	224	223	223	224	161	1	207	224	224	224	225	224	223	169	61	223	224	224	127	0	21	167	220	223	224	224	225	145	1	169	223	223	225	224	224	225	224	223	224	223	224	224	157	0	196	223	224	224	223	224	225	179	119	225	224	224	224	193	0	152	224	224	224	224	224	224	224	224	224	224	225	166	127	223	170	0	199	224	224	225	223	225	224	182	1	194	223	221	110	32	213	223	224	222	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
225	225	225	224	224	224	224	224	225	224	225	222	225	225	224	223	228	218	207	214	223	215	222	228	228	228	229	229	226	223	222	228	226	203	189	200	221	227	225	224	224	223	224	222	226	222	226	91	157	227	222	223	224	176	1	1	192	220	225	119	145	224	222	223	224	223	97	5	101	93	76	4	200	223	225	223	224	209	1	111	224	224	223	224	223	224	219	47	218	223	224	221	222	42	30	135	83	26	174	223	224	225	224	224	168	1	155	224	225	224	223	224	222	224	223	222	54	0	183	224	223	224	178	67	223	219	54	0	205	223	222	54	221	224	216	24	31	217	223	224	224	226	161	0	206	223	224	224	223	225	225	168	61	223	224	223	222	111	2	1	56	197	223	223	223	108	1	181	225	224	222	224	224	224	223	227	224	224	225	223	156	0	195	224	223	224	223	224	225	179	120	223	224	224	222	191	0	153	224	223	224	224	224	224	224	224	224	224	221	91	212	222	216	7	107	227	223	224	221	224	226	183	0	93	125	45	79	217	224	223	223	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
226	226	224	223	224	224	224	225	225	225	223	224	226	224	226	228	229	211	208	228	199	183	220	227	226	224	221	227	224	224	223	227	226	226	216	190	185	205	220	223	225	226	227	224	224	223	225	91	155	225	225	224	225	222	153	0	11	203	226	120	144	224	224	224	223	223	90	44	220	227	222	120	178	225	224	224	224	211	0	109	224	224	224	223	225	222	219	49	216	224	225	224	220	41	57	209	119	0	191	223	224	225	224	223	145	0	142	224	225	224	224	225	225	223	225	222	59	2	192	224	225	223	176	113	224	224	172	1	105	228	169	114	224	224	214	23	31	219	223	225	223	225	163	0	207	223	224	224	224	226	223	170	62	225	225	225	225	223	199	52	0	1	165	224	226	109	0	173	223	224	223	225	224	219	224	224	224	224	226	225	155	0	193	223	224	224	224	224	225	180	119	224	223	224	224	192	0	152	223	223	224	224	224	224	224	224	224	224	211	20	139	130	134	8	8	219	224	224	224	223	224	182	0	161	190	15	89	221	224	224	223	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	223	225	225	225	224	224	225	224	224	223	226	225	224	221	226	227	201	204	221	189	168	227	227	223	222	225	224	225	226	223	224	224	225	223	227	203	184	204	225	223	225	224	225	224	226	220	91	158	223	224	223	223	227	224	125	0	32	209	134	141	222	224	225	224	223	91	45	221	221	227	200	198	224	222	224	224	212	0	123	223	224	224	224	223	224	220	45	217	224	224	225	221	36	70	225	225	79	17	219	227	222	226	223	185	0	112	224	225	223	224	224	224	225	224	218	32	31	222	227	225	224	152	126	225	226	219	16	13	218	74	203	224	224	216	21	31	221	223	224	224	225	162	0	205	225	224	224	224	224	225	187	56	224	227	222	225	224	224	221	164	2	3	205	224	149	0	177	225	225	224	224	223	225	224	225	223	223	223	224	155	2	193	225	224	224	224	224	224	177	119	224	224	224	224	193	0	153	224	224	224	224	224	224	224	224	224	224	165	88	182	177	181	157	0	156	224	225	223	225	223	181	1	187	224	194	0	132	224	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	225	225	225	224	223	224	223	227	224	224	224	226	225	224	219	228	214	209	185	186	220	235	225	222	225	223	224	227	224	223	223	224	226	225	228	204	191	213	226	223	223	223	225	222	224	90	159	223	223	221	225	223	223	224	95	0	59	76	148	225	224	224	226	223	89	43	223	221	222	224	225	225	224	224	224	213	0	128	224	224	224	224	224	224	203	67	223	224	224	224	220	35	65	225	223	208	1	103	223	225	222	225	217	0	43	221	224	224	223	223	224	224	223	219	2	160	224	223	223	223	110	127	223	224	225	122	1	73	74	223	225	225	215	23	31	220	224	223	224	225	160	0	198	223	225	224	223	224	225	193	82	223	224	224	223	224	225	225	225	100	0	181	223	210	3	65	222	222	225	223	225	224	224	223	224	223	223	224	155	1	185	224	223	224	224	223	225	171	127	224	223	224	224	193	1	153	223	224	224	224	224	224	224	224	224	224	67	198	224	224	223	223	47	58	224	224	224	225	224	180	0	189	224	223	99	7	205	222	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	225	224	222	222	225	223	223	224	223	224	225	225	226	229	226	221	233	220	230	182	180	216	232	223	224	222	222	232	227	225	224	224	225	224	224	222	224	198	196	223	223	223	223	224	225	224	93	160	224	225	225	223	225	223	225	220	62	2	0	159	222	223	227	223	224	93	44	223	224	227	223	224	223	203	224	224	218	13	58	224	224	224	224	225	224	175	95	225	225	223	224	221	39	66	224	224	224	132	0	192	224	222	223	223	151	1	149	224	224	224	225	224	224	224	116	57	222	224	223	225	224	120	107	225	224	224	209	5	1	171	224	224	225	213	22	34	223	224	224	223	224	199	0	128	225	223	222	223	224	222	91	163	226	222	228	208	224	223	225	225	150	0	187	221	225	150	0	171	225	224	224	224	224	225	223	122	220	224	225	172	0	175	222	225	225	223	226	223	90	172	223	224	225	223	192	0	154	225	224	223	223	225	204	215	225	225	194	44	221	225	224	224	225	171	1	207	225	224	224	222	184	0	189	224	222	211	3	71	222	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	226	216	197	214	227	226	214	221	224	226	227	225	229	227	229	213	225	212	214	204	170	215	228	224	224	223	225	226	223	224	223	224	223	221	225	224	226	220	220	230	227	223	225	224	225	222	67	159	224	224	223	224	225	224	225	224	209	35	0	156	225	224	224	225	225	70	26	221	224	222	224	225	164	136	224	225	223	146	0	151	221	226	221	224	197	24	193	223	223	224	225	220	31	71	225	224	224	221	22	33	219	225	225	223	223	78	1	165	224	223	224	224	222	166	52	217	225	224	224	225	224	75	82	224	225	223	225	70	2	219	224	224	223	221	23	16	219	222	224	224	222	227	69	0	189	225	225	224	222	170	40	219	222	224	216	46	218	224	223	224	75	45	218	224	225	224	129	0	175	224	223	225	224	224	183	44	222	223	224	222	38	18	198	225	222	224	221	156	51	223	221	224	223	223	200	0	116	223	224	223	224	218	59	220	224	224	138	95	225	222	225	226	223	215	10	111	224	223	224	226	184	0	189	224	224	225	158	1	157	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	227	217	197	214	226	225	212	221	222	227	227	223	224	233	231	205	214	215	209	187	215	227	221	224	223	224	225	224	224	224	225	225	224	224	225	224	223	220	219	227	224	223	224	222	177	98	11	35	120	199	225	223	225	224	224	222	224	207	5	145	228	227	223	196	122	11	8	94	152	144	130	97	0	206	224	224	224	225	122	4	35	96	98	68	31	176	222	222	224	225	197	126	12	36	140	204	223	224	189	2	46	169	223	224	223	222	154	27	90	176	197	162	101	105	222	224	226	224	223	157	114	10	29	124	203	224	227	173	75	223	225	224	171	102	10	9	78	137	220	224	223	225	217	73	1	59	104	95	54	47	202	224	223	228	222	74	35	172	190	86	60	213	224	225	223	223	227	174	42	92	166	199	178	103	31	168	225	224	224	222	204	59	1	70	103	96	47	60	218	225	225	224	224	193	111	11	16	106	149	135	126	39	97	224	216	139	23	45	177	223	223	225	221	138	27	14	117	209	222	151	76	12	81	169	224	227	221	62	0	122	211	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
225	226	216	198	213	227	223	210	221	224	223	223	223	228	229	230	213	208	229	218	197	229	225	226	220	222	222	219	221	225	223	224	222	224	226	225	223	223	225	224	222	224	226	223	225	213	203	215	212	197	213	227	224	223	224	225	224	223	224	172	183	222	225	225	210	189	208	214	206	203	202	204	206	210	223	223	223	224	225	224	214	176	138	140	187	220	225	223	223	224	224	207	194	214	210	190	211	224	224	223	204	206	198	222	224	223	226	222	219	192	149	145	183	216	225	224	221	224	225	223	199	203	212	214	192	215	223	224	219	210	224	222	220	209	193	214	213	192	197	220	224	224	222	224	224	209	171	136	156	194	224	225	225	224	223	227	222	200	149	153	197	225	223	225	222	225	223	224	223	222	208	168	139	160	195	223	224	224	223	226	224	224	222	207	161	131	160	210	223	222	223	224	225	225	211	197	215	212	209	201	192	205	200	220	223	214	191	213	212	206	224	223	223	221	193	215	210	197	213	219	192	206	214	207	203	222	222	225	221	205	196	211	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
227	224	221	208	216	226	227	215	220	224	224	224	225	228	227	225	222	201	233	191	210	229	225	227	218	222	224	218	223	224	224	226	224	223	224	224	224	224	225	224	224	223	223	224	221	224	223	224	225	223	224	224	223	225	225	223	224	224	224	222	223	225	224	224	225	226	224	223	228	224	224	224	221	225	224	226	224	222	225	223	224	224	224	224	225	221	224	222	225	224	225	224	223	223	224	223	225	225	225	224	224	223	224	224	227	224	226	223	226	224	225	223	224	224	224	221	224	224	224	224	224	224	223	225	224	224	224	224	224	226	225	225	224	223	222	225	223	224	222	225	225	223	224	224	223	224	223	224	226	223	224	224	224	223	224	224	223	223	223	225	224	224	223	224	224	224	223	224	223	223	224	224	224	223	223	224	225	224	223	225	223	222	226	224	222	221	225	223	224	225	224	223	224	222	225	224	226	223	225	223	224	225	223	224	223	226	223	223	222	224	223	225	223	224	227	224	224	224	224	223	225	223	224	224	225	220	225	223	226	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	223	226	227	226	225	227	224	223	227	225	223	224	228	224	222	215	185	220	211	229	223	222	223	229	223	220	218	228	225	224	224	223	225	223	222	224	224	224	224	223	224	224	224	224	224	223	225	223	223	223	222	226	224	223	225	225	224	224	224	224	223	224	224	225	224	222	224	223	223	223	224	225	224	225	225	222	226	227	224	223	225	224	224	223	224	227	223	223	224	225	224	223	225	225	223	225	224	224	227	223	223	224	225	222	223	223	225	225	223	223	225	225	222	223	224	224	226	224	224	224	224	224	223	224	224	223	223	224	222	224	226	223	224	224	225	225	223	224	224	223	227	225	225	225	224	224	226	225	224	226	224	224	225	224	224	225	224	224	223	224	225	224	224	224	225	223	224	224	226	225	224	225	224	224	224	224	223	225	224	224	224	225	224	225	223	225	223	224	223	225	224	224	225	224	224	224	223	223	225	225	222	225	224	225	224	225	224	223	224	225	227	224	224	225	222	224	222	225	224	224	224	222	224	225	223	225	223	224	226	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	223	223	226	222	224	222	223	224	223	223	228	220	225	224	229	231	186	215	224	224	223	224	225	224	223	225	223	225	222	223	224	224	225	223	223	225	224	224	225	224	226	225	225	225	209	191	218	220	202	190	197	213	223	224	224	225	223	225	225	224	223	223	224	223	225	225	222	227	223	224	224	224	224	223	222	226	222	225	224	224	223	224	224	224	224	221	227	224	223	224	223	225	224	223	224	222	224	225	226	223	226	223	224	224	225	223	223	225	227	224	224	224	222	224	227	223	222	225	224	224	224	225	225	225	224	225	223	224	225	224	223	222	224	225	222	225	223	225	223	226	222	223	225	222	224	227	223	223	226	223	223	223	225	224	224	222	224	223	223	224	223	224	223	225	222	223	224	224	224	224	223	224	223	225	225	225	223	224	223	223	224	224	225	223	224	223	224	224	225	223	224	225	224	224	223	224	224	224	224	223	224	223	225	221	225	224	227	209	194	217	217	218	193	205	223	224	225	225	224	225	223	224	227	223	223	223	225	223	224	221	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	226	227	226	224	226	225	226	228	226	227	226	225	230	201	214	193	224	232	225	225	230	224	225	224	224	226	227	228	227	228	224	224	223	225	224	223	225	223	224	223	223	224	205	158	90	14	0	0	94	113	86	52	148	219	224	224	221	224	225	225	224	223	223	225	222	225	223	225	225	224	225	224	224	224	227	224	225	224	224	227	223	228	224	223	224	224	224	224	224	223	227	226	222	225	225	225	224	223	224	225	224	224	224	223	225	224	223	224	223	225	227	224	225	224	221	224	223	224	225	222	224	223	224	223	227	224	225	224	224	224	224	224	226	224	224	225	224	224	224	225	224	223	224	224	224	225	224	223	225	225	225	224	224	223	224	225	223	225	224	224	224	225	222	225	225	224	224	224	222	224	225	224	224	224	224	224	225	224	224	225	224	224	223	223	224	224	224	222	224	223	224	224	224	225	224	223	223	225	225	226	223	223	225	224	224	223	155	75	4	1	14	96	165	224	224	224	224	225	222	224	227	221	224	224	223	224	224	224	225	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
225	228	227	228	230	228	230	228	228	231	230	231	226	231	221	146	209	188	217	229	230	227	230	228	228	230	231	228	229	228	228	230	224	224	224	224	224	224	224	224	224	224	224	224	223	225	223	153	1	100	224	224	224	157	0	97	223	223	223	225	222	225	224	224	224	225	223	224	223	223	224	225	223	225	223	225	223	226	223	224	224	225	224	223	224	224	223	224	224	223	223	223	224	225	224	224	227	222	224	223	223	225	225	222	225	223	225	224	224	224	225	224	225	224	222	224	226	224	223	223	223	224	225	225	222	224	223	224	226	223	227	224	223	224	224	224	225	224	224	223	224	224	225	223	222	225	224	223	225	224	225	223	224	225	226	223	224	225	224	224	224	225	223	224	224	225	223	225	223	226	225	223	224	224	223	225	225	224	223	222	224	225	224	224	223	227	224	223	227	224	224	223	224	224	224	224	227	224	224	223	224	224	224	224	224	224	224	224	224	224	80	1	143	225	223	224	224	224	224	224	224	224	224	224	224	224	224	224	225	224	222	223	223	223	223	224	223	224	223	223	224	222	225	222	224	224	225	224
224	224	224	224	223	224	225	224	224	223	224	222	226	230	186	161	218	187	217	229	224	223	224	225	224	224	224	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	223	224	225	223	224	165	1	119	224	224	224	226	61	2	196	227	223	225	212	202	213	211	199	222	224	226	225	225	224	222	197	147	109	162	204	222	223	227	223	223	227	208	205	210	217	224	225	224	224	224	225	223	224	206	206	204	227	226	206	202	210	211	203	199	199	198	204	222	227	224	223	225	224	224	221	202	156	111	158	199	216	224	224	223	203	207	210	207	199	222	227	224	224	209	208	209	210	208	220	224	225	225	224	223	224	201	211	225	224	224	223	224	224	222	199	204	211	220	225	223	224	224	224	223	205	207	211	203	222	223	198	210	212	205	205	227	222	227	222	224	223	216	193	145	110	171	207	220	224	225	223	225	222	196	140	128	199	221	224	224	224	224	224	224	224	224	224	225	112	1	163	224	224	224	224	224	224	224	224	224	224	224	222	225	226	223	187	219	224	224	224	223	225	225	215	202	211	211	202	206	193	215	225	225	223	224
224	224	225	229	224	224	224	224	224	225	224	224	226	227	189	153	223	205	205	227	225	224	223	224	225	224	223	223	225	224	225	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	164	0	118	224	222	224	223	96	0	183	225	226	222	164	87	7	29	102	205	224	225	225	223	191	69	121	184	199	150	60	61	195	225	224	224	220	153	107	1	43	223	224	225	224	222	223	224	137	0	54	123	223	225	150	52	3	40	117	109	113	108	4	207	224	223	223	223	224	178	41	74	140	171	142	62	17	161	224	222	137	48	5	59	115	218	223	225	225	165	85	4	40	116	214	224	222	225	224	225	219	14	81	223	222	224	223	224	225	203	95	9	0	64	221	224	224	224	225	223	128	73	31	119	214	219	118	55	3	74	138	223	224	225	225	225	165	32	91	150	168	135	48	36	198	224	224	222	84	90	158	115	1	169	224	224	224	224	224	224	224	224	225	225	112	1	163	225	224	224	224	224	224	224	224	224	224	224	225	222	223	202	0	160	225	227	223	224	223	224	192	123	46	0	76	112	115	33	176	224	224	224
223	225	220	215	222	224	225	224	224	223	225	224	217	192	157	160	235	206	207	229	219	217	225	228	224	223	220	219	230	226	223	224	224	224	224	224	224	224	224	224	224	223	224	225	224	224	224	163	1	117	223	224	224	224	65	0	201	224	223	226	225	221	22	114	225	223	222	224	224	179	41	206	225	221	225	225	221	139	0	168	225	223	224	225	224	52	1	179	223	223	226	225	222	222	21	1	145	225	225	224	225	213	0	146	223	224	224	223	125	200	223	224	225	223	142	2	175	222	223	227	227	222	136	39	223	227	227	191	1	179	224	225	223	224	225	224	208	0	143	224	224	222	225	224	224	226	169	0	1	213	226	226	225	224	223	224	224	188	0	0	81	224	225	223	223	223	224	181	118	224	223	224	225	189	3	194	224	224	223	224	221	95	7	189	224	223	226	224	220	116	91	222	225	109	113	223	223	224	144	136	224	224	224	224	224	224	224	224	225	223	113	0	165	222	226	224	224	224	224	224	224	224	224	224	225	223	225	107	0	38	219	222	224	224	224	223	224	224	160	0	211	224	224	159	1	198	224	222
224	224	211	195	216	227	226	228	226	224	226	226	199	155	155	205	231	205	228	228	224	221	229	224	225	226	223	221	229	225	223	223	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	164	0	126	223	222	225	164	0	135	225	223	224	226	224	223	24	113	223	224	225	224	205	1	189	223	225	225	223	224	226	225	112	0	193	224	223	223	224	65	0	52	223	224	224	224	224	181	105	10	142	224	224	224	222	215	2	146	224	224	228	224	212	218	224	223	224	169	2	170	223	225	225	222	223	227	221	121	222	223	225	197	0	177	225	224	221	224	224	224	205	0	145	223	224	225	222	225	223	224	114	152	2	156	222	224	224	225	223	224	223	191	88	113	0	132	224	224	224	224	223	178	121	225	225	223	223	187	0	191	225	224	224	223	109	0	191	225	223	224	226	224	224	221	141	224	223	10	170	225	225	224	215	200	224	224	224	224	224	224	224	224	224	224	112	1	164	224	225	224	224	224	224	224	224	224	224	224	225	224	222	85	142	1	195	225	223	224	224	224	224	225	157	1	211	222	225	214	0	126	223	227
224	226	211	189	214	228	226	229	228	225	226	218	178	178	212	224	219	217	238	226	227	225	221	225	223	222	224	226	223	222	224	225	224	224	224	224	224	224	224	224	224	224	224	223	224	225	224	163	0	56	152	142	82	2	172	224	224	223	225	224	224	222	26	115	225	224	224	223	80	56	225	223	224	224	224	223	223	224	217	0	61	224	224	224	218	77	147	2	208	225	223	223	225	90	199	0	139	224	223	224	225	212	2	150	224	224	225	203	223	224	224	225	215	11	55	227	224	223	223	224	223	225	225	223	224	226	223	196	0	177	223	224	225	223	225	224	207	1	146	223	224	226	224	225	223	215	78	224	95	27	221	224	222	223	225	224	225	189	90	219	84	2	166	226	223	223	223	179	119	224	223	224	225	190	0	192	224	224	223	199	0	97	223	224	224	223	224	224	225	223	222	223	221	24	50	210	225	223	226	222	224	224	224	224	224	224	224	224	223	225	111	2	163	224	223	224	224	224	224	224	224	224	224	224	223	223	181	127	223	12	99	225	224	224	223	224	225	224	157	0	211	223	224	203	0	157	225	223
223	224	214	198	218	229	229	227	226	224	226	208	164	196	227	229	216	214	229	222	224	224	223	228	223	226	226	224	224	226	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	223	224	163	0	53	160	160	130	67	44	132	221	224	223	223	224	222	25	112	223	225	224	210	2	149	224	224	224	224	223	224	225	224	224	47	2	206	225	223	212	80	217	0	98	224	224	224	200	137	215	1	140	223	224	223	223	216	1	138	223	220	200	99	219	224	223	225	172	1	138	224	224	223	225	224	224	225	225	226	221	224	224	195	0	171	226	222	221	222	224	222	204	0	142	224	224	223	224	223	224	143	133	223	180	0	188	225	223	224	224	222	223	190	75	224	218	55	0	180	225	225	227	175	121	223	224	225	223	190	1	192	223	223	224	103	0	184	224	224	225	221	223	223	225	224	223	224	225	133	0	30	166	218	224	225	224	224	224	224	224	224	224	224	225	223	112	2	163	224	223	224	224	224	224	224	224	224	224	224	224	225	90	189	225	108	0	209	225	224	224	227	226	224	158	1	206	223	188	94	128	225	224	224
227	225	218	215	222	225	224	222	225	227	222	195	202	224	225	221	212	233	227	222	224	227	230	228	231	227	226	224	228	222	223	225	224	224	224	224	224	224	224	224	224	224	224	223	224	224	224	163	1	121	223	226	224	223	143	0	85	225	225	224	225	222	25	114	223	223	225	176	0	149	225	225	223	224	224	225	223	224	223	123	1	197	223	225	210	79	224	117	0	218	223	224	102	217	218	10	94	224	223	225	224	213	23	39	86	80	34	78	223	225	225	221	137	0	151	224	224	224	223	224	224	224	224	223	223	225	224	197	0	50	102	96	95	95	95	98	94	0	145	223	225	224	225	224	224	65	202	225	224	33	93	222	225	224	223	223	223	189	82	223	222	214	28	12	207	223	224	177	121	225	224	224	224	187	0	190	224	223	221	31	1	212	224	225	224	224	224	225	224	224	224	225	224	221	107	0	0	59	191	224	224	224	224	224	224	224	224	224	225	224	112	1	164	224	225	224	224	224	224	224	224	224	224	224	225	219	67	223	222	212	2	157	224	225	224	225	223	223	159	0	63	80	35	63	202	225	222	225
225	221	226	225	224	224	227	224	226	223	194	210	229	230	227	218	206	216	224	225	223	228	227	222	230	224	222	225	225	225	223	224	225	224	224	224	224	224	224	224	224	224	222	224	224	223	223	164	1	113	225	224	222	223	223	78	1	191	223	224	223	223	24	115	224	225	223	167	0	154	225	223	223	224	224	222	223	225	224	124	0	199	225	224	190	74	224	211	0	147	224	211	103	225	220	15	84	225	225	222	225	213	4	148	224	222	216	119	220	224	224	225	137	0	154	225	223	225	223	225	225	224	224	228	223	223	225	196	1	172	224	221	222	222	222	221	207	0	145	224	224	224	224	224	214	9	140	168	170	41	1	207	225	222	224	225	225	189	83	224	225	224	208	10	33	223	223	176	121	224	224	225	224	187	0	193	225	225	218	37	1	211	224	224	225	225	224	225	224	224	223	224	224	224	224	187	64	0	0	181	223	225	225	224	224	223	224	224	223	225	111	0	164	224	223	225	224	224	224	222	224	225	224	223	225	182	19	165	165	152	0	37	223	224	224	224	224	225	159	0	197	225	219	143	4	135	223	224
225	226	224	225	225	224	226	223	225	204	200	227	227	227	226	220	215	224	225	225	223	225	223	225	223	223	226	224	224	223	226	225	224	224	224	224	224	224	224	224	225	225	223	225	223	225	225	164	1	116	222	225	227	222	225	106	0	113	224	225	225	222	25	115	225	224	223	190	2	98	224	224	225	224	225	223	224	224	223	56	21	219	223	225	193	73	223	223	59	20	222	133	191	222	219	15	86	225	224	224	224	214	2	148	225	223	225	203	222	222	223	224	181	1	144	224	225	224	226	220	223	224	224	224	225	223	223	196	2	179	223	224	227	224	224	225	207	0	146	223	224	224	224	224	145	102	178	173	173	164	0	154	224	225	223	223	224	190	82	223	223	223	223	199	0	65	222	182	121	225	223	223	224	190	0	192	223	224	227	114	0	182	221	225	223	224	224	224	224	222	227	224	227	225	224	225	222	162	1	22	218	223	223	224	224	224	223	225	224	225	114	0	162	226	224	225	224	224	223	225	222	225	224	223	224	88	143	176	174	177	127	0	184	224	225	225	225	224	160	0	211	224	223	223	146	0	173	223
223	227	225	222	223	227	224	226	199	198	222	226	224	224	234	215	227	227	223	224	222	224	225	223	224	224	224	222	225	222	224	224	223	223	224	224	224	224	224	224	224	225	225	224	224	224	223	161	0	119	223	223	223	223	224	107	1	138	220	224	225	221	27	114	222	225	225	224	42	6	211	224	223	223	223	224	224	225	223	0	115	224	225	223	174	62	225	224	185	1	170	90	217	225	220	15	93	223	224	222	225	215	0	144	224	224	224	224	223	223	227	224	218	16	49	219	223	224	224	225	225	225	224	221	220	224	224	197	2	176	224	221	223	221	224	225	204	0	144	223	222	227	224	223	73	203	225	223	224	225	85	30	220	224	225	225	224	191	82	224	224	225	223	225	180	0	112	184	118	224	225	225	223	187	1	192	225	223	223	204	0	78	225	224	225	223	223	224	221	223	219	224	222	225	225	224	225	222	130	0	184	223	225	223	224	223	225	225	224	225	113	0	168	222	224	223	223	226	224	222	210	224	223	224	219	48	217	223	223	224	222	19	115	227	224	224	223	223	159	2	209	223	225	222	192	1	127	227
224	222	223	228	227	226	225	212	197	222	226	227	225	228	229	213	226	226	225	226	224	227	226	225	226	225	228	226	227	227	225	223	224	223	224	224	224	224	224	224	227	224	224	226	223	224	224	167	0	121	224	224	225	226	222	57	0	204	225	224	225	223	25	117	225	224	225	223	166	4	100	223	225	224	223	225	223	223	140	38	223	224	224	224	172	63	225	224	222	26	0	144	225	227	218	9	114	224	224	225	223	215	1	149	224	225	223	226	225	202	197	223	225	167	0	134	225	224	224	224	226	224	223	144	207	222	223	197	0	177	226	222	224	224	225	223	207	0	144	223	226	223	223	219	55	218	223	225	227	223	172	0	199	223	225	223	224	191	83	224	223	223	226	225	223	141	1	29	132	224	223	225	224	190	1	193	224	225	225	223	127	1	179	225	224	223	224	225	225	225	104	220	219	209	224	224	224	224	167	0	205	223	223	224	223	224	224	224	225	223	112	0	157	223	223	224	223	223	225	187	98	223	225	225	176	82	223	224	223	224	223	110	3	214	224	224	224	223	162	0	207	225	226	223	180	1	140	223
224	225	224	224	224	226	207	194	219	229	223	226	222	229	214	217	225	221	225	225	225	222	225	227	224	225	223	221	226	227	226	224	225	224	224	224	224	224	224	224	223	224	223	225	224	226	224	155	0	57	221	223	224	212	84	0	116	222	224	222	224	223	20	90	223	224	223	222	225	134	1	128	222	223	224	224	224	160	42	215	223	224	224	225	185	59	225	224	224	131	0	220	223	226	221	4	93	223	227	223	225	207	1	125	225	224	225	224	218	61	208	224	224	224	152	2	141	222	225	223	227	224	185	40	223	225	225	197	0	177	224	226	225	223	224	222	212	0	135	227	224	227	223	150	65	225	224	223	224	224	221	24	102	225	226	223	226	190	78	225	224	223	223	225	223	224	117	0	128	224	224	223	225	192	0	193	224	223	222	225	222	123	0	155	223	224	225	225	224	136	80	225	217	58	216	227	225	221	58	90	224	224	223	224	224	225	223	224	223	223	96	0	73	217	224	224	223	224	218	31	171	225	223	222	73	153	224	224	227	223	225	204	0	168	225	225	224	223	157	0	200	222	223	209	60	50	217	227
224	221	224	223	224	225	211	199	224	225	223	223	228	227	206	219	224	225	224	225	223	211	228	228	220	218	222	209	226	228	227	224	225	224	224	224	224	224	224	224	224	225	225	224	215	183	116	26	10	3	51	107	95	30	57	155	222	223	226	224	190	149	12	13	144	209	224	226	223	224	174	57	57	141	159	153	110	121	215	225	223	224	223	175	81	28	167	219	225	201	69	228	223	210	154	11	17	166	216	222	178	67	7	14	111	139	132	130	56	82	224	227	224	224	224	183	75	80	145	161	149	107	36	155	224	222	180	83	11	61	183	218	220	224	224	190	106	8	31	166	220	219	155	24	29	164	218	224	224	219	164	29	2	119	215	224	176	88	18	153	207	225	223	223	226	222	224	73	105	224	222	225	184	90	11	66	180	222	224	222	224	223	167	57	94	158	157	155	77	59	184	224	226	74	45	150	165	86	94	214	224	225	225	226	224	223	224	224	184	106	12	8	4	46	107	112	107	99	31	21	216	224	213	132	6	74	175	224	223	222	211	144	8	13	148	223	211	162	45	10	54	133	121	50	97	212	223	225
227	224	224	225	223	225	214	193	203	225	222	226	226	225	216	221	222	225	224	223	224	226	225	225	226	226	225	227	225	224	224	224	224	224	224	224	224	224	224	224	225	225	223	226	219	198	181	218	224	221	196	180	194	222	223	223	225	224	223	223	199	194	222	225	191	215	225	225	226	224	224	225	198	168	145	180	216	223	226	224	224	224	222	193	220	223	205	219	225	219	212	223	224	212	191	223	221	180	217	223	189	199	222	225	219	219	197	179	186	219	225	223	224	223	224	224	224	212	170	143	157	199	220	223	223	226	195	208	223	211	179	220	224	224	224	192	203	224	213	176	226	221	188	220	221	181	222	223	225	217	198	223	222	192	215	224	191	212	223	202	212	224	223	227	223	225	225	218	186	222	224	223	183	214	224	216	184	223	224	224	225	223	225	223	202	163	147	168	193	224	223	224	225	221	200	149	155	206	223	224	224	224	224	223	223	224	224	224	207	180	215	221	222	220	222	208	185	185	189	210	224	228	213	194	223	211	191	224	224	226	209	204	224	221	185	221	212	181	218	223	205	177	202	220	224	224	224	224
223	223	225	224	223	223	228	197	189	226	224	226	224	216	213	225	225	224	224	224	223	226	227	222	221	224	225	219	221	223	223	224	223	224	224	224	224	224	224	224	223	225	223	224	225	224	224	224	223	224	225	224	223	222	224	224	224	225	223	225	223	225	223	223	223	224	224	224	223	224	223	225	223	223	224	223	224	224	224	224	224	223	224	223	224	223	225	224	223	224	223	226	225	225	225	224	226	225	224	224	224	225	223	224	224	224	227	223	224	224	225	223	223	225	225	224	224	224	222	225	225	224	225	223	224	224	223	223	224	224	227	224	223	225	223	224	224	225	223	223	224	224	224	225	225	224	223	223	224	225	223	225	227	224	222	224	226	223	223	225	223	224	225	224	224	224	222	224	223	225	224	224	224	223	225	223	224	223	223	224	224	223	223	224	224	222	227	223	224	224	222	225	224	223	223	225	225	223	225	225	223	223	224	223	223	224	223	224	223	224	224	223	224	224	224	225	223	224	223	224	225	222	224	225	224	225	223	224	224	224	224	224	224	225	224	225	226	222	224	225	225	223	223	223	224	224	227	224
225	223	223	224	224	224	226	199	199	225	224	225	224	212	214	226	224	224	224	224	224	224	229	220	214	225	222	215	214	225	225	221	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	225	223	225	226	224	226	212	200	224	224	226	210	219	231	223	224	224	224	224	225	224	226	226	225	226	225	226	225	225	223	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	223	224	224	225	223	222	225	217	225	224	229	211	205	219	226	224	224	224	224	223	224	222	224	226	223	223	223	224	223	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	223	225	223	224	225	224	222	225	224	225	223	226	210	222	226	224	224	224	225	223	225	225	223	224	224	224	223	225	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
225	224	224	224	225	224	224	225	223	224	223	225	222	206	210	223	224	224	223	224	224	224	225	225	223	223	223	224	224	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
225	223	224	224	223	223	225	222	224	225	224	224	223	226	217	224	225	224	225	224	224	224	224	224	224	225	225	224	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	226	222	224	224	222	225	227	227	225	224	224	221	226	226	223	224	224	225	223	223	224	224	223	224	224	223	223	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	223	223	228	223	221	221	223	224	227	222	226	223	224	224	224	224	224	224	224	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	223	226	224	224	224	224	224	224	224	223	224	224	224	225	224	224	225	223	224	224	226	224	224	224	224	224	224	224	224	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	225	222	224	224	224	222	224	223	223	225	224	223	223	222	224	225	224	225	225	224	224	225	224	225	224	224	224	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	225	225	224	224	225	225	224	224	224	223	224	224	225	226	225	224	223	224	223	224	224	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
223	224	223	225	224	223	224	224	224	223	224	224	224	225	224	224	225	224	224	224	224	223	225	224	225	224	223	224	224	224	224	224	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	223	223	225	224	224	220	225	223	225	225	225	223	224	224	224	224	226	225	226	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
225	223	225	224	225	225	228	224	224	223	223	225	227	230	213	208	208	214	218	223	226	228	226	225	225	226	227	226	225	224	224	225	224	224	223	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	225	227	232	230	217	231	236	234	228	221	223	216	219	214	199	201	197	206	223	230	228	228	227	225	225	225	225	225	224	224	224	226	226	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	226	229	229	223	224	226	229	231	224	201	202	208	225	227	205	197	205	210	217	225	224	223	225	224	224	224	224	224	224	224	224	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	225	224	224	224	224	224	223	227	218	209	213	215	220	216	211	223	226	226	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	224	224	224	224	224	224	225	230	229	229	218	214	221	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
224	224	224	224	224	224	224	224	224	225	223	225	225	225	226	225	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224
];
logoGreen = [223	224	223	226	214	147	183	221	226	224	224	224	224	224	224	224	224	224	224	224	225	225	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	225	232	139	91	131	188	227	225	224	224	224	224	224	224	224	224	223	223	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	226	223	231	128	93	121	220	225	223	224	224	224	224	224	224	224	225	224	224	224	224	225	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	223	225	228	137	69	168	215	225	224	224	224	224	224	224	224	224	224	225	224	224	224	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	226	225	227	126	30	122	197	226	224	224	224	224	224	224	224	224	224	224	223	223	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	225	224	234	105	50	166	228	224	223	224	224	224	224	224	224	223	225	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	223	222	237	124	132	229	225	224	224	224	224	224	224	224	224	224	225	225	225	222	222	222	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	226	225	237	116	79	215	228	224	224	223	224	224	224	224	224	225	223	224	220	224	233	229	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	226	224	237	117	67	212	228	224	224	225	224	224	223	225	224	224	223	212	222	245	254	243	228	224	224	223	225	224	225	223	224	225	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	222	224	236	117	69	210	227	224	224	223	225	224	224	224	223	225	218	194	244	235	185	238	217	227	224	226	224	224	223	226	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	220	206	226	131	73	182	211	221	224	220	215	222	216	225	201	206	204	160	180	214	218	205	174	218	217	217	220	216	222	207	218	223	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	223	221	228	157	48	186	230	221	225	222	222	221	224	223	220	224	194	92	156	188	231	202	212	221	220	220	224	222	224	216	223	223	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	225	228	199	71	150	234	230	226	224	225	226	222	222	224	230	199	23	140	228	226	202	224	223	225	223	226	224	225	226	223	224	224	224	224	224	225	225	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	224	225	226	210	137	85	125	187	221	228	229	229	229	228	227	238	169	24	63	87	191	216	228	222	224	225	224	223	224	224	224	225	224	224	224	224	224	225	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	224	224	224	224	212	210	174	101	87	157	192	195	194	205	190	173	67	24	66	68	110	216	225	222	222	223	225	224	224	225	226	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	222	222	223	221	225	226	229	229	194	152	104	82	158	184	48	63	59	57	89	28	63	199	233	223	222	222	222	222	222	221	223	223	224	224	224	223	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	223	221	221	221	220	220	222	221	228	227	218	98	74	135	113	45	48	108	91	123	98	103	200	219	225	222	220	221	222	220	221	222	224	224	223	224	224	224	224	224	224	223	224	223	223	223	222	223	223	224	222	224	223	224	223	223	222	223	223	224	222	223	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	222	223	224	223	222	223	223	223	223	223	223	222	222	222	223	223	224	223	223	223	224	223	224	223	223	224	223	223	224	223	223	222	223	223	223	224	223	223	224	223	223	222	224	223	223	225	222	223	223	222	223	223	224	222	223	223	223	224	223	225	223	223	224	223	223	223	224	223	223	224	224	223	223	224	223	223	222	223	222	223	223	222	224	223	222	223	223	223	223	222	223	223	224	222	224	223	223	223	224	223	223	223	222	224	224	223	222	223	223	223	224	224	224	223	223	223	222	223	223	224	222	223	223	223	223	223	223	223	223	223	222	223	223	222	223	223	223	222	223	222	225	222	222	223	223	224	222	223	222	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	224	225	223	223	224	223	224	224	224	237	174	12	111	130	85	84	124	152	147	122	65	104	119	214	228	226	226	224	224	223	224	225	224	223	224	224	224	224	224	223	223	224	222	225	223	224	224	223	222	224	222	224	223	223	223	222	223	223	223	224	223	222	222	224	224	224	224	223	222	222	224	224	223	222	222	223	222	223	221	222	224	223	223	223	223	223	222	224	223	222	223	222	223	222	223	223	223	222	223	222	224	224	222	223	224	223	223	223	223	223	223	223	223	223	223	222	223	224	224	223	224	223	223	224	224	223	223	223	223	223	224	223	222	222	223	223	223	223	225	223	223	223	223	223	223	223	224	223	223	222	221	224	224	224	224	224	224	222	221	224	223	223	224	223	224	224	224	223	223	224	222	221	222	224	224	223	223	224	222	224	223	223	223	223	222	223	223	222	222	223	223	222	224	224	224	224	223	223	223	223	223	223	223	223	223	223	221	224	222	223	223	223	223	222	224	224	225	222	223	225	223	224	222	222	221	224	222	223	223	222	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	225	225	226	225	224	223	225	224	224	225	227	208	46	70	77	118	155	210	163	166	175	87	47	37	70	145	223	225	224	223	225	224	224	225	224	224	224	224	224	225	224	223	221	224	223	224	222	223	222	222	223	222	224	223	223	223	223	223	223	225	222	222	224	225	222	222	222	223	223	221	223	223	222	224	224	223	223	222	223	223	224	224	222	223	224	223	224	222	222	224	223	223	223	223	223	224	223	224	224	222	224	224	223	224	224	222	222	221	224	222	223	223	224	223	223	221	223	223	223	224	223	223	224	221	223	223	223	222	222	223	223	223	221	225	224	224	222	224	224	223	224	223	223	225	223	222	223	224	222	223	223	223	223	222	223	223	223	222	224	224	223	222	224	224	223	224	222	223	224	224	223	224	223	223	223	222	223	223	224	224	222	223	222	224	224	223	223	222	224	224	221	224	223	223	223	222	223	224	223	223	223	223	223	223	223	223	223	224	223	223	222	223	224	222	224	223	224	223	223	223	222	223	223	223	225	224	223	222	222	224	222	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	221	215	210	216	224	223	216	223	224	224	223	230	103	35	78	133	167	217	193	166	224	157	90	52	28	87	169	228	224	225	224	223	224	224	224	224	224	224	224	223	223	223	226	192	161	168	180	189	219	224	222	222	224	223	222	223	192	165	188	186	165	192	223	222	223	223	223	224	224	223	224	222	225	225	223	223	222	222	224	224	223	221	223	223	223	222	223	224	223	223	223	224	223	223	222	223	223	222	223	223	224	223	223	221	223	223	224	222	224	225	223	224	222	223	224	223	223	224	223	192	167	189	184	201	223	224	224	223	224	224	223	223	224	216	178	188	175	180	222	222	223	224	223	223	223	224	222	222	224	223	223	222	225	222	222	225	223	222	222	223	223	222	223	222	225	222	223	223	223	223	225	222	222	224	223	225	222	223	223	221	223	222	224	223	224	223	223	223	223	223	223	221	224	223	224	224	222	223	225	223	223	223	223	223	223	223	223	223	224	223	222	224	224	222	223	223	223	223	223	224	224	223	223	224	223	222	224	223	223	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	222	196	172	204	229	219	191	216	224	223	222	232	173	50	63	138	181	203	196	174	225	190	123	85	92	86	52	162	236	225	224	224	224	224	224	224	224	224	224	224	224	224	223	167	113	39	27	40	142	223	225	223	223	223	224	223	162	106	53	55	112	159	224	223	224	223	224	223	222	223	222	223	223	223	224	223	225	223	223	223	223	223	224	223	223	224	224	223	223	222	222	223	223	222	224	222	224	224	223	222	223	223	223	222	224	222	223	223	224	222	223	223	224	223	223	224	223	222	224	163	106	55	37	65	219	225	221	222	222	223	223	224	222	146	23	54	113	143	221	223	224	222	224	223	222	222	224	224	223	222	224	223	224	223	225	222	222	223	224	222	224	223	223	224	223	223	222	223	221	223	222	223	225	222	223	224	223	221	222	224	224	224	224	222	223	223	223	223	223	225	222	223	224	222	223	222	224	223	224	223	223	223	223	223	223	223	223	222	223	224	222	223	223	223	222	224	222	224	223	222	222	224	223	224	223	222	223	223	223	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	197	171	205	228	217	190	215	225	225	222	223	218	105	104	132	141	156	222	189	151	173	121	64	46	101	118	77	197	229	224	224	224	223	224	224	224	224	224	225	223	223	223	223	223	204	55	29	19	158	222	223	223	223	223	222	224	223	112	120	222	223	223	223	224	223	221	224	222	222	223	223	223	224	222	224	223	224	223	223	223	222	222	222	225	223	223	223	223	223	223	224	222	224	222	224	223	222	222	225	223	224	221	224	221	223	222	223	224	223	223	223	222	223	224	222	222	224	222	223	223	185	26	24	189	222	223	224	223	223	224	224	218	48	25	67	224	223	223	223	222	223	223	224	222	225	223	222	223	223	224	222	223	223	223	223	222	221	225	223	222	223	223	223	222	224	223	223	223	223	223	223	222	223	223	223	224	222	222	224	223	223	224	224	223	222	222	223	224	223	223	224	223	225	222	222	223	223	222	223	223	223	223	223	223	223	223	224	222	225	223	224	222	223	223	222	224	224	224	223	224	224	223	223	223	222	223	223	224	223	222	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	196	173	204	228	217	191	216	227	224	222	221	231	174	92	129	185	200	189	150	103	117	148	108	151	237	172	176	123	212	229	225	224	224	224	224	224	224	224	224	223	223	221	223	222	223	126	35	27	26	187	222	222	223	224	223	222	222	136	157	222	224	223	216	201	217	223	211	201	200	201	205	221	223	223	209	216	222	218	201	219	223	222	223	219	215	222	214	211	222	214	205	221	221	212	201	217	223	223	222	223	223	224	223	223	222	223	223	207	164	147	185	215	223	222	223	224	222	223	223	223	210	34	22	91	224	223	224	224	223	222	223	180	33	33	78	222	223	223	223	204	218	221	218	204	220	222	223	222	213	219	221	208	220	222	223	222	212	172	160	206	222	222	223	223	223	224	222	225	213	173	147	182	208	223	224	223	221	204	219	221	214	205	223	223	223	224	215	220	222	208	220	223	206	213	225	218	202	218	223	223	223	223	223	223	223	223	222	225	222	206	211	224	223	222	223	225	223	220	200	215	221	218	202	208	221	223	223	223	221	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	225	205	189	209	227	218	194	216	224	224	223	222	227	201	121	160	177	164	167	158	151	108	116	177	231	226	226	173	127	113	230	228	225	223	224	224	224	224	224	223	225	223	224	224	223	223	109	173	134	17	39	209	223	224	223	223	223	224	132	152	224	223	223	201	136	34	49	145	177	176	165	57	116	224	220	180	112	44	66	164	204	223	223	223	208	137	45	141	196	224	184	158	48	67	179	162	75	93	217	223	223	222	224	222	223	223	188	97	143	178	187	144	80	94	210	223	223	222	223	223	222	201	50	48	32	210	224	222	224	223	223	221	80	180	50	76	221	223	222	220	172	81	44	83	169	216	224	223	222	202	113	57	167	209	224	224	129	120	150	136	24	175	222	224	222	225	223	187	76	102	162	152	149	79	66	198	222	215	157	64	44	113	176	222	221	222	224	187	101	66	164	210	222	182	91	45	69	153	215	223	223	223	223	223	223	223	223	223	223	219	36	115	224	223	223	223	224	222	218	174	106	40	127	180	121	53	170	222	222	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	224	223	222	223	223	223	224	224	225	222	223	224	223	123	132	144	138	161	141	164	119	149	168	229	224	228	229	150	92	121	213	231	225	224	224	223	225	224	224	224	223	223	224	222	223	108	171	220	95	19	68	220	224	223	224	222	222	135	151	223	223	223	223	223	93	55	222	222	224	224	178	86	223	223	223	212	16	133	222	223	223	222	224	224	219	61	219	224	222	222	223	52	87	223	223	213	35	86	223	223	223	222	222	223	177	70	209	224	224	223	223	214	122	39	199	224	224	223	222	223	199	93	165	16	132	223	224	224	222	222	194	99	215	37	75	220	223	223	223	223	168	17	201	224	223	223	224	223	223	186	77	224	223	223	155	62	221	224	224	159	130	224	222	222	224	143	27	167	222	222	223	225	217	125	94	224	222	224	152	16	200	224	223	223	223	223	222	183	124	223	222	223	223	198	19	155	223	222	223	223	223	223	223	223	223	223	223	222	195	14	49	215	223	223	224	224	222	224	223	190	15	194	223	223	163	19	184	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	224	224	224	224	224	224	223	224	224	223	225	224	228	203	171	118	141	213	168	141	150	174	173	227	223	224	223	229	165	83	95	181	229	227	225	224	224	224	224	225	223	224	224	223	224	109	160	223	213	56	18	101	222	223	224	221	223	132	154	224	222	223	222	223	110	69	222	223	223	223	217	200	223	223	223	210	23	123	223	223	221	224	223	222	219	74	218	222	224	224	217	64	89	224	223	222	153	15	202	224	224	224	222	202	24	208	222	223	222	224	223	223	224	74	34	210	224	222	223	223	181	87	219	45	42	218	223	222	225	224	116	189	217	36	71	222	223	222	222	223	171	21	208	224	222	223	224	223	222	176	85	223	223	221	58	115	224	222	221	213	185	225	221	223	163	11	180	224	224	222	224	224	221	222	120	219	223	223	166	18	193	224	223	222	224	222	224	183	132	223	222	223	222	195	24	162	223	223	223	223	223	223	223	223	223	223	221	222	134	143	24	156	223	224	223	222	222	223	223	184	22	190	223	223	207	26	96	221	225	221	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	224	224	224	224	225	224	223	225	224	224	224	224	225	220	165	118	140	231	156	150	164	190	212	224	222	223	222	224	219	119	54	123	145	219	229	222	225	224	224	224	222	223	224	222	223	108	162	222	222	207	44	16	143	223	224	223	222	135	152	222	225	223	223	222	108	69	221	225	223	217	218	223	224	224	223	211	23	125	222	222	225	223	223	223	218	73	214	225	222	224	218	61	89	224	223	225	146	17	212	222	223	223	223	65	101	223	222	223	223	222	223	224	222	208	18	108	223	225	223	223	168	106	223	125	17	174	223	224	222	212	69	222	212	37	59	217	222	223	223	223	170	22	206	224	223	223	222	223	223	176	85	222	223	219	59	58	211	223	224	223	223	223	224	212	29	81	221	224	222	224	224	223	224	222	221	221	223	224	165	20	194	223	223	222	223	223	223	183	132	222	222	223	224	194	24	163	222	223	223	223	223	223	223	223	223	223	223	214	100	221	74	76	224	223	223	222	225	222	223	185	22	192	224	223	204	20	111	223	222	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	224	224	225	225	225	225	225	225	226	224	224	226	226	226	229	180	140	179	221	179	170	173	234	227	226	227	226	224	225	228	189	69	70	97	140	202	229	224	225	223	224	226	223	221	224	224	108	163	222	225	222	200	36	18	169	221	224	224	134	153	222	222	224	222	223	109	61	221	222	219	120	167	223	223	222	225	210	24	125	222	224	224	223	223	223	217	72	216	223	221	224	219	61	94	222	223	197	26	146	223	223	223	223	204	20	148	221	223	223	222	224	223	223	223	220	59	35	212	222	223	222	175	97	224	210	23	69	222	223	223	141	156	222	214	46	58	217	223	222	222	223	169	23	206	223	224	223	224	222	224	177	84	224	223	222	139	19	51	170	220	221	224	223	223	156	11	175	222	223	222	224	224	222	223	224	222	222	224	223	164	19	194	223	223	223	222	224	223	183	131	224	223	223	223	194	25	161	223	223	223	223	223	223	223	223	223	223	223	175	138	224	175	15	198	224	224	222	223	223	223	186	20	193	224	220	125	51	214	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	223	221	220	220	220	220	220	219	219	221	221	220	221	220	223	204	173	175	168	155	158	192	225	219	219	222	220	221	219	221	227	225	126	37	58	114	183	231	230	224	223	224	223	223	221	224	108	162	222	223	223	223	180	22	22	195	224	222	134	152	223	223	224	223	222	113	33	115	110	96	30	204	222	223	224	224	212	25	124	223	223	222	223	221	223	218	72	217	222	223	222	219	69	57	145	97	52	177	222	224	222	222	224	170	16	160	224	223	221	224	224	223	224	224	221	78	28	188	223	224	224	182	88	222	225	75	24	204	222	219	78	219	223	216	54	58	216	223	224	223	224	169	22	205	222	223	223	222	223	224	177	85	222	223	224	221	125	23	22	76	201	222	223	222	122	20	185	223	223	223	223	221	223	222	222	222	223	223	222	163	22	193	223	223	223	223	223	223	183	131	223	223	223	223	194	26	162	223	222	223	223	223	223	223	223	223	223	222	109	208	223	216	31	124	222	222	223	225	223	221	185	28	109	136	66	95	215	223	224	222	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
221	222	220	219	219	219	219	220	220	220	218	219	218	219	219	224	177	133	113	205	127	93	197	222	218	218	220	217	219	221	220	216	222	238	198	73	21	73	146	194	225	224	222	223	222	224	223	109	162	223	222	223	223	223	165	19	36	205	224	130	154	222	226	222	223	224	106	66	221	222	221	136	179	224	222	223	224	209	25	122	223	223	223	222	225	221	218	72	217	224	222	223	217	65	80	208	134	22	194	224	223	223	223	224	158	14	154	223	223	224	223	223	223	222	223	223	82	28	197	223	223	222	179	126	225	223	175	15	122	223	174	128	224	223	214	52	58	218	223	223	222	223	168	21	207	222	223	223	223	224	222	176	85	223	223	222	223	222	198	76	20	22	170	221	224	122	21	178	222	223	224	223	223	223	223	224	223	223	221	225	164	19	195	224	224	223	224	223	223	184	130	224	222	223	223	193	26	161	222	222	223	223	223	223	223	223	223	223	209	44	149	141	147	39	42	220	223	222	223	222	223	183	23	169	192	35	105	224	222	223	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	224	225	225	225	224	224	225	226	224	225	224	225	226	225	230	117	80	85	151	131	70	210	229	225	226	225	225	225	225	225	224	222	225	231	231	140	26	95	136	165	227	223	223	222	224	224	107	163	224	224	224	222	222	224	136	18	59	212	144	151	225	221	222	223	224	106	65	223	222	222	197	202	225	223	223	222	208	22	135	222	223	224	223	223	224	219	68	215	224	222	223	219	62	91	223	223	96	38	220	222	223	224	222	192	15	122	224	222	223	223	223	224	223	223	219	57	59	223	222	223	223	161	140	223	224	218	46	41	217	94	205	222	223	214	52	58	220	223	223	223	223	167	21	207	225	223	223	224	221	223	191	80	223	223	223	223	224	223	220	169	30	31	204	222	161	10	183	223	223	223	224	224	224	222	222	224	224	223	224	164	19	196	223	223	223	223	223	222	181	131	223	223	223	223	194	25	162	223	223	223	223	223	223	223	223	223	223	174	105	188	180	185	161	19	164	223	223	222	225	223	185	22	192	224	200	14	144	223	222	222	222	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	225	225	225	224	225	225	224	222	226	224	223	221	225	224	69	25	100	80	121	86	181	222	223	222	222	222	224	221	224	224	225	222	225	225	242	160	46	164	218	224	223	224	223	223	223	107	162	223	222	224	223	222	222	223	111	17	81	92	159	223	224	223	221	224	108	68	222	225	223	222	223	222	224	224	223	211	18	138	224	224	224	223	224	223	201	93	221	224	224	223	219	63	90	223	223	207	16	120	224	223	223	223	214	26	70	222	223	224	223	224	223	225	223	218	24	165	224	223	222	224	125	138	222	223	223	137	22	90	94	222	224	223	215	52	58	221	223	222	223	223	165	21	200	222	223	224	224	223	223	196	100	224	223	223	222	223	223	223	223	117	16	186	224	208	26	86	221	223	224	222	223	221	223	223	219	222	224	222	162	17	190	223	222	224	223	222	223	174	139	223	222	223	223	194	24	162	222	223	223	223	223	223	223	223	223	223	86	204	223	223	223	224	68	82	223	223	223	223	223	186	20	190	223	223	115	26	203	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	223	215	210	215	224	222	218	218	225	226	223	222	222	224	215	54	107	151	81	123	94	111	225	213	214	209	209	223	222	224	224	224	223	224	224	230	203	84	105	165	223	224	223	222	223	223	109	165	223	222	223	224	223	224	222	218	82	21	19	163	223	222	222	224	224	111	69	222	224	222	222	223	222	205	223	222	217	45	78	223	223	222	223	223	224	180	109	223	223	222	224	219	63	89	224	223	224	141	18	194	224	223	224	224	157	15	158	221	224	224	223	223	224	223	127	79	220	223	222	223	224	135	119	223	223	223	209	34	19	176	223	222	223	216	53	59	221	223	223	224	223	196	24	138	223	223	223	225	223	223	107	168	224	223	221	211	224	224	223	223	159	15	192	225	222	157	11	176	224	222	224	223	224	223	222	132	219	223	223	175	12	180	223	223	223	224	221	224	109	175	222	224	223	224	193	25	161	223	222	222	224	223	206	214	223	222	200	68	221	223	223	223	223	176	20	206	223	223	224	223	186	21	189	224	223	210	24	95	223	222	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	222	198	169	198	228	223	192	209	225	222	223	224	222	217	208	142	210	130	97	145	69	162	232	224	224	221	221	225	223	224	223	224	225	225	225	226	217	115	141	152	216	222	222	223	223	223	88	164	224	224	222	223	223	223	223	223	213	55	16	166	223	224	223	222	223	87	53	220	223	223	223	223	169	145	223	225	222	155	12	158	221	224	225	223	200	50	196	224	222	224	223	223	57	92	223	224	223	220	46	56	217	225	223	222	222	94	24	169	224	222	223	223	221	170	76	214	224	223	222	223	223	95	103	225	223	224	223	95	32	218	223	224	223	218	49	44	220	223	224	223	223	222	93	21	194	224	223	221	219	170	69	219	223	225	217	69	217	221	222	223	95	66	215	223	223	223	140	28	181	223	222	222	223	224	187	69	221	222	225	221	60	44	196	222	223	223	220	161	74	221	225	223	224	222	203	24	130	222	224	222	224	217	82	219	224	224	148	111	222	221	226	221	224	215	33	125	223	223	224	224	189	19	192	224	223	222	163	8	164	223	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	223	199	171	200	227	222	197	209	226	222	223	225	219	216	220	130	179	83	74	46	122	237	226	225	224	225	225	224	224	224	225	223	226	222	223	224	227	179	160	207	226	222	223	224	181	112	40	61	129	198	223	224	223	224	224	223	222	205	22	155	223	222	224	199	132	38	35	110	159	153	142	114	29	206	223	223	223	223	132	28	61	114	114	87	50	177	223	223	224	223	195	134	42	62	150	206	224	222	189	20	69	174	222	223	224	222	162	49	105	181	199	170	113	119	222	223	221	224	224	165	127	40	56	133	202	223	222	176	95	223	223	223	174	111	41	36	98	143	219	222	223	224	214	89	28	82	118	111	78	68	204	223	224	223	221	99	56	177	192	102	80	213	223	223	224	222	223	177	62	105	172	202	179	118	47	174	223	223	223	221	206	77	29	91	117	112	70	78	217	222	223	223	222	195	126	35	44	126	156	145	137	63	114	223	216	152	47	69	181	224	223	222	220	143	54	42	128	208	220	158	91	36	102	175	224	222	222	87	22	132	211	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	222	200	172	199	228	222	193	209	225	225	225	223	221	215	232	136	155	96	23	100	189	227	222	215	217	217	214	220	225	224	224	224	224	224	224	225	223	229	226	227	224	221	223	222	209	205	212	212	198	213	223	223	224	223	223	223	222	223	177	187	223	222	223	213	191	210	213	208	205	201	203	205	206	224	222	223	223	223	223	213	180	148	150	189	219	223	224	222	222	223	209	192	213	208	192	211	223	224	224	206	205	199	222	223	221	224	224	223	193	157	151	185	215	223	221	224	224	223	222	201	203	212	213	193	215	224	224	220	212	223	222	224	209	196	213	213	195	198	221	224	223	223	223	223	209	178	143	161	197	222	224	223	223	222	222	222	202	151	160	199	222	224	223	224	224	222	223	222	223	206	173	147	163	193	221	223	222	225	221	224	222	223	204	166	141	164	208	224	223	224	223	223	224	211	200	212	212	209	200	192	208	202	217	224	214	194	210	211	204	223	222	223	222	196	212	213	199	213	220	194	205	213	205	202	223	223	223	221	207	197	211	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	222	209	187	203	226	222	196	211	226	224	224	225	219	216	234	172	101	30	49	136	201	221	223	208	208	208	204	216	223	224	224	224	225	224	224	224	224	223	224	222	223	223	225	225	224	223	223	223	222	223	223	222	223	223	222	223	223	223	223	223	223	224	223	223	224	221	222	223	223	223	221	225	223	221	224	224	223	223	224	224	222	224	222	223	225	222	223	223	223	225	222	224	222	224	223	223	223	223	222	223	223	222	223	222	225	221	222	221	223	224	223	223	223	222	225	223	222	223	223	223	223	222	223	223	222	223	222	224	221	223	223	223	222	223	223	222	223	223	223	223	224	224	224	223	224	224	223	223	222	222	223	223	224	224	223	221	224	224	223	222	225	224	224	223	222	224	224	222	224	224	223	223	222	224	224	223	224	224	223	222	223	224	223	221	224	222	222	223	222	222	222	223	223	223	224	224	222	223	224	223	222	223	224	222	224	222	224	223	224	222	223	224	221	222	223	222	223	222	224	223	222	225	223	223	224	223	224	224	221	224	222	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	225	222	222	224	223	223	225	223	224	225	224	222	224	236	141	74	43	81	180	228	210	211	222	213	208	205	221	225	224	224	223	224	225	226	224	226	224	223	225	224	223	224	223	223	224	223	222	224	224	223	221	223	224	223	223	223	223	224	222	222	223	223	223	223	223	223	222	222	224	224	223	223	223	223	221	224	222	223	223	223	223	224	222	223	222	222	223	223	223	223	224	223	223	222	223	223	223	222	224	223	222	222	223	222	224	223	223	222	224	223	223	223	224	223	222	224	222	223	223	223	223	223	223	221	223	224	224	224	223	221	223	223	224	223	223	223	223	225	223	222	223	223	223	221	222	224	223	222	224	222	223	222	224	222	224	222	224	222	223	223	222	223	221	223	224	222	223	221	223	221	223	224	223	223	221	225	222	223	224	222	223	224	223	223	223	224	224	224	223	223	223	223	221	225	223	222	222	223	223	223	222	223	223	222	225	223	224	222	222	222	223	224	223	223	223	223	222	226	223	223	223	223	223	224	223	222	222	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	223	224	223	225	224	224	224	224	224	224	225	223	224	233	83	56	56	96	226	228	222	223	224	223	224	222	223	223	224	226	224	223	225	223	223	224	223	225	224	222	223	223	220	212	196	217	219	204	192	200	211	222	222	222	222	224	222	224	223	223	223	223	224	223	222	223	222	224	223	224	224	223	224	223	225	223	223	224	222	224	223	223	221	223	222	222	224	223	224	224	223	222	224	223	223	223	223	223	222	221	224	223	224	225	222	222	223	223	222	223	223	223	222	222	223	223	222	224	223	224	223	223	223	223	223	224	223	223	223	223	226	221	223	223	224	222	223	221	226	223	223	223	223	223	222	224	224	224	222	224	222	223	223	224	221	223	222	223	222	224	223	222	225	221	224	223	222	224	223	222	223	223	223	223	223	224	222	223	223	224	223	223	222	223	222	223	221	223	224	223	222	224	223	222	223	223	224	222	223	225	224	222	224	223	223	222	207	195	217	220	217	196	207	224	223	223	223	223	223	223	222	222	224	222	223	225	223	223	225	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	224	225	223	224	224	225	224	224	224	225	222	225	226	234	156	38	91	96	179	233	224	224	224	224	224	224	225	223	224	223	224	224	224	225	225	224	224	225	225	224	225	222	223	208	163	102	49	33	28	110	124	105	71	157	220	223	224	225	222	222	223	223	223	223	223	224	223	222	223	223	223	223	224	223	223	222	223	223	223	223	222	224	221	224	222	225	224	223	224	221	224	222	223	223	223	222	223	223	222	223	223	221	223	222	223	222	223	222	223	224	223	222	224	223	223	225	223	223	223	223	223	223	224	223	224	223	223	223	223	224	223	222	225	221	223	224	224	223	223	222	223	223	223	223	222	224	222	222	224	223	223	223	223	223	222	223	223	222	224	223	224	222	223	223	223	222	223	223	224	223	223	223	223	223	223	222	223	223	223	223	222	221	224	222	224	223	223	223	223	224	222	223	224	223	223	222	224	224	223	223	221	224	223	223	224	223	224	164	92	32	31	44	112	167	224	224	224	222	222	223	223	223	224	223	223	223	222	223	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	224	223	225	223	223	223	223	224	223	223	223	230	202	23	107	90	86	186	229	224	222	222	223	222	223	223	224	223	223	222	224	224	224	224	224	224	224	224	224	224	223	223	222	224	222	158	17	116	223	224	224	161	18	114	221	224	222	223	224	222	224	224	223	223	224	223	223	222	223	223	224	223	224	223	224	220	224	224	224	223	224	224	223	223	222	224	224	222	224	222	223	223	224	223	222	223	223	224	224	222	223	223	223	224	223	223	221	225	223	224	223	224	223	223	224	223	224	222	224	223	223	223	223	224	222	222	221	224	222	223	223	223	224	223	223	223	225	222	225	222	223	224	223	223	222	224	223	222	223	224	223	223	223	222	224	223	223	224	223	223	224	222	225	223	224	223	224	221	222	224	223	223	223	223	222	223	224	223	222	222	224	223	224	222	223	224	222	224	223	224	223	224	224	223	222	223	223	223	223	223	223	223	223	223	223	223	223	222	96	17	154	223	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	224	223	224	223	223	223	224	222	224	223	223	223	223	223	223	223	223
224	224	224	224	223	224	224	224	223	224	226	224	226	226	84	60	114	68	101	184	229	225	224	224	224	224	224	223	223	224	224	224	224	224	224	224	224	224	224	224	224	224	222	223	224	222	223	171	17	135	222	224	223	221	83	20	199	222	222	224	212	204	211	211	201	220	223	221	223	222	224	221	200	153	116	165	203	223	224	222	224	224	222	207	205	210	217	222	223	223	222	223	223	224	222	208	208	203	222	221	206	199	210	211	202	201	202	200	206	221	222	223	222	225	224	223	220	204	161	121	163	202	215	221	224	222	202	207	213	207	201	223	222	223	223	209	208	211	210	206	219	223	223	223	223	222	223	203	211	222	223	223	222	223	223	220	198	203	211	219	223	223	226	222	222	224	203	209	210	205	222	223	200	208	212	207	204	222	223	222	223	222	224	216	196	151	124	176	207	219	223	223	222	223	223	199	152	140	198	222	223	223	223	223	223	223	223	223	223	222	130	19	172	223	224	223	223	223	223	223	223	223	223	223	223	223	221	222	192	218	223	222	223	222	223	223	215	201	209	210	200	203	196	215	223	223	223	224
223	224	225	225	224	225	222	225	225	223	224	224	236	149	72	85	94	102	66	196	230	225	225	224	225	224	225	225	225	224	225	223	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	170	20	129	223	223	223	223	109	19	185	223	224	223	174	104	36	54	118	204	222	224	223	222	197	88	131	184	203	159	80	81	196	223	224	224	223	162	120	29	67	222	223	223	223	223	223	223	147	20	78	138	218	224	159	77	31	67	131	127	128	120	27	210	223	222	224	222	222	182	59	94	152	175	148	85	41	166	223	221	151	69	32	82	128	219	223	224	223	169	102	33	64	131	214	223	223	223	224	224	217	42	99	224	223	223	224	224	224	207	112	38	28	83	220	225	222	223	223	220	140	95	58	133	214	218	134	77	33	94	152	222	223	223	223	223	169	53	109	156	173	144	74	56	199	223	223	221	100	106	167	129	24	171	223	223	223	223	223	223	223	223	223	224	127	20	167	224	221	223	223	223	223	223	223	223	223	223	223	223	224	201	15	165	223	222	222	225	222	223	198	134	67	32	98	130	125	54	180	223	222	223
222	221	207	196	211	226	225	224	224	224	223	233	182	74	111	69	95	145	60	194	213	200	216	215	213	216	211	209	219	221	223	226	224	224	224	224	224	224	224	224	224	223	223	224	223	224	223	169	19	128	224	224	223	223	88	15	203	223	223	224	223	221	53	125	223	224	223	223	222	187	63	208	225	222	223	223	220	146	15	174	222	223	223	222	222	74	19	183	224	223	223	223	223	223	46	24	154	223	223	223	223	213	31	156	224	223	225	224	137	201	224	223	222	222	149	29	180	221	225	220	222	221	143	66	223	222	222	193	19	183	222	222	224	222	224	223	207	25	154	222	223	223	223	223	223	221	178	23	27	212	224	221	223	224	222	225	222	191	23	19	102	222	223	222	224	220	224	185	132	221	222	223	223	191	22	197	224	225	222	223	222	108	37	190	223	222	224	223	219	127	107	223	223	124	124	224	224	222	155	149	223	223	223	223	223	223	223	223	223	222	127	21	169	223	224	223	223	223	223	223	223	223	223	223	223	224	223	122	22	65	223	223	224	222	224	224	223	222	168	24	209	224	223	167	11	200	223	223
225	223	195	169	202	227	221	214	221	224	226	222	78	96	128	121	142	160	93	183	224	218	216	216	216	222	218	211	219	224	224	223	224	224	224	224	224	224	224	224	224	224	223	223	223	222	222	170	19	138	222	223	223	170	22	144	222	222	224	221	223	220	55	127	222	222	222	222	207	16	186	222	223	224	222	224	221	223	122	18	194	224	222	224	224	86	18	75	223	225	222	223	223	183	120	41	150	223	223	225	223	214	32	158	223	224	222	224	211	215	223	223	224	173	8	176	222	222	223	223	224	222	222	136	221	222	223	199	21	183	223	222	225	224	224	223	207	28	153	224	224	223	223	223	224	222	125	159	30	163	223	223	223	223	224	221	224	196	104	128	11	143	224	224	223	223	222	182	131	223	225	222	225	191	23	194	223	222	223	223	127	20	192	224	223	223	222	223	224	221	148	224	223	44	177	223	223	223	212	202	223	223	223	223	223	223	223	223	224	223	126	19	169	223	223	223	223	223	223	223	223	223	223	223	223	224	219	101	153	18	195	222	222	223	223	222	222	223	167	24	208	223	223	213	25	137	224	222
222	222	194	162	200	228	217	210	218	223	232	160	52	142	164	138	154	191	178	195	228	225	225	225	227	226	226	228	225	224	224	223	224	224	224	224	224	224	224	224	224	224	223	222	223	224	223	169	26	82	159	148	100	25	177	224	224	224	224	223	222	222	56	126	223	223	223	224	103	79	223	224	223	222	223	224	224	224	218	25	82	222	222	224	219	97	155	20	208	223	224	222	223	110	198	33	150	223	222	222	225	215	32	154	223	223	223	207	221	223	223	223	216	36	81	222	224	222	222	223	222	223	223	220	221	224	224	197	21	183	224	223	223	223	222	223	205	27	153	224	222	221	223	223	222	218	96	224	113	54	220	223	223	223	223	224	223	189	107	218	100	15	172	224	222	224	222	180	133	224	224	223	223	192	23	194	224	223	222	200	19	113	224	223	224	223	223	223	223	222	223	222	220	53	72	213	223	222	223	223	223	223	223	223	223	223	223	223	222	224	126	21	169	224	222	224	223	223	223	223	223	223	223	223	223	224	182	137	222	42	116	225	223	222	222	223	224	224	165	24	209	223	221	206	16	165	223	224
224	223	198	172	201	227	221	218	222	230	205	127	112	150	202	116	132	180	209	232	224	223	224	224	223	221	218	220	224	224	224	225	224	224	224	224	224	224	224	224	224	224	223	223	223	222	223	169	25	78	168	167	142	88	66	142	218	223	223	224	224	221	54	127	223	223	223	209	20	155	223	223	223	224	222	224	224	223	222	75	30	206	224	223	215	98	214	28	114	222	224	221	199	149	216	27	146	222	223	224	222	213	35	149	220	219	199	117	220	222	224	222	177	13	147	221	223	223	223	222	223	223	222	221	225	222	223	196	21	176	219	221	222	221	219	221	208	24	153	224	223	223	224	224	224	151	144	222	181	12	188	223	224	221	222	223	224	192	95	224	219	74	16	184	222	223	222	182	133	222	224	223	222	192	23	193	223	223	222	120	18	189	223	224	223	225	222	224	223	223	223	223	223	142	12	56	170	222	222	225	223	223	223	223	223	223	223	223	224	222	128	20	169	223	222	223	223	223	223	223	223	223	223	223	223	223	107	192	222	123	24	209	223	223	225	222	221	224	166	23	206	217	190	110	137	223	223	223
222	224	208	196	213	227	224	226	225	227	130	133	187	228	218	167	169	161	180	230	224	222	220	225	220	217	216	215	224	223	224	224	224	224	224	224	224	224	224	224	224	224	223	222	223	223	223	169	17	135	225	224	223	223	151	14	102	223	223	223	222	222	54	125	222	222	223	181	16	158	225	223	224	222	223	223	223	223	222	133	19	196	222	223	207	100	223	131	26	215	223	223	121	214	218	42	113	224	222	223	223	213	49	66	104	98	57	98	223	223	223	225	144	16	160	223	223	224	224	224	223	223	223	225	222	222	224	196	28	75	116	113	113	113	113	115	110	22	156	223	222	224	223	223	223	86	205	223	223	60	109	223	223	222	224	224	222	191	100	222	223	211	48	37	209	224	223	181	132	223	223	223	223	191	25	195	223	222	220	59	27	212	223	223	223	221	223	222	223	224	224	223	223	220	120	29	15	79	193	223	223	223	223	223	223	223	223	223	223	224	126	20	169	223	223	223	223	223	223	223	223	223	223	223	223	219	88	222	223	210	18	165	223	223	223	224	224	224	164	26	87	98	58	84	201	223	223	223
223	225	224	225	224	224	223	224	236	155	96	167	224	222	220	207	163	134	215	225	225	224	216	213	223	214	211	216	223	225	224	224	225	224	224	224	224	224	224	224	224	224	224	223	224	222	224	170	19	132	223	223	223	224	222	99	11	193	222	224	222	221	56	126	224	222	224	173	13	162	223	222	224	222	224	223	224	223	224	132	23	202	223	224	192	94	223	209	25	156	223	209	120	223	218	45	106	223	223	223	223	213	35	156	224	223	216	131	219	224	223	223	149	16	159	223	224	225	222	223	223	223	222	223	224	222	223	197	22	175	222	222	220	221	221	222	207	25	153	224	222	223	223	223	213	40	150	172	175	67	24	207	223	223	223	223	223	191	99	223	222	224	208	34	59	221	223	180	132	223	224	223	223	194	24	193	223	223	222	65	29	208	224	223	223	224	224	223	224	223	224	223	223	224	224	189	83	22	22	185	223	223	223	224	224	222	223	224	224	223	125	20	168	223	224	223	222	223	223	223	224	222	224	222	223	188	45	169	171	164	29	63	218	223	224	222	223	223	164	30	198	222	220	154	28	145	222	223
223	224	224	225	227	224	226	231	195	79	137	188	229	223	217	192	190	225	224	225	224	225	223	225	224	223	224	224	226	225	226	224	224	224	224	224	224	224	224	224	223	223	222	222	223	222	222	169	19	130	223	223	222	223	224	121	21	127	223	223	222	219	54	126	223	222	224	192	17	116	224	224	223	223	223	224	222	223	223	80	50	218	222	222	195	95	225	223	79	48	221	142	194	223	218	47	104	224	222	223	221	213	32	156	223	224	223	206	220	223	222	224	185	11	152	224	224	222	224	224	224	223	223	222	225	222	224	197	21	183	224	222	222	221	222	222	206	27	152	224	223	223	222	224	153	119	185	176	178	170	25	161	222	225	222	224	224	192	100	224	224	222	222	200	17	84	221	186	132	223	222	222	223	192	22	194	224	221	222	130	17	184	224	223	222	223	223	223	223	223	222	224	222	222	223	222	223	167	20	53	219	222	222	223	223	223	222	223	224	222	125	21	168	224	223	224	223	224	222	224	223	223	223	224	223	107	149	181	175	181	136	21	190	225	223	223	225	222	166	24	211	224	223	223	153	16	178	224
224	221	220	221	219	219	223	227	78	117	168	226	220	224	151	127	215	223	223	219	221	220	220	219	220	220	219	221	219	221	219	220	221	223	224	224	224	224	224	224	226	225	223	222	225	223	222	170	19	130	224	224	222	224	221	120	19	150	224	224	223	220	54	125	223	222	223	222	66	30	209	223	224	224	222	223	221	223	220	28	129	223	225	222	184	85	224	224	189	19	174	108	218	223	217	45	112	223	223	223	223	214	33	154	222	223	221	223	222	224	223	223	217	40	74	220	224	224	222	224	223	224	223	222	221	224	223	196	21	182	222	225	222	225	223	223	206	25	154	223	223	222	223	223	93	200	222	222	225	223	106	57	221	222	223	223	224	192	100	223	222	223	223	223	185	12	127	189	134	224	223	223	224	192	24	193	223	224	222	203	21	99	223	223	223	223	224	222	225	224	218	222	226	223	224	223	223	223	140	22	182	224	225	222	224	222	223	223	222	225	126	19	171	223	223	222	222	224	222	224	213	223	223	223	216	71	214	224	222	224	220	45	129	223	221	224	222	224	163	25	209	222	223	223	194	18	140	222
223	224	223	221	222	222	228	133	117	165	208	223	226	209	104	113	231	221	220	222	223	221	223	221	223	221	222	223	219	222	221	222	222	223	224	224	224	224	224	224	222	224	224	224	224	223	223	172	20	132	223	224	223	221	221	82	14	203	223	222	223	221	54	126	223	222	223	224	174	11	119	224	223	222	223	223	224	224	151	60	221	223	224	224	174	82	222	223	222	53	21	157	224	222	219	38	125	223	223	223	224	215	34	158	223	222	223	224	223	201	198	223	223	172	14	146	224	223	224	223	224	223	223	152	207	223	222	198	20	182	224	223	224	221	225	223	207	27	155	223	223	223	224	216	77	217	224	223	222	222	175	14	202	224	225	222	223	192	101	221	224	220	224	223	224	152	18	50	138	224	224	225	223	192	22	194	223	223	225	222	140	13	179	223	221	224	222	223	223	223	122	219	220	209	223	224	223	223	170	23	204	224	223	223	222	223	223	222	223	222	130	18	165	222	224	224	224	222	224	192	112	222	223	223	181	100	222	224	224	222	222	125	27	217	224	223	223	224	164	24	209	222	221	224	184	15	150	222
222	224	224	224	223	227	181	115	168	183	233	225	227	201	93	162	233	226	225	223	220	217	221	223	219	220	219	216	222	222	225	224	223	224	224	224	224	224	224	224	224	224	222	223	223	221	223	163	17	81	219	222	224	212	103	17	130	223	224	223	223	223	50	109	223	225	224	223	223	146	16	137	219	224	224	222	223	163	64	216	222	224	223	222	191	81	223	223	223	141	26	214	224	221	220	38	109	224	222	224	225	207	25	135	223	222	224	223	219	84	211	223	224	223	157	26	149	221	223	224	222	225	191	64	220	223	225	199	19	181	222	223	223	225	224	223	209	26	145	222	223	222	224	159	90	223	224	224	224	223	221	46	117	223	221	224	223	193	100	223	223	224	221	224	222	223	128	11	139	223	223	222	223	194	16	196	224	223	223	222	223	133	29	162	223	222	222	223	223	146	98	222	216	81	217	222	223	222	75	107	223	223	223	222	221	224	222	223	223	224	112	20	93	214	223	221	223	222	215	56	177	224	224	223	94	160	224	222	222	223	223	204	18	172	223	223	224	222	162	21	202	223	224	209	78	66	218	222
223	225	224	225	224	229	122	114	153	229	223	225	229	155	72	197	229	223	225	225	216	198	220	215	215	209	211	196	213	218	223	224	223	224	224	224	224	224	224	224	224	223	224	221	214	185	127	53	41	33	76	122	109	57	75	160	223	222	224	224	192	158	40	41	150	212	222	223	223	223	178	76	79	151	167	160	124	130	218	223	224	222	220	178	100	50	172	218	223	203	91	223	224	212	159	39	45	171	213	221	182	86	35	46	126	151	141	143	74	98	221	222	222	223	223	185	92	96	157	165	157	117	57	163	223	221	185	101	34	84	184	217	223	223	222	192	121	35	56	170	220	219	158	50	56	170	215	223	224	218	168	56	29	129	215	223	180	103	46	158	207	223	224	222	224	223	222	88	122	223	223	218	190	107	31	90	181	221	223	223	222	224	171	75	110	166	166	164	93	72	189	223	221	96	66	158	170	103	107	214	223	223	223	224	223	222	223	223	188	119	41	39	34	71	122	126	120	116	53	54	219	222	211	141	33	95	178	222	224	223	210	150	37	41	155	222	209	168	70	34	78	142	133	73	105	215	224	223
222	224	224	223	224	229	161	116	133	230	226	226	218	95	84	223	226	223	224	224	224	225	223	225	226	226	225	227	225	224	224	224	224	224	224	224	224	224	224	224	223	223	223	224	218	204	183	215	221	221	194	187	195	219	223	222	223	224	222	223	201	197	221	220	192	214	223	223	224	223	223	223	200	173	153	184	215	222	224	223	223	223	223	194	219	221	204	220	223	218	212	224	223	216	192	222	220	186	218	222	189	205	221	220	218	216	200	183	188	219	223	223	223	222	223	224	223	212	174	153	164	201	219	222	224	224	193	208	224	209	183	221	223	222	223	194	205	223	213	181	221	222	190	219	218	187	220	222	223	217	200	222	223	195	213	223	194	212	223	204	210	222	224	222	222	223	223	217	188	223	224	221	185	214	222	213	189	222	222	223	223	222	223	224	201	167	153	171	199	222	224	223	222	220	202	158	162	209	223	224	224	224	223	222	222	223	223	223	205	184	213	225	221	219	221	210	185	191	191	208	223	222	213	195	222	211	193	224	222	224	209	206	222	220	187	222	214	183	216	222	204	178	201	217	223	223	223	223
222	225	223	224	225	225	231	142	113	230	224	228	210	90	108	223	224	226	224	224	225	222	223	221	217	220	217	216	218	223	223	224	223	224	224	224	224	224	224	224	225	223	224	223	223	223	223	223	224	223	223	224	222	223	222	223	223	223	222	225	222	223	224	224	222	223	224	222	222	223	224	223	224	222	224	222	224	223	223	224	222	224	222	222	224	224	223	223	222	223	224	222	223	222	223	223	221	223	223	223	223	223	224	224	224	224	222	224	223	221	223	224	223	223	222	224	223	224	223	223	223	224	223	224	223	222	224	222	224	224	222	225	222	223	222	224	223	223	223	224	224	223	222	222	223	223	224	222	223	223	224	223	222	223	224	224	221	222	224	223	222	224	223	223	222	223	223	223	223	223	223	223	223	224	223	222	223	223	225	223	224	222	222	223	223	223	222	223	224	223	223	223	224	224	223	222	223	222	223	223	222	223	223	222	223	224	222	224	224	222	224	222	222	223	222	223	224	224	222	223	223	224	223	223	222	223	224	224	223	223	223	223	223	223	223	223	224	223	223	223	223	224	222	224	222	224	222	223
223	225	225	223	225	224	230	151	118	229	224	231	181	78	135	232	224	224	224	224	223	221	224	213	205	212	208	204	205	220	224	225	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	225	223	225	222	226	231	149	98	214	232	195	100	89	163	233	224	224	224	224	225	224	224	224	225	226	225	226	225	225	223	225	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	225	224	224	225	223	226	210	184	220	230	198	148	120	141	230	224	224	224	224	224	225	223	225	224	223	225	224	225	225	224	223	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	225	225	223	224	225	224	228	231	225	224	227	206	133	165	216	224	224	224	225	223	225	225	223	224	224	224	223	225	224	224	223	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	226	222	224	225	224	224	223	225	224	225	226	194	127	176	209	226	224	223	224	224	222	223	223	225	224	223	224	224	224	224	225	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	224	224	225	224	225	224	224	223	224	226	214	205	186	216	225	224	225	224	224	224	224	224	222	223	223	224	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	224	224	224	224	224	223	223	223	225	223	224	227	228	224	225	224	224	225	223	223	224	224	223	224	224	225	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	224	224	225	224	225	225	225	225	224	222	223	225	225	224	224	224	224	224	224	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	223	224	224	224	224	224	224	224	224	223	224	224	224	225	224	224	225	225	224	224	225	224	224	224	224	224	224	224	224	224	225	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	225	224	224	224	224	224	225	225	223	225	224	225	225	224	226	223	222	223	224	224	224	225	224	225	224	224	224	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	223	225	224	224	223	223	224	224	224	225	224	224	223	224	223	225	225	224	223	226	224	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	224	223	223	224	225	226	224	224	223	224	224	224	223	224	224	224	224	224	224	224	223	225	224	225	224	223	224	224	224	224	224	225	225	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
222	225	224	223	222	225	224	225	223	223	223	225	225	226	226	224	223	226	226	226	225	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
224	223	225	224	224	220	223	224	224	225	227	226	225	220	199	190	190	199	206	213	219	223	225	225	224	225	226	225	224	223	223	224	224	224	223	223	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
221	216	217	228	223	204	217	227	226	219	212	216	207	206	196	178	180	178	189	210	221	225	224	223	224	224	224	224	224	223	223	223	225	225	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	221	219	224	223	224	225	226	227	210	177	179	191	212	213	189	180	185	192	204	220	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	225	224	224	224	224	224	225	220	203	192	197	198	207	199	193	215	223	221	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	224	224	224	224	224	224	224	223	227	224	224	210	201	213	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
223	224	224	224	224	224	224	224	224	223	225	225	223	226	227	225	224	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223	223
];
logoBlue = [225	222	224	223	214	134	178	221	227	223	224	224	224	224	224	224	224	224	224	224	222	222	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
229	227	224	236	130	74	117	181	229	225	224	224	224	224	224	224	224	224	224	224	226	225	226	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
227	225	221	234	113	75	107	220	227	223	224	224	224	224	224	224	222	225	224	224	222	222	225	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	225	228	122	43	160	215	227	224	224	224	224	224	224	224	224	224	225	226	226	226	224	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
225	225	225	230	112	18	111	189	228	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
227	224	226	233	93	35	156	229	226	223	224	224	224	224	224	224	224	225	223	224	226	226	226	226	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	223	237	111	114	228	227	226	224	224	224	224	224	224	224	226	224	224	227	227	227	222	222	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
227	220	225	237	104	62	215	227	226	224	224	224	224	224	224	224	224	225	225	214	208	217	221	226	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
227	225	226	237	105	51	210	229	222	226	224	224	226	224	224	224	224	226	188	169	217	247	219	220	226	222	223	222	224	225	223	226	225	226	226	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	225	224	237	105	56	205	229	226	229	228	223	226	222	224	224	226	212	141	208	223	164	219	199	226	222	225	224	226	221	224	222	222	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
227	216	193	221	115	56	162	198	215	217	214	207	217	210	224	188	195	188	113	141	200	204	178	149	209	210	211	221	210	223	195	209	219	222	224	224	224	224	226	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
229	221	216	231	144	32	182	227	215	221	217	214	222	222	224	215	223	182	62	114	158	215	174	200	215	214	217	224	219	226	208	218	220	222	224	224	224	226	226	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
229	226	222	233	170	46	135	233	233	224	222	224	227	225	223	222	229	196	14	100	201	202	178	222	222	223	220	225	224	224	228	225	222	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	224	227	231	184	117	74	106	170	211	224	228	229	232	229	223	232	156	10	41	69	172	204	223	217	220	219	223	222	222	221	221	223	224	226	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	226	224	224	226	208	208	167	86	68	135	180	185	192	201	182	166	52	12	48	52	93	214	224	219	216	224	225	226	226	222	223	222	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	217	219	222	223	221	227	226	190	144	85	63	130	167	29	47	45	39	70	15	48	196	230	219	219	218	223	219	223	217	221	221	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	220	218	215	218	218	217	219	219	220	222	211	93	47	101	96	35	30	90	71	101	80	86	192	213	221	218	221	218	213	218	217	221	222	224	223	224	224	224	224	224	224	226	228	228	228	228	227	229	228	226	228	229	226	226	229	225	227	228	228	226	229	225	226	228	228	228	228	228	228	228	226	228	228	225	228	228	225	228	228	229	226	225	228	228	228	228	227	226	228	227	229	227	229	228	226	229	228	227	226	228	226	228	228	226	226	228	226	225	229	227	228	228	228	228	227	228	226	228	228	227	226	228	225	226	227	226	228	227	229	225	229	227	229	228	225	226	226	227	228	228	226	228	228	228	226	228	229	226	226	229	228	226	228	230	227	228	227	228	226	227	228	226	229	228	228	228	228	229	228	228	226	228	228	228	223	226	229	228	228	226	227	226	226	228	227	228	225	228	229	226	229	225	229	228	227	228	228	226	227	228	228	228	228	228	228	228	228	229	225	226	228	227	229	228	229	227	228	227	227	227	227	226	224	226	227	228	227	228	223	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	228	227	223	224	222	225	226	223	223	238	169	1	92	116	62	68	110	135	123	98	49	82	104	201	223	225	225	224	228	224	222	225	224	223	224	224	224	224	224	228	229	224	229	228	228	226	223	228	227	227	228	228	225	226	228	227	228	228	228	229	228	227	223	229	226	226	228	228	229	227	229	226	228	228	227	228	227	226	225	228	228	223	225	228	228	228	227	226	228	227	228	225	228	228	224	228	228	225	228	227	226	228	231	225	230	225	228	228	228	228	228	226	226	228	225	227	228	226	228	228	226	228	226	228	229	228	228	225	226	225	226	228	227	226	228	228	228	228	229	225	228	228	228	229	228	228	226	225	230	225	228	226	226	224	228	226	226	228	228	229	229	225	228	225	224	226	226	228	226	226	229	226	229	228	228	227	228	226	228	226	225	228	228	228	227	228	228	227	227	226	226	227	226	226	228	228	228	231	228	228	228	228	228	228	228	228	227	228	225	229	228	225	228	227	226	226	227	226	228	225	228	231	228	229	227	229	227	228	228	229	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
225	227	227	226	225	226	223	227	222	226	225	230	204	30	61	63	106	139	189	141	143	154	64	30	22	48	130	221	227	227	221	225	224	224	224	224	224	224	224	224	220	222	228	228	228	228	229	227	228	227	228	224	229	226	228	228	227	226	229	225	227	227	227	228	229	228	229	226	225	228	228	228	224	227	226	226	225	228	228	225	228	226	226	227	228	228	227	226	229	227	226	228	224	228	227	228	226	227	229	229	225	226	228	228	228	226	229	229	227	229	227	226	227	226	227	228	225	226	227	228	228	224	228	226	227	229	228	225	227	227	230	228	229	227	229	229	226	228	226	226	225	227	228	228	229	225	227	229	229	227	228	228	228	227	229	228	229	226	227	228	225	225	225	226	228	228	226	227	228	229	226	225	229	228	228	228	228	228	228	228	226	227	228	227	226	228	229	228	228	228	226	230	226	229	228	226	227	228	224	228	228	228	228	228	228	228	228	228	228	225	226	233	225	229	227	223	226	228	229	226	231	227	228	228	229	224	228	228	229	228	226	227	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
229	222	209	199	211	226	222	209	219	224	221	225	230	89	22	60	104	148	198	172	150	195	135	71	34	10	70	157	232	224	221	224	223	224	224	224	224	224	224	224	226	226	228	225	213	203	201	209	210	226	226	225	228	228	228	227	228	211	203	213	211	200	213	228	227	228	229	231	226	224	228	223	228	227	227	225	228	229	227	228	228	228	228	228	225	228	229	226	226	228	230	228	226	225	228	227	229	228	227	226	229	226	225	228	225	228	228	228	227	228	227	228	229	227	228	226	229	225	226	228	213	205	211	211	215	228	228	224	228	224	227	228	228	226	224	207	209	206	209	227	229	227	230	228	226	228	226	229	229	226	229	228	229	229	228	225	230	223	225	228	227	226	229	228	228	225	228	229	226	228	228	230	227	225	224	225	230	227	225	229	226	229	227	226	231	226	225	225	228	228	225	228	228	226	227	229	229	227	228	225	228	228	228	228	228	228	228	228	228	228	228	225	229	226	227	228	225	228	228	225	226	226	228	228	226	229	226	226	225	228	229	225	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	217	180	147	190	228	210	171	207	229	224	214	224	163	35	42	114	158	183	179	146	201	166	97	68	78	72	35	149	229	220	224	224	224	224	224	224	224	224	224	219	225	226	227	209	186	162	166	167	199	225	228	228	225	228	226	228	203	185	170	168	190	202	226	228	228	225	226	228	225	226	227	228	228	228	229	225	227	229	223	228	229	228	226	228	228	226	229	228	228	227	228	228	228	224	229	229	228	226	226	228	228	226	226	231	226	227	228	228	222	228	228	226	226	228	228	226	228	228	226	201	188	171	165	172	226	227	228	227	227	228	228	229	228	197	161	173	186	198	225	228	226	228	224	228	229	227	226	226	228	227	226	224	226	228	224	227	229	228	228	227	226	225	223	226	229	228	225	228	228	228	227	228	229	227	229	226	228	230	225	229	226	229	226	227	228	228	228	228	228	230	227	228	229	225	226	227	229	228	226	228	228	228	228	228	228	228	228	225	225	230	227	225	228	228	228	229	229	226	229	228	227	228	228	226	229	229	228	229	228	229	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	222	183	146	193	232	211	169	209	229	223	215	221	210	90	79	103	111	128	199	161	135	154	96	48	38	92	109	60	188	232	226	224	224	224	224	224	224	224	224	225	223	228	229	228	228	218	171	161	163	201	229	225	226	229	228	227	226	229	190	189	227	228	228	226	229	228	228	230	227	228	228	228	228	226	227	229	225	226	229	228	228	227	227	227	229	225	226	228	225	229	231	226	227	229	225	226	228	225	227	227	227	226	228	229	228	228	230	228	229	229	229	224	228	228	230	225	227	228	227	231	228	207	162	162	211	227	226	229	227	228	226	226	225	168	165	174	224	228	226	225	227	228	225	226	227	227	228	228	229	228	226	227	228	225	228	228	229	228	224	228	227	226	228	228	227	227	226	228	223	229	228	228	225	228	228	228	226	229	229	226	228	231	226	224	225	231	227	228	230	226	228	226	225	227	227	228	228	226	229	228	228	228	228	228	228	228	228	229	227	228	228	229	228	228	228	228	226	228	228	227	229	226	228	228	228	227	228	228	226	228	225	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	222	181	149	195	230	209	170	206	226	222	213	220	234	168	66	102	150	167	159	126	76	90	123	84	148	231	165	158	103	205	232	227	224	224	224	224	224	224	224	223	224	229	228	226	225	228	189	168	163	165	208	229	229	225	229	226	227	227	196	204	227	226	226	223	217	222	225	223	214	217	218	220	225	228	231	219	221	224	225	215	224	226	228	229	225	223	227	222	219	227	218	217	226	224	221	221	222	229	228	227	225	229	228	225	227	225	228	227	220	200	197	210	223	228	227	231	226	227	228	228	229	220	167	163	181	226	226	226	226	229	225	224	210	167	165	179	228	223	225	228	217	223	226	226	217	225	225	228	228	224	224	226	220	223	229	228	227	221	205	198	220	227	227	228	229	225	229	227	228	221	202	195	211	221	228	229	223	225	218	224	226	222	220	228	228	228	226	223	225	221	220	225	228	217	221	224	226	216	223	228	228	228	228	228	228	228	228	228	227	228	218	219	229	225	227	228	227	228	225	219	223	227	224	216	222	226	227	225	229	227	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	223	190	166	200	229	210	176	210	229	226	214	217	224	200	97	126	140	130	143	131	124	77	87	157	233	224	228	167	102	97	231	229	227	223	224	224	224	224	224	226	222	228	226	229	227	229	185	205	194	162	164	218	226	228	228	228	228	226	195	201	229	228	228	219	195	168	166	199	212	209	203	172	184	229	226	208	188	170	173	204	219	223	228	226	216	196	167	193	220	228	211	204	172	174	208	201	174	183	223	229	225	228	228	227	227	228	213	181	198	206	209	200	180	179	217	231	225	228	227	228	223	216	167	171	164	218	229	227	226	225	229	227	179	208	170	174	226	229	228	223	207	180	168	178	205	227	228	228	226	218	186	169	208	221	226	228	191	189	202	191	164	207	226	228	228	228	228	212	176	184	205	200	197	176	174	216	231	224	203	173	166	190	206	224	228	227	229	210	185	178	202	221	227	212	180	172	173	202	222	228	228	228	228	228	228	228	228	228	225	226	163	189	228	228	225	229	226	227	225	206	187	169	192	211	188	168	206	226	228	226	227	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	224	222	221	223	224	221	224	222	224	220	215	217	222	220	107	113	110	104	132	120	144	97	124	148	223	221	224	231	142	70	107	212	233	224	222	226	224	227	224	224	224	225	228	226	227	228	186	205	226	184	160	173	226	229	228	228	228	225	194	203	226	228	228	228	228	181	171	227	227	226	226	207	180	225	228	226	220	161	193	228	226	229	227	226	226	221	175	224	229	227	225	228	169	180	226	228	221	167	182	225	228	228	226	227	229	209	174	223	223	229	228	226	226	189	169	214	226	226	229	227	229	213	181	203	161	192	226	226	229	227	227	213	185	223	163	175	225	228	228	228	228	204	161	219	226	228	229	228	228	228	215	176	229	225	223	202	168	227	226	226	205	192	227	226	228	226	195	163	201	227	229	225	224	226	191	180	228	227	226	199	164	215	228	228	231	225	228	227	208	188	228	227	228	228	219	157	199	228	227	228	228	228	228	228	228	228	228	228	225	212	160	172	220	225	228	228	226	227	226	225	211	162	215	225	228	203	162	210	226	225	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	224	224	224	224	224	226	227	226	226	224	224	224	229	197	170	98	105	187	145	122	123	149	156	228	224	219	225	229	151	57	78	177	233	230	223	228	219	224	224	223	229	226	223	229	226	185	202	224	222	169	164	186	227	228	226	228	228	193	204	226	227	228	227	228	187	172	227	229	228	229	222	218	228	226	228	224	166	185	228	226	226	230	228	228	227	174	224	228	229	226	224	169	183	226	226	227	202	158	216	229	228	228	227	216	160	217	227	228	227	226	228	225	227	176	163	220	228	225	228	228	207	181	224	168	168	223	228	225	230	228	186	211	225	166	176	228	228	227	227	228	207	162	221	226	227	229	226	228	227	211	178	228	228	226	172	187	229	227	228	223	209	227	228	228	205	161	208	230	226	227	226	228	225	228	191	224	229	228	203	163	217	226	228	228	226	227	229	210	194	228	227	228	227	215	162	206	228	228	228	228	228	228	228	228	228	228	225	229	194	198	163	202	230	226	228	227	227	231	226	212	165	209	228	228	215	165	183	228	229	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	224	222	224	224	224	225	226	223	222	224	225	223	222	227	221	156	107	102	205	128	125	141	175	209	225	218	221	223	226	211	97	37	107	134	216	227	223	225	224	225	222	227	232	229	227	228	186	202	227	228	219	163	161	198	229	226	226	225	194	204	228	227	226	229	228	184	174	226	227	226	225	223	226	226	227	229	221	165	189	227	228	227	228	228	228	224	173	221	227	227	229	227	171	182	228	228	227	198	163	220	227	228	227	227	172	185	227	229	228	226	225	229	226	228	222	160	185	229	227	225	228	202	184	228	193	159	209	228	226	225	218	178	227	220	165	169	225	227	228	228	228	206	160	218	226	228	228	227	226	228	214	179	229	228	228	174	172	218	228	226	228	228	225	228	220	165	181	226	226	227	230	226	225	228	227	224	226	229	226	202	163	212	228	228	227	228	225	226	212	192	227	227	228	229	214	162	206	227	228	228	228	228	228	228	228	228	228	228	224	183	226	173	175	223	227	226	227	229	227	224	212	164	214	226	228	217	164	188	226	225	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
229	226	224	225	225	225	225	225	227	225	226	224	226	225	225	232	174	119	146	197	165	161	166	232	223	219	221	224	222	225	228	186	64	55	83	130	199	232	225	224	224	229	222	225	228	228	226	185	200	229	228	229	215	165	161	205	230	226	226	195	206	228	223	229	223	228	183	172	226	228	223	185	202	228	228	228	227	222	162	190	227	230	226	225	228	225	222	178	221	226	228	226	223	172	188	226	226	214	165	198	225	228	228	228	218	161	198	228	225	228	228	226	228	226	225	225	172	167	222	227	228	231	207	178	226	218	164	176	227	229	228	200	203	227	225	166	171	225	228	227	227	228	205	161	222	228	226	228	229	227	228	213	180	226	228	223	197	160	171	205	228	226	226	228	228	202	160	207	227	225	229	226	226	229	228	228	227	227	224	228	206	162	216	225	228	228	227	226	226	212	191	229	228	228	228	214	162	204	228	228	228	228	228	228	228	228	228	228	228	208	192	229	207	160	216	226	226	229	223	228	229	213	161	211	228	225	192	169	219	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	224	218	217	217	217	217	217	219	216	220	217	217	220	217	223	200	131	134	145	132	149	192	220	210	212	216	215	217	218	219	225	221	120	24	48	96	174	231	230	226	223	226	228	230	226	227	188	200	228	225	225	229	209	165	163	214	225	229	191	204	229	225	228	229	227	190	166	188	188	182	166	216	227	228	226	226	221	161	192	228	228	227	228	226	229	224	173	225	227	229	226	226	174	172	194	186	171	208	227	226	229	227	226	208	161	198	226	228	228	226	226	228	226	228	226	176	164	217	228	228	226	209	177	228	225	180	160	218	227	226	178	222	228	226	168	173	222	225	226	228	229	206	160	221	227	228	228	227	228	229	210	181	227	228	226	226	190	164	163	173	213	228	223	228	193	159	210	228	228	225	228	228	229	227	228	225	228	228	228	205	160	214	228	225	228	225	228	228	210	193	225	228	228	227	213	163	205	228	227	228	228	228	228	228	228	228	228	226	183	222	227	224	163	193	228	227	228	228	228	227	210	164	186	190	173	183	226	228	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
225	221	217	216	216	215	215	216	216	214	214	216	215	215	213	223	166	95	85	164	89	59	181	218	215	218	215	215	215	216	215	214	219	236	196	66	9	52	135	196	225	227	226	228	227	228	228	185	204	228	231	228	228	227	205	158	163	220	229	191	203	227	225	227	225	226	183	175	226	226	226	195	209	229	227	229	226	222	162	190	228	228	228	227	227	226	224	178	222	226	229	228	224	173	176	222	193	160	213	229	229	226	228	226	200	163	196	228	228	226	228	228	228	227	228	225	176	164	216	228	224	227	210	196	229	228	208	163	194	227	204	193	226	228	222	169	173	224	225	228	227	228	206	162	219	227	228	228	228	229	227	210	181	224	228	229	228	227	216	174	163	163	202	228	227	192	164	210	227	229	228	226	228	226	228	224	228	229	228	227	203	158	216	226	226	228	226	228	228	211	192	226	227	228	228	213	163	204	227	227	228	228	228	228	228	228	228	228	220	168	198	195	192	166	166	224	229	225	228	228	228	211	161	208	213	166	183	229	227	229	228	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
229	226	225	225	225	224	224	225	225	224	224	227	223	225	224	231	100	53	55	115	94	45	167	216	222	227	225	227	227	223	224	224	225	227	233	234	133	14	74	128	151	226	229	226	227	227	227	184	201	226	224	229	228	226	226	192	164	172	219	195	200	230	230	229	228	226	187	177	222	226	229	216	214	229	225	229	223	222	162	195	227	228	226	229	225	226	227	172	226	226	223	226	222	173	180	226	228	186	166	224	228	228	229	227	211	160	191	224	229	225	229	228	224	226	229	224	173	169	228	226	228	228	204	201	224	227	224	168	166	225	183	218	227	228	225	169	171	225	225	229	228	226	205	162	219	227	229	228	226	228	228	218	180	228	224	225	228	226	228	228	207	163	166	218	227	203	160	209	228	226	228	226	226	229	227	229	228	226	225	226	207	159	215	228	228	228	228	228	227	210	191	228	228	228	228	214	162	205	228	228	228	228	228	228	228	228	228	228	207	183	212	211	214	206	160	203	228	226	228	225	225	210	163	211	224	216	162	192	228	227	229	227	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	225	225	225	224	224	227	226	226	223	224	219	225	225	222	54	10	73	59	85	50	140	216	226	222	217	220	224	221	224	226	222	225	223	225	243	157	29	149	215	226	225	228	228	225	228	185	203	225	227	229	226	227	227	231	186	159	180	179	205	228	226	228	225	229	185	171	227	226	228	225	226	229	226	226	229	222	159	199	226	226	226	231	226	229	215	180	226	226	226	229	227	170	182	228	225	221	159	192	228	224	228	228	221	163	177	226	228	226	225	228	231	227	225	224	161	207	224	231	228	226	190	200	227	228	226	194	163	182	181	227	229	226	223	169	173	226	228	228	228	226	203	162	215	227	228	226	226	229	226	215	182	228	228	228	227	228	226	228	226	189	161	208	228	219	164	177	226	225	230	227	226	228	228	225	225	228	228	227	204	162	212	228	227	226	229	227	228	205	197	228	227	228	228	214	162	205	227	228	228	228	228	228	228	228	228	228	178	218	228	229	225	228	173	180	228	228	228	228	228	210	161	211	231	225	190	164	217	227	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	210	198	207	222	220	215	218	220	225	228	217	221	218	209	38	84	122	59	89	67	92	215	203	204	200	200	216	218	222	224	224	224	222	224	233	202	58	84	161	223	228	225	227	228	229	184	203	228	229	226	226	228	228	229	223	181	162	160	208	227	227	228	228	226	187	172	230	226	226	227	228	227	218	228	227	225	168	177	228	229	225	228	228	226	209	184	228	228	227	226	224	171	183	226	228	226	196	159	215	226	227	229	228	205	163	201	228	226	226	228	228	226	228	193	178	221	231	227	228	226	194	193	228	228	228	219	164	163	208	229	227	226	225	169	175	226	228	228	229	229	213	163	199	228	225	228	224	229	227	184	206	229	228	228	220	226	228	226	228	198	163	211	226	229	199	161	205	230	225	226	229	226	228	227	193	224	228	228	210	156	209	228	228	228	228	225	226	186	206	227	226	228	228	211	167	205	224	227	228	226	228	221	220	228	229	216	176	221	228	231	228	226	208	161	220	228	229	226	227	209	164	213	226	227	218	165	181	227	225	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	223	186	145	185	230	218	171	195	229	219	224	222	216	207	207	122	187	106	77	113	51	158	231	224	224	222	220	221	221	224	223	224	224	224	223	223	212	109	136	139	214	228	229	228	228	227	179	202	226	226	228	228	228	228	226	228	222	170	163	202	228	226	228	229	226	179	170	225	229	227	228	228	207	200	228	225	227	196	160	202	223	227	226	228	215	171	215	226	228	226	226	228	170	183	228	226	228	228	172	170	222	227	226	227	227	179	162	207	226	227	228	229	226	205	176	223	229	228	225	228	228	182	186	229	226	228	228	185	162	223	228	226	225	225	172	170	224	227	226	229	225	228	179	164	213	229	226	228	228	204	173	221	227	229	222	173	225	228	227	228	182	173	226	228	228	228	196	164	207	228	227	229	228	226	214	172	227	228	227	227	172	167	217	229	227	229	225	203	176	224	228	228	229	228	218	160	191	227	226	227	226	223	176	225	226	226	199	188	229	226	228	225	228	225	165	190	228	225	226	229	209	160	211	226	228	229	205	158	206	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	222	185	148	189	229	217	178	197	229	219	220	224	215	208	218	111	151	68	52	29	114	238	222	227	226	227	225	222	224	224	225	226	223	223	224	226	228	173	154	206	225	227	228	223	208	185	166	172	194	216	228	226	228	226	224	225	225	219	162	206	229	226	226	216	194	167	166	187	201	196	190	186	161	218	229	228	228	228	194	164	171	190	189	181	170	207	227	227	226	228	216	196	166	172	199	221	226	225	213	164	173	206	228	228	226	222	201	171	186	210	212	206	189	190	224	228	228	226	228	202	195	164	171	192	218	229	229	207	181	223	226	228	207	188	166	165	184	195	225	227	225	229	225	177	167	178	191	188	176	175	219	228	228	229	226	183	171	207	215	180	177	223	228	224	228	227	224	210	175	184	208	217	207	187	168	208	228	228	229	227	218	177	164	180	190	189	176	176	223	229	226	229	227	210	191	167	167	187	200	194	191	171	184	229	224	197	169	177	210	226	225	229	225	199	169	167	192	222	223	200	184	168	183	209	226	228	226	177	162	194	221	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	221	185	149	188	232	218	175	197	229	224	222	223	215	204	231	130	139	77	3	84	177	226	221	209	211	211	208	216	225	226	226	223	222	227	222	220	223	228	225	230	224	227	223	229	224	217	223	220	216	221	224	229	229	228	228	229	227	228	206	214	227	229	224	222	214	222	219	220	218	217	219	221	221	228	227	225	229	226	229	221	209	201	199	210	225	228	228	228	227	228	221	213	221	221	215	221	228	224	228	218	219	217	222	228	226	229	223	224	211	204	199	206	221	228	228	229	226	228	227	216	215	220	221	214	223	226	226	225	224	229	230	225	221	215	221	221	212	218	223	226	229	225	229	228	219	206	198	203	214	227	229	226	228	227	226	230	214	202	202	214	231	228	226	223	229	227	228	228	225	217	205	194	204	215	224	228	227	224	225	226	225	227	221	206	192	201	219	226	227	228	228	226	229	223	217	223	224	221	216	216	223	217	226	226	224	213	221	219	215	229	227	225	226	215	223	222	212	223	225	217	219	218	216	218	227	227	226	223	219	215	219	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	223	197	168	194	226	218	181	202	225	222	226	227	214	210	231	165	77	13	29	127	197	210	220	198	195	193	191	208	221	224	225	226	224	226	224	226	224	224	222	223	223	225	230	228	226	225	228	228	227	229	229	227	228	228	227	228	228	228	227	225	228	226	229	226	227	228	227	227	228	229	228	226	228	228	229	226	225	228	228	224	227	226	227	228	228	227	225	226	229	227	225	228	230	224	225	224	228	228	223	228	231	227	228	228	230	227	227	228	228	230	225	228	228	227	228	229	225	228	228	229	228	228	224	229	225	228	227	226	228	226	228	228	227	225	228	227	228	227	228	228	228	224	226	225	226	228	228	230	227	225	228	228	229	226	229	226	229	228	226	227	229	228	226	231	227	228	226	227	228	226	231	228	228	229	226	228	226	226	228	228	227	229	228	227	229	229	227	229	229	225	227	228	228	228	226	227	227	226	228	228	229	225	226	227	229	227	226	227	226	227	228	229	228	228	228	223	228	227	228	226	227	229	229	226	227	228	229	227	228	230	227	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	221	218	219	222	222	221	224	222	222	224	224	222	224	236	130	44	23	67	173	224	198	199	216	204	196	196	215	223	224	222	221	222	224	225	226	225	224	228	224	222	228	226	228	229	228	226	228	226	226	225	227	228	226	228	226	228	229	226	225	227	228	228	228	228	227	228	227	227	226	226	228	228	228	226	229	229	228	228	231	228	228	226	227	228	226	227	225	228	226	229	228	228	228	228	226	229	228	228	226	225	227	229	225	227	228	226	228	227	226	228	228	227	228	229	227	227	225	231	228	228	228	225	228	228	231	228	226	223	228	228	223	228	226	228	226	225	229	227	225	229	226	228	228	228	227	229	226	225	229	227	228	229	224	225	229	225	226	228	228	228	225	228	228	228	228	225	228	227	228	228	228	224	231	228	228	224	231	229	226	227	228	226	228	225	226	228	224	228	226	229	228	224	228	227	229	227	227	228	226	227	229	228	226	225	227	228	228	227	229	226	228	226	226	227	228	227	229	225	228	228	225	228	224	228	228	227	227	227	225	229	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	223	225	225	227	223	228	226	226	226	223	221	224	226	236	69	34	35	83	225	224	223	224	226	223	222	220	224	225	226	225	224	226	224	223	224	224	221	227	224	223	228	228	227	219	216	225	225	217	215	219	222	228	227	225	229	228	231	230	228	231	225	228	228	228	231	225	228	229	229	226	226	229	226	225	231	225	226	226	227	226	229	229	228	228	226	228	226	231	226	229	226	227	226	228	225	229	226	230	227	227	228	228	226	227	227	227	226	224	227	228	228	227	227	226	223	227	229	224	229	226	228	226	228	229	226	228	229	226	229	223	229	228	228	227	230	227	228	226	228	227	231	228	225	228	228	226	229	227	228	226	228	226	229	226	226	229	227	225	227	226	228	228	227	226	226	228	225	226	231	227	228	225	228	224	226	226	227	225	225	222	229	226	227	228	227	228	228	226	226	228	229	224	228	227	228	228	226	227	225	229	226	231	229	228	228	228	218	215	225	225	223	215	219	228	229	228	228	228	226	225	225	228	228	228	225	227	225	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	224	223	222	225	222	221	222	225	223	223	226	221	221	237	153	25	68	69	173	235	220	224	224	222	224	224	223	222	223	222	223	222	224	224	225	224	226	225	224	226	224	227	228	217	203	186	169	162	164	187	190	182	176	200	225	228	224	226	225	229	226	228	225	225	228	223	226	227	226	228	228	228	226	228	228	228	228	228	228	228	226	228	228	226	227	230	226	228	226	228	226	226	230	225	228	229	228	228	227	228	226	228	228	227	225	229	228	227	228	226	228	228	226	228	228	228	228	225	228	226	228	229	228	228	228	224	229	226	229	226	228	225	229	228	228	226	229	228	228	227	228	228	225	228	227	226	229	227	228	228	228	226	228	229	227	229	226	227	229	228	226	227	228	227	226	229	228	228	226	227	228	226	228	228	228	227	229	224	228	228	229	228	226	227	228	228	228	229	227	226	228	228	226	228	228	227	226	226	228	228	227	229	225	228	224	228	228	203	180	165	165	168	189	206	226	226	226	225	229	227	228	224	229	228	228	225	227	228	226	228	225	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	223	219	217	219	217	217	220	217	218	217	221	216	225	196	16	95	63	68	179	227	217	219	222	217	219	220	217	218	217	219	219	222	222	224	224	226	224	224	224	224	224	228	228	227	229	228	196	162	191	228	226	226	206	159	184	226	229	227	228	223	229	226	226	228	226	226	228	225	227	228	226	228	228	229	226	226	230	228	226	226	228	226	228	228	228	227	226	226	227	226	227	229	228	226	228	226	227	228	226	229	229	228	225	228	228	228	229	228	229	228	226	228	226	225	229	229	228	228	228	226	228	228	226	228	226	227	225	227	228	228	228	223	229	226	228	226	228	229	230	227	227	226	229	228	228	227	228	228	227	226	226	229	228	230	227	226	228	228	224	228	228	229	227	227	228	228	226	228	225	229	228	228	228	225	226	229	228	228	227	227	229	226	228	228	228	228	228	228	226	229	228	228	226	224	231	226	228	228	225	228	228	228	228	228	228	228	228	228	223	184	165	199	228	226	229	228	228	228	228	228	228	228	228	228	228	229	228	226	228	227	228	225	228	225	229	225	226	227	228	228	228	228	225	228	228	228	229
226	222	224	226	225	226	222	226	228	226	225	223	228	225	79	52	89	46	78	181	232	222	224	222	226	222	222	226	226	224	224	224	222	224	224	224	224	224	224	224	224	224	227	228	229	227	229	207	162	197	225	224	229	227	182	166	218	226	227	229	220	219	222	223	216	225	229	227	228	229	226	227	217	203	196	206	217	227	228	228	229	228	226	221	217	218	225	227	228	228	227	228	228	228	225	221	221	217	228	228	216	216	218	223	216	214	217	213	219	226	229	228	227	227	226	228	225	217	199	193	205	221	223	228	226	228	216	219	220	217	216	227	229	228	228	221	220	223	220	217	224	228	226	226	228	227	228	216	219	229	229	229	227	228	228	225	216	219	221	227	228	225	225	227	227	226	217	221	218	218	224	225	213	219	222	220	218	228	225	229	228	225	228	224	213	199	197	206	219	225	228	226	227	228	227	214	202	198	214	227	228	228	228	228	228	228	228	228	228	229	194	163	201	229	226	229	228	228	228	228	228	228	228	228	227	224	227	227	214	224	228	227	228	228	226	226	223	217	222	218	213	220	213	225	228	228	225	224
229	224	223	222	222	227	223	227	220	224	224	224	237	145	63	64	73	87	47	191	233	227	222	222	225	224	224	220	223	224	225	221	224	224	224	224	224	224	224	224	224	224	228	228	228	228	228	204	163	193	229	225	229	223	187	160	208	228	227	228	209	186	164	172	193	218	225	229	228	228	211	180	192	210	215	202	177	178	216	226	226	226	228	205	190	164	173	227	228	226	228	228	225	228	196	161	174	195	224	229	202	178	166	172	194	193	195	192	165	219	231	228	228	227	225	211	171	183	200	204	198	179	167	204	228	226	200	174	161	176	196	224	225	229	226	206	182	163	174	196	224	228	227	228	224	230	222	168	183	228	225	228	226	224	229	219	190	166	162	178	225	230	227	228	226	227	198	181	173	194	222	223	196	179	163	180	201	227	228	228	226	228	204	170	185	204	202	199	171	169	217	228	229	227	185	183	200	190	162	209	228	228	228	228	228	228	228	228	228	229	186	161	202	229	228	228	228	228	228	228	228	228	228	228	228	225	229	217	158	205	226	226	227	229	228	228	214	196	172	165	184	194	194	169	209	228	227	228
227	220	198	181	205	225	223	222	226	226	224	230	180	60	87	42	82	117	40	192	201	184	209	209	207	208	204	199	213	218	223	225	224	224	224	224	224	224	224	224	224	223	228	229	228	226	228	203	163	192	228	226	228	228	181	162	216	228	225	227	228	223	169	187	228	228	227	228	227	210	174	221	227	227	228	228	226	198	160	206	229	225	228	229	227	176	163	212	228	225	230	228	225	228	172	164	197	228	226	229	226	221	164	205	228	228	227	226	195	219	226	228	229	227	201	162	212	226	224	227	228	229	195	173	225	226	226	214	165	210	227	229	228	227	230	228	223	164	200	225	228	227	228	228	229	225	209	165	163	218	229	227	228	226	228	230	227	210	163	160	181	225	228	228	229	227	226	210	195	228	227	228	228	214	161	214	226	229	227	228	227	187	167	210	228	227	229	228	227	193	185	225	228	193	190	228	229	227	201	194	228	228	228	228	228	228	228	228	226	227	192	164	206	228	229	228	228	228	228	228	228	228	228	228	228	226	228	187	162	172	226	227	224	225	226	229	229	225	207	161	223	226	228	204	160	215	228	228
227	221	179	144	193	227	215	205	215	222	224	223	61	64	99	97	134	121	74	180	224	209	207	205	209	219	215	202	210	222	226	223	226	224	224	224	224	224	224	224	224	224	228	228	228	227	227	204	160	198	228	225	228	204	162	199	229	227	226	225	228	227	172	192	227	225	229	227	222	157	213	227	228	229	228	226	227	228	193	164	215	226	227	228	226	177	162	177	225	230	223	228	229	208	189	168	197	228	228	229	227	222	162	200	228	226	226	226	225	224	228	225	224	208	158	208	227	229	228	227	228	226	227	195	227	227	226	214	161	209	228	227	228	226	224	228	222	158	202	228	226	228	225	226	228	227	189	205	163	205	225	229	228	226	228	228	226	215	181	193	157	199	226	224	228	228	227	209	192	226	227	228	224	216	163	213	228	227	228	225	191	158	213	229	231	228	223	229	226	223	200	226	225	168	206	228	228	228	219	215	228	228	228	228	228	228	228	228	226	228	191	163	201	228	226	228	228	228	228	228	228	228	228	228	228	224	226	186	199	160	219	229	227	228	228	225	227	226	203	164	219	227	228	219	167	193	226	226
227	221	176	133	187	228	208	196	208	226	232	149	37	108	152	125	134	164	152	190	232	227	228	227	230	227	225	227	222	219	222	224	224	224	224	224	224	224	224	224	224	224	228	227	228	229	228	203	165	179	201	200	186	165	207	226	224	229	229	228	227	224	170	192	228	229	229	226	181	173	228	226	228	225	229	229	229	226	222	162	175	227	227	226	223	183	201	164	218	226	229	227	226	183	216	162	196	228	227	227	227	222	166	202	228	228	228	219	224	228	228	226	221	165	176	226	222	228	227	228	227	226	224	229	228	227	228	217	166	209	226	228	226	225	233	228	218	164	205	226	227	228	228	226	228	225	182	224	187	171	225	228	227	225	228	226	224	213	189	223	187	156	208	227	227	226	228	208	196	226	228	228	228	215	162	215	226	231	227	218	160	190	228	228	226	225	228	229	228	227	228	227	225	170	174	222	228	227	230	228	228	228	228	228	228	228	228	228	227	229	191	162	203	226	227	226	228	228	228	228	228	228	228	228	225	229	210	196	227	166	188	227	228	225	227	228	229	226	202	166	222	225	228	215	160	204	228	228
226	219	182	147	191	228	219	213	219	230	202	106	75	127	195	102	104	145	182	233	224	219	219	223	221	218	215	217	224	227	224	224	224	224	224	224	224	224	224	224	224	224	228	228	228	227	228	203	164	178	207	209	194	179	175	195	225	228	225	229	226	227	171	192	225	228	231	215	164	203	228	229	228	226	230	226	229	228	227	175	166	218	230	225	224	182	225	161	191	227	226	228	215	201	221	163	196	227	228	226	227	220	159	203	227	224	217	189	225	225	229	229	209	161	202	228	228	225	228	227	228	226	231	225	226	225	229	214	162	208	227	227	226	226	225	226	217	163	199	226	229	225	226	228	226	197	200	227	211	158	214	226	228	228	225	227	228	213	181	226	224	176	161	211	229	226	226	210	193	228	226	228	227	217	161	214	225	223	227	192	161	211	229	226	228	228	227	228	228	229	225	229	228	199	160	167	207	225	227	227	228	228	228	228	228	228	228	228	229	228	190	166	205	229	227	228	228	228	228	228	228	228	228	228	228	226	185	211	229	192	161	221	228	228	230	228	227	226	205	160	216	227	213	188	194	226	228	228
229	222	199	182	204	226	224	225	225	227	117	92	158	231	212	139	135	147	174	232	222	219	219	218	216	208	204	206	221	225	226	222	224	224	224	224	224	224	224	224	224	224	228	227	228	228	228	203	164	196	224	229	228	225	200	162	182	228	228	228	229	224	172	187	227	227	226	210	163	199	227	226	229	225	228	228	225	229	227	192	165	212	227	228	218	183	229	192	162	222	225	228	189	223	226	165	190	226	227	228	228	221	172	171	186	182	173	184	225	228	228	228	196	161	199	228	228	226	229	226	228	229	228	224	227	229	226	214	163	176	187	185	189	189	185	187	188	162	201	225	229	226	226	229	229	177	220	228	228	175	187	225	226	227	228	228	227	214	184	227	225	220	169	164	222	226	228	208	196	226	229	229	228	216	158	215	228	228	225	169	164	222	229	226	228	228	228	229	228	226	226	228	228	226	188	164	160	174	216	228	228	228	228	228	228	228	228	228	226	226	191	161	201	228	226	229	228	228	228	228	228	228	228	228	226	227	179	227	225	223	163	204	228	224	228	229	228	226	206	162	183	184	174	177	215	226	228	226
228	228	227	223	224	224	224	224	237	152	67	132	221	220	212	179	120	110	218	225	224	221	210	206	217	205	205	209	224	223	226	222	225	224	224	224	224	224	224	224	224	226	223	228	226	227	228	202	163	191	226	228	228	228	227	182	158	216	227	226	227	226	169	192	226	229	228	209	161	201	228	227	228	225	226	227	226	228	226	194	163	217	228	226	213	181	228	220	162	199	228	222	190	228	223	167	189	228	228	225	228	221	162	202	226	227	226	191	225	226	229	228	197	163	201	226	228	227	228	228	228	228	225	227	228	228	228	215	165	208	227	227	223	226	226	226	217	162	200	226	225	228	228	228	221	165	203	207	205	177	164	217	228	225	229	226	228	212	184	228	229	226	218	166	170	224	225	209	194	228	226	226	228	213	160	217	228	226	225	172	165	219	224	228	228	230	224	228	226	228	228	228	228	226	226	212	178	164	160	214	225	228	228	226	226	227	228	226	228	226	190	161	205	228	226	226	225	228	228	228	226	229	226	227	226	212	166	206	205	206	165	174	225	228	226	227	229	226	204	164	216	229	224	200	164	196	227	228
226	225	224	227	226	224	228	233	193	61	96	181	226	220	210	168	149	219	229	223	226	227	221	223	226	223	225	222	225	224	226	222	224	224	224	224	224	224	224	224	224	226	228	229	225	229	229	201	165	193	228	226	226	227	230	190	162	188	229	224	229	226	171	188	228	225	226	213	160	188	226	226	226	229	228	228	227	229	223	180	168	224	228	231	218	180	224	231	178	171	226	197	213	225	223	170	186	229	225	228	228	219	166	203	226	228	226	221	225	225	228	226	212	161	201	226	229	227	227	227	226	228	229	225	227	230	226	215	162	212	229	225	229	228	225	229	220	160	202	229	229	228	225	226	199	189	204	207	208	204	164	205	225	227	227	226	226	213	182	228	226	227	230	218	159	179	226	211	196	224	227	227	228	217	160	215	228	228	226	192	161	209	229	228	228	228	228	228	228	227	226	226	228	229	229	229	225	207	161	170	224	227	227	229	228	228	228	228	226	229	189	164	202	227	228	229	228	226	227	229	227	226	228	228	228	186	199	210	206	208	193	166	216	229	228	228	227	225	202	163	223	226	225	225	205	163	210	226
228	225	216	216	216	217	221	221	69	74	144	228	221	226	143	103	189	222	215	216	216	217	216	218	217	217	216	219	219	217	215	217	222	223	224	224	224	224	224	224	221	227	226	225	227	228	228	201	160	196	229	226	227	226	228	190	163	198	227	226	228	226	169	191	227	229	226	227	174	164	220	231	226	228	228	228	228	226	227	164	192	228	227	227	211	178	229	224	216	163	209	184	220	228	224	169	189	225	229	227	226	220	164	203	227	228	228	228	227	226	224	228	223	162	175	225	229	226	227	229	228	222	228	227	225	226	228	214	162	206	227	226	227	228	228	228	218	162	203	225	227	226	228	225	182	219	229	227	229	228	185	170	226	227	228	228	226	213	184	228	223	228	223	228	214	160	196	209	193	226	228	228	228	214	162	211	228	226	227	219	164	182	228	228	226	225	228	225	228	228	224	227	229	226	229	228	226	228	201	160	204	229	225	227	226	227	228	226	225	225	194	160	206	227	229	227	228	227	227	221	222	229	225	228	223	175	225	228	228	226	223	166	194	222	228	226	227	228	201	163	217	227	226	225	215	158	195	228
229	219	215	215	218	219	219	128	82	135	208	222	220	202	86	90	224	215	217	221	218	221	218	218	218	218	222	218	216	219	218	218	223	223	224	224	224	224	224	224	226	222	224	229	228	228	228	204	163	194	228	226	228	227	226	175	162	221	226	227	226	226	171	191	226	225	228	226	211	161	187	226	226	227	225	228	229	226	197	172	226	228	226	224	213	177	229	228	224	168	164	199	229	229	223	166	191	229	229	226	228	225	163	201	229	229	225	229	226	217	216	225	228	204	159	198	229	228	232	228	229	231	225	199	219	225	227	218	163	212	229	225	226	230	227	225	219	163	201	223	230	225	228	223	176	222	226	226	228	228	206	161	219	228	227	228	228	213	183	228	228	227	229	226	228	198	160	169	196	226	228	227	228	213	163	215	228	228	227	227	195	161	207	226	228	229	223	228	226	226	188	225	224	221	228	226	228	228	205	161	220	228	225	229	227	228	228	227	228	228	194	162	202	227	228	226	229	227	230	214	187	228	226	226	210	182	227	226	226	227	228	192	163	222	226	228	228	228	205	163	221	229	228	228	211	163	199	227
225	229	226	224	229	231	156	82	123	170	235	223	230	190	72	142	235	222	227	228	217	211	218	222	216	214	216	210	219	216	223	226	224	224	224	224	224	224	224	224	226	222	227	226	228	228	228	202	159	177	224	228	226	222	182	164	195	225	226	227	229	225	170	185	225	230	226	227	226	198	159	194	226	228	226	227	228	204	175	221	227	226	228	229	213	180	228	228	228	200	162	224	228	227	228	164	186	226	226	226	227	217	161	196	228	223	229	229	224	180	220	228	226	229	199	162	196	226	228	228	226	229	215	174	227	226	227	214	160	210	225	232	228	224	226	225	220	162	196	226	229	229	229	200	182	228	226	226	226	229	223	168	186	226	225	226	230	212	183	228	228	226	226	229	227	228	192	159	193	228	228	227	226	215	162	215	226	225	228	229	225	192	163	204	223	227	229	228	228	195	182	229	224	174	221	228	226	224	179	185	229	228	225	227	228	229	227	228	225	226	189	161	180	221	228	228	225	227	222	172	209	229	228	225	183	204	226	227	229	231	228	216	164	207	228	226	226	228	202	159	215	227	228	221	178	177	222	228
228	226	224	224	224	232	114	85	135	232	225	222	233	141	47	184	233	224	220	227	206	179	209	207	209	202	205	180	204	209	222	224	224	224	224	224	224	224	224	224	222	224	229	228	220	208	193	170	168	163	176	189	184	170	175	202	228	227	229	226	215	201	166	166	200	219	227	230	225	228	205	178	180	200	203	202	187	195	223	228	226	225	227	209	179	170	204	223	228	216	177	229	228	224	201	164	168	203	220	226	211	178	168	165	193	203	196	198	174	185	228	228	227	229	228	208	182	183	199	202	203	188	172	202	229	226	214	183	166	177	212	222	228	228	227	213	190	166	172	208	220	227	203	171	171	204	226	229	226	223	205	173	168	190	223	229	207	184	171	200	219	228	228	227	229	227	223	183	191	229	227	226	212	185	166	180	211	226	229	227	225	228	208	177	188	203	205	205	181	177	211	229	227	182	173	204	202	185	186	222	228	228	226	227	228	227	228	228	213	189	167	164	168	174	189	191	190	185	172	170	224	225	224	196	164	184	211	227	226	227	224	198	163	166	199	227	220	204	173	168	178	199	193	177	186	222	226	226
226	226	224	224	226	228	153	90	121	233	227	228	215	71	63	222	227	226	224	226	224	223	224	227	226	224	227	227	227	222	224	224	224	224	224	224	224	224	224	224	228	226	225	229	223	216	208	222	230	221	215	213	213	226	225	227	226	226	227	223	214	214	229	226	212	220	228	228	227	229	229	228	213	203	200	211	223	227	225	228	228	228	227	215	224	226	218	224	226	224	222	226	229	219	212	228	226	212	220	228	215	217	226	227	224	225	215	208	211	221	226	225	228	228	228	224	229	222	209	202	206	216	225	227	226	229	215	218	228	222	212	225	228	227	228	215	217	228	221	210	227	226	211	224	225	209	225	227	224	225	215	227	227	212	224	231	213	222	221	219	221	227	228	226	227	228	228	223	213	227	226	224	208	222	225	220	211	228	225	228	228	228	224	226	215	205	203	206	215	227	226	228	229	225	217	199	206	218	223	226	222	224	229	228	227	228	228	228	219	209	224	228	226	225	227	222	209	213	214	219	228	232	223	213	228	219	214	226	227	227	217	219	227	226	212	227	226	206	221	227	218	206	215	224	228	228	228	229
228	224	224	222	224	222	236	121	99	229	226	227	199	65	87	226	222	223	226	222	224	221	222	216	214	217	214	211	213	221	223	226	223	224	224	224	224	224	224	224	220	224	229	228	228	228	228	229	228	228	228	226	227	225	225	228	228	228	228	227	228	228	226	226	227	229	224	227	227	228	228	228	226	228	226	227	226	228	229	226	227	228	225	227	224	229	228	229	227	229	228	223	228	229	226	228	227	228	228	229	228	226	228	226	226	226	228	228	228	228	226	228	223	226	231	226	228	226	225	228	226	226	228	228	228	225	228	227	226	226	226	229	227	226	230	226	228	228	225	228	226	228	227	229	228	228	228	227	228	226	228	226	229	228	223	226	227	227	228	228	227	224	226	231	225	229	225	228	225	226	228	229	228	228	226	228	229	225	224	229	226	228	227	229	228	227	228	225	226	229	225	228	226	228	225	229	228	227	228	228	230	225	228	228	225	226	228	226	228	227	226	227	225	228	227	226	229	226	228	228	226	223	228	228	225	228	229	226	228	228	228	228	228	226	228	228	227	227	229	228	228	226	228	228	225	226	228	228
228	224	222	221	227	226	233	128	101	230	224	231	162	51	120	232	224	224	224	224	219	216	218	205	190	204	199	186	188	217	229	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
229	225	223	225	223	225	234	144	75	213	234	192	75	63	154	234	222	224	224	226	227	226	225	225	225	228	227	228	225	227	225	225	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
229	224	226	224	225	221	229	205	177	217	230	195	113	89	128	233	224	224	224	224	226	227	225	227	225	221	222	226	227	222	222	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	224	227	223	224	223	226	226	231	227	222	228	181	103	135	214	224	224	224	225	221	223	223	221	226	224	224	221	223	224	224	225	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	225	225	222	225	224	224	224	224	226	222	228	180	110	148	206	225	224	223	224	224	223	224	224	224	226	223	224	224	224	224	222	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
226	226	224	222	224	226	225	221	222	228	224	223	217	202	168	214	225	224	225	224	224	224	224	226	223	224	224	226	226	226	225	226	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
227	227	223	224	224	223	224	222	224	223	221	226	225	227	225	224	224	224	225	223	223	222	222	221	224	224	222	224	224	222	222	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	222	226	224	223	224	224	224	224	226	228	225	223	224	224	224	224	224	224	224	224	224	224	222	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	223	225	224	224	224	224	222	224	224	225	224	224	224	225	222	224	225	224	224	224	223	224	226	224	224	224	224	224	224	224	225	222	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
227	223	223	226	224	222	223	227	224	223	225	224	224	224	221	223	224	225	224	222	224	226	227	222	225	224	224	224	225	225	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
229	226	225	224	222	224	226	224	224	224	224	224	224	224	227	228	227	224	224	223	225	224	224	222	223	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	224	223	224	224	224	225	224	224	223	224	226	226	224	222	222	222	222	222	224	224	221	225	226	225	224	223	224	224	224	224	224	225	225	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
223	224	226	224	223	227	225	223	223	224	224	227	224	225	223	224	219	226	228	226	223	224	224	224	222	222	222	222	222	222	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
230	225	227	224	219	216	220	224	222	224	226	228	226	218	186	166	168	180	192	203	211	219	221	223	222	223	224	221	222	221	221	222	224	222	221	221	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	207	205	219	213	188	204	222	215	212	203	200	190	189	174	149	151	148	163	193	212	218	221	222	222	222	222	222	222	221	221	221	223	221	220	222	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	215	210	220	221	224	223	219	224	197	151	147	171	203	200	163	152	160	170	188	214	224	226	222	224	224	224	224	224	224	224	224	222	220	219	219	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	225	224	224	224	224	224	224	212	182	172	174	178	191	179	173	204	216	217	221	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	224	224	224	224	224	224	224	224	222	218	220	199	185	200	223	225	226	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
228	226	224	224	224	224	224	224	224	224	224	225	224	228	229	225	222	224	225	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	224	226	226	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228	228
];

logo(:, :, 1) = uint8(logoRed);
logo(:, :, 2) = uint8(logoGreen);
logo(:, :, 3) = uint8(logoBlue);


% ------------------------------------------------------------------------- 
% ------------------------------------------------------------------------- 
% ------------------------------------------------------------------------- 


%| ABOUT CALLBACKS:
%| GUIDE automatically appends subfunction prototypes to this file, and 
%| sets objects' callback properties to call them through the FEVAL 
%| switchyard above. This comment describes that mechanism.
%|
%| Each callback subfunction declaration has the following form:
%| <SUBFUNCTION_NAME>(H, EVENTDATA, HANDLES, VARARGIN)
%|
%| The subfunction name is composed using the object's Tag and the 
%| callback type separated by '_', e.g. 'slider2_Callback',
%| 'figure1_CloseRequestFcn', 'axis1_ButtondownFcn'.
%|
%| H is the callback object's handle (obtained using GCBO).
%|
%| EVENTDATA is empty, but reserved for future use.
%|
%| HANDLES is a structure containing handles of components in GUI using
%| tags as fieldnames, e.g. handles.figure1, handles.slider2. This
%| structure is created at GUI startup using GUIHANDLES and stored in
%| the figure's application data using GUIDATA. A copy of the structure
%| is passed to each callback.  You can store additional information in
%| this structure at GUI startup, and you can change the structure
%| during callbacks.  Call guidata(h, handles) after changing your
%| copy to replace the stored original so that subsequent callbacks see
%| the updates. Type "help guihandles" and "help guidata" for more
%| information.
%|
%| VARARGIN contains any extra arguments you have passed to the
%| callback. Specify the extra arguments by editing the callback
%| property in the inspector. By default, GUIDE sets the property to:
%| <MFILENAME>('<SUBFUNCTION_NAME>', gcbo, [], guidata(gcbo))
%| Add any extra arguments after the last argument, before the final
%| closing parenthesis.