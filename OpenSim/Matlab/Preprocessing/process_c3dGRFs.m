function [GRFTz_byLimb, COP_smoothed] = ...
                        process_c3dGRFs(cSim, tInfo, figHandleArray)
% Purpose:  Calls functions to: 
%           (1) extract reaction forces and moments from a 'simulateable' 
%               segment of analog GRF data, read from the C3D file of a 
%               Gillette subject, 
%           (2) convert these data to *action* forces and moments, 
%               acting on each FP, 
%           (3) compute the COP and vertical torque from the 
%               action forces and moments, for each FP that was hit, 
%           (4) convert the GRF, Tz, and COP data from the  
%               FP coordinate systems to the lab coordinate system, 
%           (5) convert the action forces and moments back to reaction
%               forces and moments, in the lab coordinate system
%
%           The data are plotted for inspection.
%
% Input:    cSim is a structure returned from extract_c3dSimData
%           tInfo is a structure containing the following 'trial info':
%               *.trial - trial number to analyze ('character array')
%               *.mass  - mass of subject in kg
%               *.speed - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%               *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%           figHandleArray is a 1x2 array corresponding to the numbers of
%               the figure windows for plotting the R and L data.
%
% Notes:    1. A C3D file stores Fx, Fy, Fz, Mx, My, Mz as *reaction* 
%              forces and moments in the FP coordinate sytems.
%           2. Times are referenced to IC of the 1st FP strike, which
%              is defined as t = 0.
%
% Called Functions:
%       extract_c3dActionGRFM(c, tInfo)
%       compute_c3dTzCOP(actionGRFM_FP, fpInfo, analogRate)
%       convert_FPtoLabCS(GRFTz_FP, fpInfo)
%       convert_actionToReaction(actionGRFTz)
%       get_GRFTzByLimb(GRFTz_byFP, tInfo)
%       smooth_COP(GRFTz_byLimb, cSim, tInfo, figHandleArray)
%
% ASA, 10-05


% From the analog GRF data in cSim, extract 'reaction' forces and moments,
% convert to 'action' forces and moments, and apply a low-pass filter  
% for each FP that was hit (in the FP coordinate systems).
[actionGRFM_FP, fpInfo] = extract_c3dActionGRFM(cSim, tInfo); 

% Compute COP and Tz from the 'action' forces and moments,
% for each FP that was hit (in the FP coordinate systems).
actionGRFTz_FP = compute_c3dTzCOP(actionGRFM_FP, fpInfo, cSim.analog.rate);

% Convert 'action' forces, Tz, and COP from the FP coordinate systems
% to the lab coordinate system, for each FP that was hit.
actionGRFTz_lab = convert_FPtoLabCS(actionGRFTz_FP, fpInfo);

% Convert 'action' forces and moments to 'reaction' forces and moments,
% for each FP that was hit (in the lab coordinate system).
reactionGRFTz_lab = convert_actionToReaction(actionGRFTz_lab);

% Superimpose GRFTz data from the FP hits corresponding to each limb.
GRFTz_byLimb = get_GRFTzByLimb(reactionGRFTz_lab, tInfo);

% Plot the GRFTz data for each limb, and interactively eliminate
% discontinuities in the COP trajectories.
COP_smoothed = smooth_COP(GRFTz_byLimb, cSim, tInfo, figHandleArray);
return;
