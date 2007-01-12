function correctedAngles = ...
                get_kAxisCorrectedAngles(measuredAngles, offset, limb)
% Purpose:  Corrects 'measured' knee flex/ext, knee var/val, knee rotation,  
%           and hip rotation angles for a malaligned knee axis.  
%
% Input:    measuredAngles is a structure comprised of the following:
%               *.kf - array of 'measured' knee flex/ext angles
%                .kv - array of 'measured' knee var/val angles
%                .kr - array of 'measured' knee rotation angles
%                .hr - array of 'measured' hip rotation angles
%           NOTE:  These are the angles as reported in C3D file; 
%                  if limb == 'L', these angles must be converted to
%                  a right-handed coordinate system before applying the
%                  corrections as formulated here.
%           offset is the assumed angle of malalignment (in degrees).
%           limb specifies whether the 'L' or 'R' limb is to be analyzed
%
%           Sign Conventions:  
%           The sign conventions for the equations, as formulated here,
%           are based on a right-handed coordinate system with X anterior, 
%           Y left, and Z superior.
%           For a R limb, flex+, var+, int+ (consistent w/ gait lab).
%           For a L limb, flex+, val+, ext+ (inconsistent w/ gait lab).
%           offset is a positive rotation about the Z axis. 
%
% Output:   correctedAngles returns a structure comprised of the following:
%               *.kf - array of 'corrected' knee flex/ext angles
%                .kv - array of 'corrected' knee var/val angles
%                .kr - array of 'corrected' knee rotation angles
%                .hr - array of 'corrected' hip rotation angles
%                .offset - offset used for correction
%
%           Sign Conventions:  
%           The sign conventions for the output joint angles are 
%           consistent with conventional gait lab descriptions.
%
% ASA, 7-05, revised 10-05


% Get sine and cosine of offset (in degrees).
So = sind(offset);
Co = cosd(offset);

% Get sines and cosines of measured angles (arrays, in degrees).
% If L limb data, convert to right-handed coordinate system. 
if strcmpi(limb, 'R')
    Sf = sind(measuredAngles.kf);
    Cf = cosd(measuredAngles.kf);
    Sv = sind(measuredAngles.kv);
    Cv = cosd(measuredAngles.kv);
    Sr = sind(measuredAngles.kr);
    Cr = cosd(measuredAngles.kr);
elseif strcmpi(limb, 'L')
    Sf = sind(measuredAngles.kf);
    Cf = cosd(measuredAngles.kf);
    Sv = sind(-1*measuredAngles.kv);    % flip sign for var/val, rotation
    Cv = cosd(-1*measuredAngles.kv);
    Sr = sind(-1*measuredAngles.kr);
    Cr = cosd(-1*measuredAngles.kr);
end

% For each point in gait cycle, compute corrected joint angles in the
% reference right-handed coordinate system.
npts = length(measuredAngles.kf);
for i = 1:npts
    kvCorrected(i) = ...                    % corrected knee var/val
        asind(Sv(i)*Co - Sf(i)*Cv(i)*So);
    
    kfCorrected(i) = ...                    % corrected knee flex/ext
        asind((Sf(i)*Cv(i)*Co + Sv(i)*So)/cosd(kvCorrected(i)));
        
    krCorrected(i) = ...                    % corrected knee rotation
        acosd(((-Cf(i)*Sr(i) + Sf(i)*Sv(i)*Cr(i))*So ...
                + Cv(i)*Cr(i)*Co)/cosd(kvCorrected(i)));
            
    % Resolve ambiguity in solution for krCorrected --
    % i.e, acos returns a positive angle, but solution could be + or -
    if strcmpi(limb, 'R')
        if offset > 0       % corrected is more internal(+) than measured
            if measuredAngles.kr(i) + offset < 0
                krCorrected(i) = -1 * krCorrected(i);   % choose - solution
            else
                krCorrected(i) = krCorrected(i);        % choose + solution
            end
           
        elseif offset < 0   % corrected is more external(-) than measured
            if measuredAngles.kr(i) + offset > 0
                krCorrected(i) = krCorrected(i);        % choose + solution
            else
                krCorrected(i) = -1 * krCorrected(i);   % choose - solution
            end

        elseif offset == 0
            krCorrected(i) = measuredAngles.kr(i);
        end
        
    % For L, must convert measured angles to right-handed CS.    
    elseif strcmpi(limb, 'L')
        if offset > 0      % corrected is more external(+) than measured
            if -1*measuredAngles.kr(i) + offset < 0
                krCorrected(i) = -1 * krCorrected(i);   % choose - solution
            else
                krCorrected(i) = krCorrected(i);        % choose + solution
            end
           
        elseif offset < 0   % corrected is more internal(-) than measured
            if -1*measuredAngles.kr(i) + offset > 0
                krCorrected(i) = krCorrected(i);        % choose + solution
            else
                krCorrected(i) = -1 * krCorrected(i);   % choose - solution
            end

        elseif offset == 0   
            krCorrected(i) = -1 * measuredAngles.kr(i); 
        end
    end
end

% Return corrected angles in structure;
% if data correspond to L limb, convert back to gait lab coordinates.
if strcmpi(limb, 'R')
    correctedAngles.kf = kfCorrected;
    correctedAngles.kv = kvCorrected;
    correctedAngles.kr = krCorrected;
    correctedAngles.hr = measuredAngles.hr - offset;
    correctedAngles.offset = offset;
elseif strcmpi(limb, 'L') 
    correctedAngles.kf = kfCorrected;
    correctedAngles.kv = -1*kvCorrected;
    correctedAngles.kr = -1*krCorrected;   
    correctedAngles.hr = measuredAngles.hr + offset;    
    correctedAngles.offset = offset;    
end
return;
