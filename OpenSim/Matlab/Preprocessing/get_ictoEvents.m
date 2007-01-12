function ictoEvents = get_ictoEvents(c, tInfo, figHandle, ref_dataFormat)
% Purpose:  Detects events corresponding to initial contact and toe-off
%           from vertical GRF and marker data, stored in structure c,
%           read from the C3D files of Gillette control subjects. 
%
%           For each forceplate hit specified in tInfo.FP, the following
%           events are detected: ic, oto, oic, to, and icNext.
%
%           IC events are determined from the vertical GRF and from the 
%               forward velocity of the AJC.
%           TO events are determined from the vertical GRF and from the 
%               forward velocity of the TOE. 
%
% Input:    c is a structure returned from read_c3DFile()
%           tInfo is a structure containing the following 'trial info':
%               *.trial - trial number to analyze ('character array')
%               *.speed - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%           figHandle specifies the number of the figure window,
%               used to display events detected from marker data.
%
% Output:  ictoEvents returns a structure with the following format:
%          *(fpHitNum).ic       - IC, to nearest analog frame
%          *(fpHitNum).oto      - opposite TO, to nearest analog frame
%          *(fpHitNum).oic      - opposite IC, to nearest analog frame
%          *(fpHitNum).to       - TO, to nearest analog frame
%          *(fpHitNum).icNext   - IC of next step, to nearest analog frame
%
% Called Functions:  
%   get_ictoFromGRF(c, tInfo)
%   get_icFromMkr(c, icFromGRF, fzData, tInfo, figHandle)
%   get_toFromMkr(c, toFromGRF, fzData, tInfo, figHandle)
%
% ASA, 9-05


% Get IC and TO events from vertical GRF data.
[icFromGRF, toFromGRF, fzData] = get_ictoFromGRF(c, tInfo, ref_dataFormat);
 
% Get IC and TO events from marker data.
[ricFromMkr, licFromMkr] = ...
            get_icFromMkr(c, icFromGRF, fzData, tInfo, figHandle);                    
[rtoFromMkr, ltoFromMkr] = ...
            get_toFromMkr(c, toFromGRF, fzData, tInfo, figHandle);
      
% Define tolerances (# analog frames = time * sampling rate) to
%   identify OTO and icNext > IC.
tol_OTO = 0.02*c.analog.rate;
tol_icNext = 0.05*c.analog.rate;

% Store elements of ictoEvents.
nHits = length(tInfo.FP);
for fpHitNum = 1:nHits
    
    % IC and TO
    ictoEvents(fpHitNum).ic = icFromGRF(fpHitNum);
    ictoEvents(fpHitNum).to = toFromGRF(fpHitNum);

    % OTO and OIC
    if fpHitNum == 1
        if strcmpi(tInfo.limb{fpHitNum}, 'R')
            otoIndex = ...
                min(find(ltoFromMkr > ictoEvents(fpHitNum).ic + tol_OTO));
            ictoEvents(fpHitNum).oto = ltoFromMkr(otoIndex);
        elseif strcmpi(tInfo.limb{fpHitNum}, 'L')
            otoIndex = ...
                min(find(rtoFromMkr > ictoEvents(fpHitNum).ic + tol_OTO));
            ictoEvents(fpHitNum).oto = rtoFromMkr(otoIndex);
        end
        ictoEvents(fpHitNum).oic = icFromGRF(fpHitNum + 1);
    else
        ictoEvents(fpHitNum).oto = ictoEvents(fpHitNum - 1).to;
        ictoEvents(fpHitNum).oic = ictoEvents(fpHitNum - 1).icNext;
    end    
    
    % IC Next
    if nHits > fpHitNum + 1
        ictoEvents(fpHitNum).icNext = icFromGRF(fpHitNum + 2);
    else
        if strcmpi(tInfo.limb{fpHitNum}, 'R')
            icNextIndex = ...
              min(find(ricFromMkr > ictoEvents(fpHitNum).ic + tol_icNext));
            ictoEvents(fpHitNum).icNext = ricFromMkr(icNextIndex);
        elseif strcmpi(tInfo.limb{fpHitNum}, 'L')
            icNextIndex = ...
              min(find(licFromMkr > ictoEvents(fpHitNum).ic + tol_icNext));
            ictoEvents(fpHitNum).icNext = licFromMkr(icNextIndex);
        end
   end
end

% Display ictoMatrix on screen for reference.
tableTitle = sprintf('\n\t%s\n\t%s', ...
                'IC and TO Events Corresponding to FP Hits:', ...
                '(analog sample #s, in order hit)');
tableHeadings = sprintf('\n%8s\t%8s\t%8s\t%8s\t%8s\t%8s', ...
                 'FP#', 'IC', 'OTO', 'OIC', 'TO', 'IC+');
disp(tableTitle);
disp(tableHeadings);   

for fpHitNum = 1:nHits          
   table(fpHitNum).row = sprintf('%8d\t%8d\t%8d\t%8d\t%8d\t%8d', ... 
                         tInfo.FP{fpHitNum}, ...
                         ictoEvents(fpHitNum).ic, ...
                         ictoEvents(fpHitNum).oto, ...
                         ictoEvents(fpHitNum).oic, ...
                         ictoEvents(fpHitNum).to, ...
                         ictoEvents(fpHitNum).icNext);
    disp(table(fpHitNum).row);
end
return;
 
