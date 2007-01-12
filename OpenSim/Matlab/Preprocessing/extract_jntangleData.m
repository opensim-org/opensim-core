function jntangleData = extract_jntangleData(c, abbr, limb)
% Purpose:  Extracts measured joint angle data from c, a structure 
%           obtained from read_c3dFile() or from extract_c3dSimData(), 
%           for the joint specified by abbr.
%
% Input:    c is a structure that contains the following data, 
%             in addition to other information:
%                   *.video         .rate,  .nframes
%                   *.jntangles     .L()    .label, .data
%                                   .R()    .label, .data
%           abbr is the jntangle label, as specified in the C3D file.
%           limb indicates if the data correspond to the 'R' or 'L' limb
%
% Output:   jntangleData returns a matrix(nVideoFrames, 3) of angles 
%           corresponding to the joint specified by abbr, in the order
%           given in the C3D file; for Gillette control subjects,  
%           the order is:  flexion, adduction, and rotation.
%           
% ASA, 10-05


% Get lists of labels corresponding to joint angle data, 
% for limb of interest, in c.
if strcmpi(limb, 'R')
    nAngles = length(c.jntangles.R);
    for jntangleIndex = 1:nAngles
        jntangleLabels{jntangleIndex} = c.jntangles.R(jntangleIndex).label;
    end
elseif strcmpi(limb, 'L')
    nAngles = length(c.jntangles.L);
    for jntangleIndex = 1:nAngles
        jntangleLabels{jntangleIndex} = c.jntangles.L(jntangleIndex).label;
    end
end
    
% Extract joint angles specified by abbr.
% NOTE:  The C3D Server returns cell arrays;
%        these arrays are converted to numeric arrays.
abbrIndex = strmatch(abbr, jntangleLabels, 'exact');
if isempty(abbrIndex)
    jntangleData = [];
    warning('jntangle unable to be extracted from input structure.');
else
    for vframeNum = 1:c.video.nframes
        if strcmpi(limb, 'R')
            flexion(vframeNum)   = c.jntangles.R(abbrIndex).data{vframeNum, 1};
            adduction(vframeNum) = c.jntangles.R(abbrIndex).data{vframeNum, 2};
            rotation(vframeNum)  = c.jntangles.R(abbrIndex).data{vframeNum, 3};
        elseif strcmpi(limb, 'L')
            flexion(vframeNum)   = c.jntangles.L(abbrIndex).data{vframeNum, 1};
            adduction(vframeNum) = c.jntangles.L(abbrIndex).data{vframeNum, 2};
            rotation(vframeNum)  = c.jntangles.L(abbrIndex).data{vframeNum, 3}; 
        end
    end
    jntangleData = [flexion' adduction' rotation'];
end
return;



