function muscRefList = get_muscRefList(muscle)
% Purpose:  Specifies, for a given muscle, a list of related muscles.  
%           The 'muscle reference list' can be used as an argument in
%           other functions, e.g., to extract information about 
%           EMG on/off times reported by Perry (1992).  
%
% Input:    muscle is the name of the muscle corresponding to the current
%               EMG channel of interest, as returned by get_emgLabels().
%
% Output:   muscRefList is a cell array of strings containing a list
%               of muscle abbreviations; these abbreviations correspond
%               to those in get_emgTimingFromPerry()
%
% Notes:    I was motivated to examine the EMG on/off times reported by 
%           Perry because I noticed a large amount of cross talk in the
%           measured EMG activity of Gillette control subjects. These 
%           'muscle reference lists' represent my attempt to guess which
%           muscles are likely contributing to the observed cross talk.
%           By overlaying Perry's reference data onto plots of the
%           subjects' EMG activity, I hope to more accurately quantify
%           the on/off times of the subjects' muscles.  
%
% ASA, 9-05

switch muscle
    case 'Rectus Femoris'
       muscRefList = {'RF', 'VASmed', 'VASint', 'VASlat'}; 
    case 'Medial Hamstrings'
        muscRefList = {'SM', 'ST', 'GR', 'ADM'};
    case 'Lateral Hamstrings'
        muscRefList = {'BFLH', 'BFSH', 'VASlat'};
    case 'Tibialis Anterior'
         muscRefList = {'TA', 'EDL', 'PERlong'};
    case 'Gastroc-Soleus'
        muscRefList = {'GAS', 'SOL'};
end
return;