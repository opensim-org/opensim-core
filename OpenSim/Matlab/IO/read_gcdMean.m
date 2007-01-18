function g = read_gcdMean(fname)
% Purpose:  This function reads a GCD file containing composite means and 
%           SDs for control subjects (e.g., 'gilletteMeanSD.GCD') and
%           stores these data in a structure.    
%
% Input:    fname is the name of the ascii datafile to be read 
%           ('character array') 
%
% Ouuput:   g returns a structure with the following format: 
%               g.comment = string from comment line in GCD file
%               g.cycle   = array of % gait cycle values
%               g.tempspatial() .label, .ave, .sd   
%               g.events()      .label, .ave, .sd       
%               g.jntangles()   .label, .ave, .sd  
%               g.jntmoments()  .label, .ave, .sd 
%               g.jntpowers()   .label, .ave, .sd 
%
%           The joint centers and GRFs are not stored, as these data
%           are meaningless when averaged across subjects.
%
% Caveat:   This function was written to read composite GCD files from 
%           Gillette; it has not been tested on files from other centers
%
% ASA 6-05
% Open ascii data file for reading.
fid = fopen(fname, 'r');	
% Check for file error.
% If error, send message to screen and stop execution.
if fid == -1								
	error(['unable to open ', fname])		
end
% Store gait cycle array.
cycle = (0:2:100)';
% Initialize counters used to index structure arrays.
ctr_tempspatial = 0;
ctr_events = 0;
ctr_jntangles = 0;
ctr_jntmoments = 0;
ctr_jntpowers = 0;
% Process the header, up to the $COMMENTS label.
nextline = fgetl(fid);
while ~strcmpi(nextline, '$COMMENTS 1')
    nextline = fgetl(fid);
end
% Process the comment line.
comment = fgetl(fid);  
% To avoid comparing and storing labels with extra characters at the 
% beginning and end of the line (e.g., the ! that precedes each line),
% define range of characters of interest; other characters are ignored.
label_range = '2:length(nextline)-3';   
% Process the data.
nextline = fgetl(fid);
while nextline ~= -1        % continue reading until end of file
 
    switch nextline(eval(label_range)) 
            
       % IF nextline is a label corresponding to temporospatial data,
       % update counter, store label, read data, and read next line.
        case {'Cadence',               ... 
              'StrideTime',            ...     
              'StePTime',              ...
              'SingleSupport',         ...
              'DoubleSupport',         ...  
              'StrideLength',          ...    
              'StePLength',            ...
              'Speed'}                    
            ctr_tempspatial = ctr_tempspatial + 1;
            tempspatial(ctr_tempspatial).label = nextline(eval(label_range)); 
            temp = fscanf(fid, '%f %f\n', 2);
            tempspatial(ctr_tempspatial).ave = temp(1);
            tempspatial(ctr_tempspatial).sd = temp(2);            
            nextline = fgetl(fid);
                          
        % IF nextline is a label corresponding to event data,
        % update counter, store label, read data, and read next line.
        case {'OppositeFootOff';       ... 
              'OppositeFootContact';   ...
              'FootOff'}
            ctr_events = ctr_events + 1;
            events(ctr_events).label = nextline(eval(label_range));
            temp = fscanf(fid, '%f %f\n', 2);
            events(ctr_events).ave = temp(1);
            events(ctr_events).sd = temp(2);
            nextline = fgetl(fid);                          
               
        % IF nextline is a label corresponding to joint angle data,
        % update counter, store label, read data, and read next line.
        case {'PelvicTilt';            ...
              'PelvicObliquity';       ... 
              'PelvicRotation';        ...
              'HipFlexExt';            ...
              'HipAbAdduct';           ...
              'HipRotation';           ...
              'KneeFlexExt';           ...
              'KneeValgVar';           ...
              'KneeRotation';          ...
              'DorsiPlanFlex';         ...
              'FootRotation';          ...
              'FootProgression'}
            ctr_jntangles = ctr_jntangles + 1;
            jntangles(ctr_jntangles).label = nextline(eval(label_range)); 
            temp = fscanf(fid, '%f %f\n', [2, length(cycle)])';
                        % transpose since fscanf fills columns before rows
            jntangles(ctr_jntangles).ave = temp(:, 1);
            jntangles(ctr_jntangles).sd = temp(:, 2);
            nextline = fgetl(fid);
          
        % IF nextline is a label corresponding to joint moment data,
        % update counter, store label, read data, and read next line.        
        case {'HipFlexExtMoment';      ...
              'HipAbAdductMoment';     ...
              'HipRotationMoment';     ...
              'KneeFlexExtMoment';     ...
              'KneeValgVarMoment';     ...
              'KneeRotationMoment';    ...
              'DorsiPlanFlexMoment';   ...
              'FootAbAdductMoment';    ...
              'FootRotationMoment'}
            ctr_jntmoments = ctr_jntmoments + 1;
            jntmoments(ctr_jntmoments).label = nextline(eval(label_range)); 
            temp = fscanf(fid, '%f %f\n', [2, length(cycle)])';
                        % transpose since fscanf fills columns before rows
            jntmoments(ctr_jntmoments).ave = temp(:, 1);
            jntmoments(ctr_jntmoments).sd = temp(:, 2);
            nextline = fgetl(fid);
        
        % IF nextline is a label corresponding to joint power data,
        % update counter, store label, read data, and read next line.        
        case {'HipPower';              ...
              'HipFlexExtPower';       ...
              'HipAbAdductPower';      ...
              'HipRotationPower';      ...
              'KneePower';             ...
              'KneeFlexExtPower';      ...
              'KneeValgVarPower';      ...
              'KneeRotationPower';     ...
              'AnklePower';            ...
              'DorsiPlanFlexPower';    ...
              'AnkleAbAdductPower';    ...
              'AnkleRotationPower'}
            ctr_jntpowers = ctr_jntpowers + 1;
            jntpowers(ctr_jntpowers).label = nextline(eval(label_range));
            temp = fscanf(fid, '%f %f\n', [2, length(cycle)])';
                        % transpose since fscanf fills columns before rows
            jntpowers(ctr_jntpowers).ave = temp(:, 1);
            jntpowers(ctr_jntpowers).sd = temp(:, 2);
            nextline = fgetl(fid);
                 
        % IF nextline is a label corresponding to pelvis translations, or 
        % joint centers or GRFs, skip to next label.
        case {'PelvicOrigin-3';        ...
              'HipJointCentre-3';      ...   
              'KneeJointCentre-3';     ...
              'AnkleJointCentre-3';    ...
              'GroundReaction-3-2';    ...
              'ForcePlateCorners-3'}
            for i = 1:length(cycle)+1
                nextline = fgetl(fid);
            end
           
        otherwise
            sprintf('case otherwise; nextline = %s\n', nextline);
            nextline = fgetl(fid);        
    end     
end     
fclose(fid);
% Store data in structure and return.
g.comment = comment;
g.cycle = cycle;
g.tempspatial = tempspatial;
g.events = events;
g.jntangles = jntangles;
g.jntmoments = jntmoments;
g.jntpowers = jntpowers;
            
