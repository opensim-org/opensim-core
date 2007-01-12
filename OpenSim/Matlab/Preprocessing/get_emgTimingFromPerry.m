function perryData = get_emgTimingFromPerry(muscAbbr)
% Purpose:  Returns EMG on/off times as a percent of the gait cycle for 
%           the muscle specified by muscAbbr, or for 'all' of the muscles
%           reported in Perry's 1992 book.
%
% Input:    muscAbbr is one of the muscle abbreviations listed below, or
%               'all' to return data for all muscles.
%
% Output:   perryData is a structure with the following format:
%           *(nMusclesReturned - either 1 or 27)
%                         .muscle    - name of muscle
%                         .onoff     - nx2 matrix of [on off] events, 
%                                      where each row is one 'burst'
%                         .onoffAlt  - 'alternate' on/off timings reported 
%
%           NOTE:  an 'on' value of 0 indicates muscle was on before IC
%                  an 'off' value of 100 indicates muscle was on after TSw
%
% ASA, 9-05


% Return perryData as specified by muscAbbr.
if strcmpi(muscAbbr, 'GMAX upper')
    perryData.muscle = 'GMAX upper';
    perryData.onoff = [0 24; 94 100];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'GMAX lower')
    perryData.muscle = 'GMAX lower';
    perryData.onoff = [0 10; 94 100];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'GMED')
    perryData.muscle = 'GMED';
    perryData.onoff = [0 29; 95 100];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'TFL')
    perryData.muscle = 'TFL';
    perryData.onoff = [26 39];
    perryData.onoffAlt = [0 43];
elseif strcmpi(muscAbbr, 'BFLH')
    perryData.muscle = 'BFLH';
    perryData.onoff = [0 5; 79 100];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'SM')        
    perryData.muscle = 'SM';
    perryData.onoff = [0 5; 81 100];
    perryData.onoffAlt = [0 27; 74 100];
elseif strcmpi(muscAbbr, 'ST')         
    perryData.muscle = 'ST';
    perryData.onoff = [0 4; 87 100];
    perryData.onoffAlt = [0 30; 85 100];
elseif strcmpi(muscAbbr, 'GR')  
    perryData.muscle = 'GR';
    perryData.onoff = [64 78];
    perryData.onoffAlt = [46 100];
elseif strcmpi(muscAbbr, 'ADM')        
    perryData.muscle = 'ADM';
    perryData.onoff = [0 6; 90 100];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'AL')    
    perryData.muscle = 'AL';
    perryData.onoff = [44 77];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'IL')
    perryData.muscle = 'IL';
    perryData.onoff = [60 72];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'SAR')
    perryData.muscle = 'SAR';
    perryData.onoff = [57 70];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'RF')        
    perryData.muscle = 'RF';
    perryData.onoff = [55 66];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'VASmed')   
    perryData.muscle = 'VASmed';
    perryData.onoff = [0 16; 87 100];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'VASint')
    perryData.muscle = 'VASint';
    perryData.onoff = [0 20; 93 100];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'VASlat')         
    perryData.muscle = 'VASlat';
    perryData.onoff = [0 15; 88 100];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'BFSH')     
    perryData.muscle = 'BFSH';
    perryData.onoff = [62 81];
    perryData.onoffAlt = [31 44];
elseif strcmpi(muscAbbr, 'GAS')       
     perryData.muscle = 'GAS';
     perryData.onoff = [6 48];
     perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'SOL')     
    perryData.muscle = 'SOL';
    perryData.onoff = [3 52];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'PTIB')       
    perryData.muscle = 'PTIB';
    perryData.onoff = [0 15; 27 51];
    perryData.onoffAlt = [0 51];
elseif strcmpi(muscAbbr, 'FDL')  
    perryData.muscle = 'FDL';
    perryData.onoff = [10 54];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'FHL')
    perryData.muscle = 'FHL';
    perryData.onoff = [47 54];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'PERbrev')
    perryData.muscle = 'PERbrev';
    perryData.onoff = [16 54];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'PERlong')        
    perryData.muscle = 'PERlong';
    perryData.onoff = [11 49];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'TA')         
    perryData.muscle = 'TA';
    perryData.onoff = [0 9; 54 100];
    perryData.onoffAlt = [0 20; 54 100];
elseif strcmpi(muscAbbr, 'EHL')        
    perryData.muscle = 'EHL';
    perryData.onoff = [0 10; 57 100];
    perryData.onoffAlt = [];
elseif strcmpi(muscAbbr, 'EDL')        
    perryData.muscle = 'EDL';
    perryData.onoff = [0 11; 55 100];
    perryData.onoffAlt = [];
    
elseif strcmpi(muscAbbr, 'all')      
    perryData(1).muscle = 'GMAX upper';
    perryData(1).onoff = [0 24; 94 100];
    perryData(1).onoffAlt = [];

    perryData(2).muscle = 'GMAX lower';
    perryData(2).onoff = [0 10; 94 100];
    perryData(2).onoffAlt = [];

    perryData(3).muscle = 'GMED';
    perryData(3).onoff = [0 29; 95 100];
    perryData(3).onoffAlt = [];

    perryData(4).muscle = 'TFL';
    perryData(4).onoff = [26 39];
    perryData(4).onoffAlt = [0 43];

    perryData(5).muscle = 'BFLH';
    perryData(5).onoff = [0 5; 79 100];
    perryData(5).onoffAlt = [];

    perryData(6).muscle = 'SM';
    perryData(6).onoff = [0 5; 81 100];
    perryData(6).onoffAlt = [0 27; 74 100];

    perryData(7).muscle = 'ST';
    perryData(7).onoff = [0 4; 87 100];
    perryData(7).onoffAlt = [0 30; 85 100];

    perryData(8).muscle = 'GR';
    perryData(8).onoff = [64 78];
    perryData(8).onoffAlt = [46 100];

    perryData(9).muscle = 'ADM';
    perryData(9).onoff = [0 6; 90 100];
    perryData(9).onoffAlt = [];

    perryData(10).muscle = 'AL';
    perryData(10).onoff = [44 77];
    perryData(10).onoffAlt = [];

    perryData(11).muscle = 'IL';
    perryData(11).onoff = [60 72];
    perryData(11).onoffAlt = [];

    perryData(12).muscle = 'SAR';
    perryData(12).onoff = [57 70];
    perryData(12).onoffAlt = [];

    perryData(13).muscle = 'RF';
    perryData(13).onoff = [55 66];
    perryData(13).onoffAlt = [];

    perryData(14).muscle = 'VASmed';
    perryData(14).onoff = [0 16; 87 100];
    perryData(14).onoffAlt = [];

    perryData(15).muscle = 'VASint';
    perryData(15).onoff = [0 20; 93 100];
    perryData(15).onoffAlt = [];

    perryData(16).muscle = 'VASlat';
    perryData(16).onoff = [0 15; 88 100];
    perryData(16).onoffAlt = [];

    perryData(17).muscle = 'BFSH';
    perryData(17).onoff = [62 81];
    perryData(17).onoffAlt = [31 44];

    perryData(18).muscle = 'GAS';
    perryData(18).onoff = [6 48];
    perryData(18).onoffAlt = [];

    perryData(19).muscle = 'SOL';
    perryData(19).onoff = [3 52];
    perryData(19).onoffAlt = [];

    perryData(20).muscle = 'PTIB';
    perryData(20).onoff = [0 15; 27 51];
    perryData(20).onoffAlt = [0 51];

    perryData(21).muscle = 'FDL';
    perryData(21).onoff = [10 54];
    perryData(21).onoffAlt = [];

    perryData(22).muscle = 'FHL';
    perryData(22).onoff = [47 54];
    perryData(22).onoffAlt = [];

    perryData(23).muscle = 'PERbrev';
    perryData(23).onoff = [16 54];
    perryData(23).onoffAlt = [];

    perryData(24).muscle = 'PERlong';
    perryData(24).onoff = [11 49];
    perryData(24).onoffAlt = [];

    perryData(25).muscle = 'TA';
    perryData(25).onoff = [0 9; 54 100];
    perryData(25).onoffAlt = [0 20; 54 100];

    perryData(26).muscle = 'EHL';
    perryData(26).onoff = [0 10; 57 100];
    perryData(26).onoffAlt = [];

    perryData(27).muscle = 'EDL';
    perryData(27).onoff = [0 11; 55 100];
    perryData(27).onoffAlt = [];

else
    perryData = [];
    error('Invalid muscle abbreviation; EMG data not returned.');
end
return;