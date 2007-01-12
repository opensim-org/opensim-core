function c = read_c3dFile(fname, ref_dataFormat)
% Purpose:  This function reads a C3D file and stores the data relevant
%           for generating a subject-specific simulation in a structure. 
%
% Input:    fname is the name of the C3D file to be read 
%           ('character array') 
%
% Output:   c returns a structure with the following format: 
%               c.subject
%               c.trial                 .num, .type 
%               c.events()              .label, .time
%               c.fpInfo()              .corners, .origin
%
%               POINT/VIDEO Data:
%               c.video         .rate,  .nframes, .units
%               c.markers()             .label, .data
%               c.jntcenters    .L(),   .label, .data
%                               .R(),   .label, .data
%               c.jntangles     .L(),   .label, .data 
%                               .R(),   .label, .data
%               c.jntmoments    .L(),   .label, .data
%                               .R(),   .label, .data
%               c.jntpowers     .L(),   .label, .data
%                               .R(),   .label, .data
%
%               ANALOG Data:
%               c.analog                .rate,  .ratio
%               c.grf()                 .label, .data
%               c.emg()                 .label, .data
%
%           If the relevant data are not found in the file,
%           the corresponding arrays are returned empty.
%
% Called Functions:
%           This function activates MLS's C3D Server (tested on v.1127)
%           as a COM object using Matlab's ActiveX interface.  
%           The C3D Server can be downloaded and installed from:
%                       http://www.c3d.org/
%                       http://www.emgsrus.com/c3d_server.htm
%
%           Other subfunctions include:
%           get_c3dEvents(itf)  - retrieves EVENT data from C3D file
%           get_c3dForcePlatform(itf) - retrieves FORCE_PLATFORM data
%                       describing FP corners and origins from C3D file
%           get_c3dVideo(itf)   - retrieves POINT/VIDEO data from C3D file
%           get_c3dAnalog(itf)  - retrieves ANALOG data from C3D file
%           store_c3dData(fname, type, v, a) - 
%                       packages the data read into the structure returned
%           
% Caveats:  This function was written to read C3D files from Gillette;
%           it has not been tested on C3D files from other centers.
%
%           NOTE:  The function must be run from the folder containing 
%                  the C3D files to be read.
%
% ASA 6-05, revised 9-05
% Commands for activating the C3D Server are based on functions provided by
% Matthew Walker and Michael Rainbow, Shriners Motion Analysis Lab, Erie PA
% Activate C3D Server as a COM.
itf = actxserver('C3DServer.C3D');
% Verify C3D registration.  
% If error, send message to screen and stop execution.
p = itf.GetRegistrationMode;
if p == 0
    error('Error: Please install the C3D Server from Motion Lab Systems');
end
% Open C3D file 'fname', read contents, and verify that file was read.
p = itf.Open(fname, 3);
if p == 1
    error(['Error: Unable to read ', fname]);
end

% Retrieve trial type from SUBJECTS:IS_STATIC.
parameter_index = itf.GetParameterIndex('SUBJECTS', 'IS_STATIC');
if parameter_index ~= -1
	item_index = 0;         % index corresponding to 1st item
	ttype = itf.GetParameterValue(parameter_index, item_index);
else
	disp('read_c3dFile: No SUBJECTS:IS_STATIC parameter, using 0');
	ttype = 0;
end

% Retrieve EVENT data.
e = get_c3dEvents(itf);
% Retrieve FORCE_PLATFORM data.
f = get_c3dForcePlatform(itf);
% Retrieve POINT/VIDEO data.
v = get_c3dVideo(itf, ref_dataFormat);
% Retrieve ANALOG data.
a = get_c3dAnalog(itf, ref_dataFormat);
% Close C3D file.
itf.Close;
% Store data in structure and return.
c = store_c3dData(fname, ttype, e, f, v, a);
return;
