classdef TRCWorker  < matlab.mixin.SetGet
% TRCWorker Read and Write TRC files
%   Utility  Class that uses OpenSim's TRCFileAdapter() class to read
%   and write OpenSim tables.
    properties
        table 
        name
        path
    end
    methods
        function readTRC(obj)
            import org.opensim.modeling.*
            if isempty(obj.name)
                error('name not set. use setName() to set a file name')
            elseif isempty(obj.path)
                error('path not set. use setPath() to set a file path')
            end
            % define the absolute path to the file. Append file type (.trc)
            % if not included. 
            if isempty(strfind(obj.name, '.trc'))
                fullfilepath = fullfile(obj.path, [obj.name '.trc']);
            else
                fullfilepath = fullfile(obj.path, [obj.name]);
            end
            % Use the OpenSim adapter to read the file
            table = TRCFileAdapter().read(fullfilepath)
            % set the objects table
            obj.setTable(table)
        end
        function writeTRC(obj)
            import org.opensim.modeling.*
            if isempty(obj.name)
                error('name not set. use setName() to set a file name')
            elseif isempty(obj.path)
                error('path not set. use setPath() to set a file path')
            end
            % edit the marker names for errors (spaces, *, ...)
            obj.updateMarkerNames()
            % define the absolute path to the file. Append file type (.trc)
            % if not included. 
            if isempty(strfind(obj.name, '.trc'))
                fullfilepath = fullfile(obj.path, [obj.name '.trc']);
            else
                fullfilepath = fullfile(obj.path, [obj.name]);
            end
            % use OpenSim file adapter to print the table to file
            TRCFileAdapter().write(obj.table, fullfilepath );
            
            display(['TRC file written: ' fullfilepath ]);
        end
        function setTable(obj,osimtable)
            % Sets the internal table to a new table
            
            % check that the input class is correct
            if isempty(strfind(osimtable.getClass, 'TimeSeriesTableVec3'))
               error('Input is not an OpenSim Vec 3 Times Series Table');
            end
            % update the internal table
            obj.table = osimtable;
        end 
        function setName(obj,name)
            % Set the filename 
            obj.name = name;
        end
        function name = getName(obj)
            % get the filename
            name = obj.name;
        end
        function setPath(obj, dirPath)
            % set the directory path
            obj.path = dirPath;
        end
        function dirPath = getPath(obj)
           dirPath = obj.path; 
        end
   end
    methods (Access = private, Hidden = true)
        function updateMarkerNames(obj)
            % method for stripping illegal types of chars from the coloumn
            % names of the marker
            table_clone = obj.table.clone();
            labels = table_clone.getColumnLabels();
            % defined set of illegal chars
            illegalChars = [{' '} {'*'} {'.'}];
            for i = 0 : labels.size() - 1
                label = char(labels.get(i));
                % replace illegal chars with underscore
                for u = 1 : length(illegalChars)
                    if ~isempty(strfind(label, illegalChars{u}))
                        disp_label = strrep(label,illegalChars{u}, '_');
                        disp(['Illegal Coloumn label. ' label ' changed to ' disp_label ]);
                        label = disp_label;
                    end
                end
                labels.set(i,label);
            end
            % set the column labels
            table_clone().setColumnLabels(labels)
            obj.setTable(table_clone)
        end
    end
end