classdef osimList
properties
        model;
        list;
    end
    
    methods 
        function obj = osimList(model,classname)
            if nargin == 0
                error('no inputs to constructor')
            elseif nargin == 1
                error('constructor takes two inputs, not one')
            elseif nargin > 2
                error(['2 inputs required, num2str(nargin) ' given])
            end
            

            switch classname
                case 'Body'
                    list = model.getBodyList();
                case 'Frame'
                    list = model.getFrameList();
                case 'Muscle'
                    list = model.getMuscleList();
                case 'Joint'
                    list = model.getJointList();
                case 'Actuator' 
                    list = model.getActuatorList();
            end
            % disp('List creation Successful')
            % allocate the list and model to local properties
            obj.list = list;
            obj.model = model;
        end
        function size = getSize(obj)
            % get the size of 
            list = obj.list;
            li = list.begin();
            size = 0;
            while ~li.equals(list.end())
                 li.next();
                 size = size + 1;
            end
        end
        function names = getNames(obj)
            list = obj.list;
            li = list.begin();
            names = [{}];
            while ~li.equals(list.end())
                names = [names {char(li.getName())}];
                li.next();
            end
            names = names';
        end
        function outputnames = getOutputNames(obj)
            list = obj.list;
            li = list.begin();
            outNames = li.getOutputNames();
            sz = outNames.size();
            outputnames = [];
            for j = 0 :sz - 1 
                outputnames = [outputnames {char(outNames.get(j))}];
            end
            outputnames = outputnames';
        end
        function reference = get(obj,name)
            import org.opensim.modeling.*
            model = obj.model;
            list = obj.list;
            li = list.begin();
            while ~li.equals(list.end())
                if strcmp(char(li.getName()),name)
                    pathname = li.getAbsolutePathName();
                    comp = model.getComponent(pathname);
                    class = comp.getConcreteClassName();
                    eval(['reference = ' char(class) '.safeDownCast(comp);'])
                    break
                end
                li.next();
            end 
            if ~exist('reference','var')
                error(['Component name: ' name ' is not found'])
            end
        end
    end
end

       