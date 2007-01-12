function done = printmenu(figHandleArray, prefix)

global GLOBAL_individualprintmenus GLOBAL_figHandles;

GLOBAL_figHandles = [GLOBAL_figHandles figHandleArray];

if ~GLOBAL_individualprintmenus
	done = 1;
	return;
end

done = 0;
while ~done
    query = 'Send figures to printer?';
    opt1 = 'print';
    opt2 = 'write to AI file';
    opt3 = 'done';
    userInput = menu(query, opt1, opt2, opt3);
    switch userInput
        case 1
            for figNum = 1:length(figHandleArray)
                orient(figHandleArray(figNum), 'tall');  
                printCommand = ...
                   ['print -f', num2str(figHandleArray(figNum)), ' -r600'];
                eval(printCommand);
            end
            done = 1;
        case 2
            for figNum = 1:length(figHandleArray)
                orient(figHandleArray(figNum), 'tall');  
                printCommand = ...
                ['print -f', num2str(figHandleArray(figNum)), ...
                ' -dill ', prefix, num2str(figNum), '.ill'];
                eval(printCommand);
            end
            done = 1;
        case 3
            done = 1;
    end
end
