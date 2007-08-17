function done = combinedprintmenu(figHandleArray, prefix)

done = 0;
while ~done
    query = 'Send all figures to printer?';
    opt1 = 'print';
    opt2 = 'write to combined PS file';
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
			filename = [prefix '.ps'];
			disp(sprintf('Writing to "%s"', filename));
            for figNum = 1:length(figHandleArray)
				disp(sprintf('Processing figure %d (%d/%d)', figHandleArray(figNum), figNum, length(figHandleArray)));
                orient(figHandleArray(figNum), 'tall');  
                set(figHandleArray(figNum), 'InvertHardCopy', 'off');
				if figNum == 1
					printCommand = ['print -f', num2str(figHandleArray(figNum)), ' -dpsc2 ', filename];
				else
					printCommand = ['print -f', num2str(figHandleArray(figNum)), ' -append -dpsc2 ', filename];
				end
                eval(printCommand);
			end
            done = 1;
        case 3
            done = 1;
    end
end
