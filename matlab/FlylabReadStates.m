function statesOut = FlylabReadStates (filename)
% statesOut = FlylabReadStates(filename)
% Read the fly & robot state data lines from the given file.
%
% filename:     The .csv file to read.
% statesOut:    All the states.
%

    try
        statesOut = csvread(filename, 49, 0);
    catch
        statesOut = []; % Empty file.
    end
    
