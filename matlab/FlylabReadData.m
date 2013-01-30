function [dataOut, iTriggerOut] = FlylabReadData (filename, nPretrigger)
% dataOut = FlylabReadData(filename)
% Read the fly & robot state data lines from the given file.
%
% filename: The .csv file to read.
% nPretrigger:  Number of samples to return prior to the trigger.
% dataOut:      All the filedata from pretrigger to posttrigger.
% iTrigger:     Index of the trigger sample.
%

    if nPretrigger==-1
        nPretrigger = intmax/2;
    end
    
    dataFile = csvread(filename, 49, 0);
    
    % Return data for N steps prior to the trigger.
    iTriggers = find(dataFile(:,2)==1);
    iTrigger = iTriggers(1);
    iPretrigger = max(1,iTrigger-nPretrigger);
    iEnd = iTriggers(length(iTriggers));
    
    iTriggerOut = max(1,iTrigger-iPretrigger);
    dataOut = dataFile(iPretrigger:iEnd,:);
    