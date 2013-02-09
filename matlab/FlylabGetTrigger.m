function [iTriggerStart, iTriggerEnd] = FlylabGetTrigger(filedata)
% iTriggerStart:     Index of the trigger rising edge.
% iTriggerEnd:       Index of the trigger falling edge.
%

    % Return data for N steps prior to the trigger.
    iTriggers = find(filedata.states(:,2)==1);
    iTriggerStart = iTriggers(1);
    iTriggerEnd = iTriggers(length(iTriggers));
    
