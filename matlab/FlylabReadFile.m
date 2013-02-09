function filedata=FlylabReadFile(filename)
% filedata = FlylabReadFile(filename)
% Read the header lines and state lines from the given .csv file (as written by Flylab).
% 
% Return a struct containing all the various fields:  filedata.header, and filedata.states
%

    filedata = struct();
    filedata.header = FlylabReadHeader(filename);
    filedata.states = FlylabReadStates(filename);

