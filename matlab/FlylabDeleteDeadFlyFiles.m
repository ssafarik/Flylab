function FlylabDeleteDeadFlyFiles (dirspec, filespec)
% FlylabDeleteDeadFlyFiles (drivespec, dirspec)
% Determine which files in the given drive & dirs contain dead flies, and delete those files.
%

    filenames = GetFilenames(dirspec, filespec);
    
    for k = 1:length(filenames)
        [filedata,iTrigger] = FlylabReadData(filenames{k}, -1);
        if FlylabHasDeadFly(filedata)
            fprintf ('Deleting %s\n', filenames{k});
            delete(filenames{k});
        else
            fprintf ('Keeping  %s\n', filenames{k}); 
        end
    end
    