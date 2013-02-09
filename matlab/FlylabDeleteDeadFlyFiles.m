function FlylabDeleteDeadFlyFiles (dirspec, filespec)
% FlylabDeleteDeadFlyFiles (drivespec, dirspec)
% Determine which files in the given drive & dirs contain dead flies, and delete those files.
%

    filenames = GetFilenames(dirspec, filespec);
    
    for k = 1:length(filenames)
        filedata = FlylabReadFile(filenames{k});
        fprintf ('-----------\n');
        if FlylabHasDeadFly(filedata)
            fprintf ('---Deleting %s\n', filenames{k});
            delete(filenames{k});
            
            [a,b,c]=fileparts(filenames{k});
            fprintf ('---Deleting %s\n', [a,b,'.mov']);
            delete([a,b,'.mov']);

            fprintf ('---rmdir %s\n', [a,b]);
            rmdir([a,b],'s');
        else
            fprintf ('Keeping  %s\n', filenames{k}); 
        end
    end
    