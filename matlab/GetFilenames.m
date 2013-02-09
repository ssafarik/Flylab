function filenames = GetFilenames (dirspec, filespec)
% filenames = GetFilenames (dirspec, leafspec, filespec)
% Take a directory, leafdir spec, and filespec, 
% Return a cell array of filenames.
%
% dirspec:   Must end with a backslash, e.g. 'E:\\' 
%            or 'C:\\Documents and Settings\\username\\FlylabData\\2012_10_03*'
% filespec:  May contain wildcards, e.g. 'zap*.csv'
%
    [base, leaf] = fileparts(dirspec);
    dirs = dir(dirspec);
    [mDirs nDirs] = size(dirs);
    k = 1;
    filenames = {};
    for iDir = 1:mDirs
        nameDir = strcat(base, filesep, dirs(iDir).name, filesep);
        filesDir = dir(strcat(nameDir,filespec));
        [mFiles nFiles] = size(filesDir);
        for iFile = 1:mFiles
            filename = strcat(nameDir, filesDir(iFile).name);
            filenames{k} = filename;
            k = k+1;
        end
    end
