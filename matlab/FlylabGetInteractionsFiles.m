function interactions = FlylabGetInteractionsFiles(dirspec, filespec, iFrameParent, iFrameChild, nMinLen, criteria)
% iFrameParent: Frame number of the parent object (1=Robot, 2=Fly1, etc)
% iFrameChild:  Frame number of the child object (1=Robot, 2=Fly1, etc)
% nMinLen:      Minimum length of a valid interaction sequence.

    filenames = GetFilenames(dirspec, filespec);

    interactions=[];
    for k=1:length(filenames); 
        fprintf ('Analyzing %s... ', filenames{k});
        filedata = FlylabReadFile(filenames{k}); 
        [start,stop] = FlylabGetInteractions(filedata, iFrameParent, iFrameChild, nMinLen, criteria); 
        fprintf ('%d interactions.\n', length(start));
        for i=1:length(start)
            interactions = [interactions; {filenames{k}, start(i), stop(i)}];
        end; 
    end;
    fprintf ('Done.\n');
