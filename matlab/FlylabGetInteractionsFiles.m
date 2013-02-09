function interactions = FlylabGetInteractionsFiles(dirspec, filespec, iFrameParent, iFrameChild, nMinLen, criteria)
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
