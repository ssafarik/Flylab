function bValid = FlylabIsValidFiledata (filedata)
    if (size(fieldnames(filedata.header))>0)
        bValid = True;
    else
        bValid = False;
    end
    