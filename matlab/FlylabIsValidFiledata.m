function bValid = FlylabIsValidFiledata (filedata)
    if (size(fieldnames(filedata.header))>0)
        bValid = true;
    else
        bValid = false;
    end
    