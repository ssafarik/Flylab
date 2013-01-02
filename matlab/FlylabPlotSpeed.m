
dirspec = 'E:\\2012_07_27\\';
filespec = '*.csv';
filenames = GetFilenames(dirspec, filespec);

for i = 1:length(filenames)
    [filedata,iTrigger] = FlylabReadData(filenames{i}, -1);

    [m,n] = size(filedata);
    if m>0
        t       = filedata(:,1) - filedata(1,1);
        vxFly   = filedata(:,12);
        vyFly   = filedata(:,13);

        plot(hypot(vxFly,vyFly));
    end
end
