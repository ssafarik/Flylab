
dirspec = 'C:\\flyranch\\2012_12_21*';
filespec = '*.csv';
filenames = GetFilenames(dirspec, filespec);

for i = 1:5%length(filenames)
    [filedata,iTrigger] = FlylabReadData(filenames{i}, -1);

    [m,n] = size(filedata);
    if m>0
        t       = filedata(:,1) - filedata(1,1);
        vxFly   = filedata(:,12);
        vyFly   = filedata(:,13);

        plot(t,hypot(vxFly,vyFly));
        title (filenames{i});
        drawnow;
    end
end
