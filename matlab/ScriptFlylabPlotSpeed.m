
dirspec = '~/FlylabData/2013_02_07*';
filespec = '*.csv';
filenames = GetFilenames(dirspec, filespec);

for i = 1:length(filenames)
    filedata = FlylabReadFile(filenames{i});

    [m,n] = size(filedata);
    if m>0
        t       = filedata.states(:,1) - filedata.states(1,1);
        stateFly = FlylabGetObjectState(filedata, 2);
        vxFly   = stateFly(:,4);
        vyFly   = stateFly(:,5);

        plot(t,hypot(vxFly,vyFly));
        title (filenames{i});
        drawnow;
    end
end
