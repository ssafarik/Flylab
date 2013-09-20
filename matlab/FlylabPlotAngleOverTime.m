function FlylabPlotAngleOverTime(dirspec, filespec)
% FlylabPlotAngleOverTime(dirspec, leafspec)
% Take a set of directories of .csv files (written by Flylab), 
% and plot the mean angle trajectory, and the individual angle trajectories.
% This is used for viewing how the fly orientation changes over time.
%
% dirspec:   Must end with a backslash, e.g. 'E:\*' 
%            or 'C:\\Documents and Settings\\username\\FlylabData\\2012_10_03*'
% filespec:  May contain wildcards, e.g. 'zap*.csv'

    nFilesMax=999;          % Set this to limit the number of files to look at.

    filenames = GetFilenames(dirspec, filespec);
    [m,nFiles] = size(filenames);
    iFiles = 1:min(nFiles,nFilesMax);

    fprintf ('Found %d files.\n', nFiles)
    nFiles = length(iFiles);
    fprintf ('Plotting files 1-%d.\n', nFiles)
    timeFly = cell(nFiles,1);
    stateFly = cell(nFiles,1);
    angleFly = cell(nFiles,1);
    lenMin = 9999999999;
    lenMax = 0;

    for i = iFiles
        fprintf('File %d: %s ... ', i, filenames{i})
        filedata = FlylabReadFile(filenames{i});

        stateFly{i} = FlylabGetObjectState(filedata, 2);
        timeFly{i} = filedata.states(:,1);
        angleFly{i} = stateFly{i}(:,3);
        lenMin = min(lenMin,length(angleFly{i}));
        lenMax = max(lenMax,length(angleFly{i}));
        fprintf ('done.\n')
    end

    % Compute the mean time array.
    timeMean = zeros(lenMin,1);
    timeMax = 0;
    len = lenMin;
    for i = iFiles
        timeMean = timeMean + timeFly{i}(1:len) - timeFly{i}(1);
        timeMax = max(timeMax, max((timeFly{i}(1:len) - timeFly{i}(1))));
    end
    timeMean = timeMean / nFiles / 60;
    timeMax = (timeMax / 60) * 1.1;
    n=length(timeMean);

    % Compute the angle statistics.
    angleMean = zeros(len,1);
    angleMat = [];
    for i = iFiles
        angleMean = angleMean + unwrap(angleFly{i}(1:len)) - angleFly{i}(1);
        angleMat = [angleMat, unwrap(angleFly{i}(1:len)) - angleFly{i}(1)]; % Time vertically, Flies horizontally.
    end
    angleMean = angleMean / nFiles;
    angleStd = std(angleMat')' / sqrt(nFiles); % Array of std deviations over time.

    indices = 1:floor(n/20):n;
    
    % Linear fit to angleMean.
    p = polyfit(timeMean, angleMean, 1);
    linMean = linspace(0, p(1)*timeMean(n), n) + p(2);
    
    % Compute confidence intervals of mean.
    ciAll = [];
    for i=indices
        [h,p,ci] = ttest(angleMat(i,:)', linMean(i), 0.05);
        ciAll = [ciAll ci];
    end

    
    %[q1 q2]=size(timeMean(indices));
    %fprintf('timeMean(indices) %d %d', q1, q2);
    %[q1 q2]=size(angleMean(indices));
    %fprintf('angleMean(indices) %d %d', q1, q2);
    %[q1 q2]=size(ciAll);
    %fprintf('ciAll %d %d\n', q1, q2);

    figure(1); 
    clf;

    % Plot the mean angle with errorbars.
    subplot(2,1,1);
    cla;
    hold off;
    errorbar(timeMean(indices), angleMean(indices), ciAll(1,:), ciAll(2,:)); %angleStd(indices));
    hold on
    plot(timeMean, linMean, '-.r');
    title(sprintf('flytype=%s,\nMean Angle Over Time When Laser On Clockwise Fly Rotation,\ntotal trial time=%0.1f hours, 95%% Confidence Interval', ...
        filedata.header.flyspec.description, ...
        timeMean(n)*nFiles/60));
    ylabel('radians');
    xlabel('minutes');
    xlim([0 timeMax]);
    ylim([-200 200]);

    % Plot the individual angle trajectories.
    subplot(2,1,2);
    cla;
    hold on;
    for i=1:nFiles
        plot((timeFly{i}-timeFly{i}(1))/60, unwrap(angleFly{i}-angleFly{i}(1))); 
    end
    title(sprintf('Individual Angle Trajectories, nTraj=%d', nFiles));
    ylabel('radians');
    xlabel('minutes');
    xlim([0 timeMax]);
    ylim([-200 200]);
    hold off;
