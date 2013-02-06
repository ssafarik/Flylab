% This script plots all the fly/fly interactions in a set of files.

dirspec='F:\\2013_01a_478*';
filespec='*.csv';
iFrameParent = 2;   % 1=robot, 2=fly1, 3=fly2, etc
iFrameChild = 3;    % 1=robot, 2=fly1, 3=fly2, etc
nMinLen = 30;       % Minimum length of an interaction.
nPre = 10;           % Number of samples before interaction start.
nPost = 10;          % Number of samples after interaction end.
nSubsample = 5;     % Only plot every Nth sample.  Increase this if you get an Out-Of-Memory error.
criteria = 'chase'; % What counts as an interaction:  'all' or 'chase'
radius = 10;        % How big is the histogram.
nBins = 256;        % Image resolution of histograms.


fprintf ('Reading files and finding interactions.  This may take some time...\n');
interactions = FlylabGetInteractionsFiles(dirspec, filespec, iFrameParent, iFrameChild, nMinLen, nPre, nPost, criteria);

fprintf ('Plotting interactions.\n');
figure(1);
FlylabPlotAllInteractions(interactions, nPre, nPost, nSubsample);

fprintf ('Plotting interactions distances.\n');
figure(gcf+1);
FlylabPlotDistanceInteractions(interactions, nPre, nPost);

fprintf ('Plotting interactions Histogram.\n');
figure(gcf+1);
FlylabPlotHistogramInteractions(interactions, radius, nBins);

fprintf ('Done.\n');
