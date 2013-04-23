function histAll = FlylabPlotHistogramPositionFiles(dirspec, filespec, iFrameParent, iFrameChild)
% FlylabPlotPositionHistogramFiles(dirspec, filespec, iFrameParent, iFrameChild)
% Take a set of directories of .csv files (as written by Flylab), and 
% plot a heatmap of where the fly has been.  Does not recurse directories.
%
% dirspec:   e.g. 'E:\\*' or 'C:\\Documents and Settings\\username\\FlylabData\\2012_10_03*'
%            Must end in a *
%

    nFilesMax=999;          % Set this to limit the number of files to look at.
    radius = 20;
    nBins = 256;
    nClipped = 150;

    filenames = GetFilenames(dirspec, filespec);
    [m, nFiles] = size(filenames);
    iFiles = 1:min(nFiles,nFilesMax);

    fprintf ('Found %d files.\n', nFiles)
    fprintf ('Plotting files 1-%d.\n', length(iFiles))
    histFile = zeros(nBins, nBins, nFiles);
    imgFile = cell(nFiles,1);
    for i = iFiles;
        fprintf('File %d: %s ... ', i, filenames{i})
        filedata = FlylabReadFile(filenames{i});
        histFile(:,:,i) = FlylabGetHistogramPosition(filedata, iFrameParent, iFrameChild, radius, nBins);
        %histFile(:,:,i) = FlylabClipByCount(histFile(:,:,i), nClipped);
        
        % Make the histogram into an image, i.e. normalize to 1.
        imgFile{i} = histFile(:,:,i)./max(max(histFile(:,:,i)));  

        fprintf ('done.\n')
    end

    % Get counts for all files, and make an image.
    histAll = sum(histFile,3);
    histClipped = FlylabClipByCount(histAll, nClipped);
    img = histClipped./max(max(histClipped));
    
    kernIdentity = [.00 .00  .00  .00  .00 .00 .00;
                    .00 .00  .00  .00  .00 .00 .00;
                    .00 .00  .00  .00  .00 .00 .00;
                    .00 .00  .00 1.00  .00 .00 .00;
                    .00 .00  .00  .00  .00 .00 .00;
                    .00 .00  .00  .00  .00 .00 .00;
                    .00 .00  .00  .00  .00 .00 .00];

    kernBlur = [.00 .00  .00  .05  .00 .00 .00;
                .00 .05  .18  .32  .18 .05 .00;
                .00 .18  .64 1.00  .64 .18 .00;
                .05 .32 1.00 1.00 1.00 .32 .05;
                .00 .18  .64 1.00  .64 .18 .00;
                .00 .05  .18  .32  .18 .05 .00;
                .00 .00  .00  .05  .00 .00 .00];
    kernBlur = kernBlur / sum(sum(kernBlur));

    % Make color values match with the colormap, and rotate 90 twice.
    imgIdent = rot90(conv2(img,kernIdentity)*length(hot),2);
    imgBlur = rot90(conv2(img,kernBlur)*length(hot),2);
    
%     figure(99);
     hold off;
     cla;
     image(imgIdent); 
     hold on; [m,n]=size(imgIdent); scatter([n/2],[m/2],50,'w');
     colormap(hot);
     axis([0 nBins 0 nBins]);
     axis equal

    figure(gcf+1);
    hold off;
    image(imgBlur); 
    hold on; [m,n]=size(imgBlur); scatter([n/2],[m/2],50,'w');
    colormap(hot);
    axis equal
    axis([0 nBins 0 nBins]);

    