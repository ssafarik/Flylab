
% Specify various trial configurations.
t0=30;
t1=3;
t2=117;

i=1;
trials{i,1} = 'HCS';                trials{i,2} = '80mW'; i=i+1;
trials{i,1} = 'HCS';                trials{i,2} = '100mW'; i=i+1;
trials{i,1} = 'HCS';                trials{i,2} = '120mW'; i=i+1;
trials{i,1} = 'HCS';                trials{i,2} = '160mW'; i=i+1;
trials{i,1} = 'HCS';                trials{i,2} = '260mW'; i=i+1;

trials{i,1} = 'TrpA1_paralysis';    trials{i,2} = '100mW'; i=i+1;
trials{i,1} = 'TrpA1_paralysis';    trials{i,2} = '120mW'; i=i+1;
trials{i,1} = 'TrpA1_paralysis';    trials{i,2} = '160mW'; i=i+1;
trials{i,1} = 'TrpA1_paralysis';    trials{i,2} = '260mW'; i=i+1;

trials{i,1} = 'TrpA1_sugar';        trials{i,2} = '260mW'; i=i+1;


% Plot them.
[m,n]=size(trials);
for i=10%1:m
    figure(i); clf;

    strFlies = trials{i,1};
    strLaserPower = trials{i,2};
    
    dirspec = '~/FlylabData/zap*';
    filespec = sprintf ('zapresponse_%s_%02ds_%02ds_%02ds_%s_*.csv', strFlies, t0, t1, t2, strLaserPower);


    txtTitle=sprintf('Response of Fly (%s) to Laser (%s, On %d-%ds)', strFlies, strLaserPower, t0, t0+t1);
    txtTitle(strfind(txtTitle,'_'))=' '; % Convert underscores to spaces.
    title (txtTitle);
    drawnow;

    nFiles = FlylabPlotSpeed(dirspec, filespec);

    txtTitle=sprintf('Response of Fly (%s) to Laser (%s, On %d-%ds), N=%d', strFlies, strLaserPower, t0, t0+t1, nFiles);
    txtTitle(strfind(txtTitle,'_'))=' '; % Convert underscores to spaces.
    title (txtTitle);
    drawnow;
end

