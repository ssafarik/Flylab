%strFlies = 'HCS';
strFlies = 'TrpA1_paralysis';
strLaserPower = '120mW';
t0=30;
t1=3;
t2=117;

f=8;

dirspec = '~/FlylabData/zap*';
filespec = sprintf ('zapresponse_%s_%02ds_%02ds_%02ds_%s_*.csv', strFlies, t0, t1, t2, strLaserPower);
%filespec = ['zapresponse_', strFlies,'_',sprintf('%02d',t0),'s_03s_117s_', strLaserPower, '_*.csv'];
filenames = GetFilenames(dirspec, filespec);

nFiles = length(filenames);
%figure(1); clf;
figure(f); clf;
speedTotal = [];
speedAll = [];
tAll = [];

for i = 1:nFiles
    filedata = FlylabReadFile(filenames{i});
    if (FlylabIsValidFiledata(filedata))
        [m,n] = size(filedata);
        if m>0
            t       = filedata.states(:,1) - filedata.states(1,1);
            stateFly = FlylabGetObjectState(filedata, 2);
            vxFly   = stateFly(:,4);
            vyFly   = stateFly(:,5);
            speed = hypot(vxFly,vyFly);
            
            if ~isempty(speedTotal)
                nSpeed = length(speed);
                nSpeedTotal = length(speedTotal);
                nMin = min(nSpeed,nSpeedTotal);
                speedTotal = speedTotal(1:nMin) + speed(1:nMin);
            else
                speedTotal = speed;
                nMin = length(speedTotal);
            end
            
            speedAll = [speedAll; speed];
            tAll = [tAll; t];
            
%             figure(1);
%             subplot(2, 1, 1);
%             plot(t,speed);
%             txtTitle=filenames{i};
%             txtTitle(strfind(txtTitle,'_'))=' '; % Convert underscores to spaces.
%             title (txtTitle);
%             xlim([0,t(nMin)]);
%             set(gca,'XTick',[0,25,30,33,50,75,100,125,150]);
%             
%             subplot(2,1,2);
%             speedMean = speedTotal(1:nMin)/i;
%             plot(t(1:nMin), speedMean);
%             title (['Mean Speed of ', num2str(i), ' Trials']);
%             xlabel('time');
%             xlim([0,t(nMin)]);
%             set(gca,'XTick',[0,25,30,33,50,75,100,125,150]);

            figure(f);
            nTimeValues = 200;%floor(max(tAll))+1;
            tMax = max(tAll);
            nSpeedValues = 100;
            speedMax = 40;
            edges = cell(2,1); 
            edges{1} = linspace(0, tMax, nTimeValues); % Time axis values.
            edges{2} = logspace(log10(speedMax)-4, log10(speedMax), nSpeedValues);  % Speed axis values 0.01 to 100.
            %edges{2} = linspace(0, speedMax, nSpeedValues);
            h1 = rot90(hist3 ([tAll, speedAll], edges));
            
            [m1, n1] = size(h1);
            h2 = h1(m1:-1:1,:); % Reverse the histogram pixels.
            h2(1,:) = 0; % Zero out spurious zero values.

            img = (h2 ./ max(max(h2)))*length(hot); % Make it an image.
            image(img);
            
            axis xy;
            set(gca, 'YScale', 'log');
            %set(gca, 'YScale', 'linear');
            n=7;
            set(gca, 'XTick', nTimeValues/150*[0, 25, 30, 33, 50, 75, 100, 125, 150]);
            set(gca, 'XTickLabel', {'0','25','30','33','50','75','100','125','150'});
            %set(gca, 'XTick', linspace(0,nTimeValues,n));
            %set(gca, 'XTickLabel', {num2str(floor(0*tMax/n)),num2str(floor(1*tMax/n)),num2str(floor(2*tMax/n)),num2str(floor(3*tMax/n)),num2str(floor(4*tMax/n)),num2str(floor(5*tMax/n)),num2str(floor(6*tMax/n)),num2str(floor(7*tMax/n)),num2str(floor(8*tMax/n))});

            set(gca, 'YTick', [nSpeedValues/100,nSpeedValues/20,nSpeedValues/10,nSpeedValues/2,nSpeedValues]);
            set(gca, 'YTickLabel', {num2str(speedMax/100),num2str(speedMax/20),num2str(speedMax/10),num2str(speedMax/2),num2str(speedMax)});
            xlabel ('time (s)');
            ylabel ('fly speed (mm/s)');
            %txtTitle=['Response of Fly (', filedata.header.flyspec.descriptionFlies, ') to Laser (', strLaserPower,', On 30-33s), N=', num2str(i)];
            txtTitle=sprintf('Response of Fly (%s) to Laser (%s, On %d-%ds), N=%d', filedata.header.flyspec.descriptionFlies, strLaserPower, t0, t0+t1, i);
            txtTitle(strfind(txtTitle,'_'))=' '; % Convert underscores to spaces.
            title (txtTitle);
            
            drawnow;
        end
    end
end
