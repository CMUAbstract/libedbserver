monitor; % set constants

tVcap = [];
tRf = [];
Vcap = [];
rf = [];

% We keep track of ytick and yticklabel as axis tick marks and axis tick
% mark labels.  This allows us to display labels representing RFID
% commands on the plot.  As they grow, each index in ytick will correspond
% to the label at the same index in yticklabel.
ytick = [];
nextYTick = 1;
yticklabel = {};

fp = fopen('data/vcapAndRf.dat', 'r');
data = fread(fp);
data = data';
len = length(data);

curTime = 0;
skip = 0;
for i = 1:len
    if skip > 0
        skip = skip - 1;
        continue;
    end
    
    if data(i) == DELIM_TIME
        % time data
        cycles = 256^3 * data(i + 1) + 256^2 * data(i + 2) + ...
                 256 * data(i + 3) + data(i + 4);
        curTime = cycles * MONITOR_CLK_PERIOD;
        skip = 4; % skip next 4 bytes: we just used them
        
    elseif data(i) == DELIM_VCAP
        % Vcap data
        adc = 256 * data(i + 1) + data(i + 2);
        V = adc / 4096 * VDD;
        tVcap = [tVcap curTime]; % append current time
        Vcap = [Vcap V]; % append Vcap reading
        skip = 2; % skip next 2 bytes: we just used them
        
    elseif data(i) == DELIM_RFRX
        % RF Rx data
        tRf = [tRf curTime]; % append current time
        
        rxByte = data(i + 1);
        skip = 1; % skip next byte: we just used it
        
        % update plotting axis if necessary
        label = rfidPlotTickLabels(find([rfidPlotTickLabels{:}] == rxByte), 2);
        [alreadyPresent, index] = ismember(label, yticklabel);
        if alreadyPresent
            % this label is already on the axis
            rf = [rf ytick(index)];
        else
            % the label is not on the axis yet
            rf = [rf nextYTick];
            
            % add a new label
            ytick = [ytick nextYTick];
            nextYTick = nextYTick + 1;
            yticklabel(1, length(yticklabel) + 1) = label;
        end
    
    elseif data(i) == DELIM_RFTX
        % RF Tx data
        tRf = [tRf curTime]; % append current time
        txByte = DELIM_RFTX;
        
        % update plotting axis if necessary
        label = rfidPlotTickLabels(find([rfidPlotTickLabels{:}] == txByte), 2);
        [alreadyPresent, index] = ismember(label, yticklabel);
        if alreadyPresent
            % this label is already on the axis
            rf = [rf ytick(index)];
        else
            % the label is not on the axis yet
            rf = [rf nextYTick];
            
            % add a new label
            ytick = [ytick nextYTick];
            nextYTick = nextYTick + 1;
            yticklabel(1, length(yticklabel) + 1) = label;
        end
    end
end
                    
% plot the data and make it look nice
if isempty(Vcap)
    error('No Vcap data!');
elseif isempty(rf)
    error('No RF data!');
else
    [ax, pVcap, pRf] = plotyy(tVcap, Vcap, tRf, rf);
end

rxDisplayFactor = 0.3; % leave 30% at the top for Rx data
VcapBottomSpace = 0.05; % leave 5% at the bottom for Vcap data spacing

% change display of Vcap y-axis
Vcap_min = min(Vcap);
Vcap_max = max(Vcap);
diff = Vcap_max - Vcap_min;
range = diff / (1 - (rxDisplayFactor + VcapBottomSpace)); % Vcap y-axis range
y1min = Vcap_min - range * VcapBottomSpace; % min
y1max = Vcap_max + range * rxDisplayFactor; % max
ax(1).YLim(1) = y1min;
ax(1).YLim(2) = y1max;
ax(1).YTick = round(y1min, 1):0.1:round(y1max, 1);

% change p2 line to dots
pRf.LineStyle = 'none';
pRf.Marker = '.';
pRf.MarkerSize = 10;

% change display of RF Rx y-axis
yticks = nextYTick - 1;
y2max = yticks + 1;
y2range = ceil(y2max / (rxDisplayFactor));
y2min = y2max - y2range;
ax(2).YLim(1) = y2min;
ax(2).YLim(2) = y2max;
ax(2).YTick = y2min:y2max;
ax(2).YTick = ytick;
ax(2).YTickLabel = yticklabel;

xlabel('Time (s)');
fclose(fp);