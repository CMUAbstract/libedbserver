monitor; % set constants

tRf = [];
rf = [];

% We keep track of ytick and yticklabel as axis tick marks and axis tick
% mark labels.  This allows us to display labels representing RFID
% commands on the plot.  As they grow, each index in ytick will correspond
% to the label at the same index in yticklabel.
ytick = [];
nextYTick = 1;
yticklabel = {};

fp = fopen('data/rf.dat', 'r');
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
        % Vcap data - skip it
        skip = 2;
        
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
            txIndex = index;
        else
            % the label is not on the axis yet
            rf = [rf nextYTick];
            
            % add a new label
            ytick = [ytick nextYTick];
            nextYTick = nextYTick + 1;
            txIndex = length(yticklabel) + 1;
            yticklabel(1, txIndex) = label;
        end
    end
end
                    
% plot the data and make it look nice
p = plot(tRf, rf);
p.LineStyle = 'none';
p.Marker = '.';
p.MarkerSize = 10;

ax = gca; % get the current axes handle
ax.YLim = [ax.YLim(1) - 1, ax.YLim(2) + 1];
ax.YTick = ytick;
ax.YTickLabel = yticklabel;
xlabel('Time (s)');
fclose(fp);