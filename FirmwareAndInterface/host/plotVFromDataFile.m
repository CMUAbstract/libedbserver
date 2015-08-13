% author: Graham Harvey
% date: 28 April 2015
%
% This file will plot up to all 5 of Vcap, Vrect, Vboost,
% Vreg, and PWM LPF ADC readings from a data file on the same axes.

monitor; % set constants

tVcap = [];
Vcap = [];
tVrect = [];
Vrect = [];
tVboost = [];
Vboost = [];
tVreg = [];
Vreg = [];
tVinj = [];
Vinj = [];

fp = fopen('data/v.dat', 'r');
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
        
    elseif data(i) == DELIM_VRECT
        % Vrect data
        adc = 256 * data(i + 1) + data(i + 2);
        V = adc / 4096 * VDD;
        tVrect = [tVrect curTime]; % append current time
        Vrect = [Vrect V]; % append Vrect reading
        skip = 2; % skip next 2 bytes: we just used them
        
    elseif data(i) == DELIM_VBOOST
        % Vboost data
        adc = 256 * data(i + 1) + data(i + 2);
        V = adc / 4096 * VDD;
        tVboost = [tVboost curTime]; % append current time
        Vboost = [Vboost V]; % append Vboost reading
        skip = 2; % skip next 2 bytes: we just used them
        
    elseif data(i) == DELIM_VREG
        % Vreg data
        adc = 256 * data(i + 1) + data(i + 2);
        V = adc / 4096 * VDD;
        tVreg = [tVreg curTime]; % append current time
        Vreg = [Vreg V]; % append Vreg reading
        skip = 2; % skip next 2 bytes: we just used them
        
    elseif data(i) == DELIM_VINJ
        % Vinj data
        adc = 256 * data(i + 1) + data(i + 2);
        V = adc / 4096 * VDD;
        tVinj = [tVinj curTime]; % append current time
        Vinj = [Vinj V]; % append PWM LPF reading
        skip = 2; % skip next 2 bytes: we just used them
        
    elseif data(i) == DELIM_RFRX
        % RF Rx data - skip it
        skip = 1;
    end
end

% plot the data on the same axes
% first determine which vectors to plot
bit_Vcap = 1;
bit_Vrect = 2;
bit_Vboost = 3;
bit_Vreg = 4;
bit_Vinj = 5;
plot_bitmask = 0;
if ~isempty(Vcap)
    plot_bitmask = bitset(plot_bitmask, bit_Vcap);
end
if ~isempty(Vrect)
    plot_bitmask = bitset(plot_bitmask, bit_Vrect);
end
if ~isempty(Vboost)
    plot_bitmask = bitset(plot_bitmask, bit_Vboost);
end
if ~isempty(Vreg)
    plot_bitmask = bitset(plot_bitmask, bit_Vreg);
end
if ~isempty(Vinj)
    plot_bitmask = bitset(plot_bitmask, bit_Vinj);
end

% now plot them
switch plot_bitmask
    case 1
        plot(tVcap, Vcap);
        ylabel('V_{cap}');
    case 2
        plot(tVrect, Vrect);
        ylabel('V_{rect}');
    case 3
        plot(tVcap, Vcap, tVrect, Vrect);
        legend('V_{cap}', 'V_{rect}');
        ylabel('V (V)');
    case 4
        plot(tVboost, Vboost);
        ylabel('V_{boost}');
    case 5
        plot(tVcap, Vcap, tVboost, Vboost);
        legend('V_{cap}', 'V_{boost}');
        ylabel('V (V)');
    case 6
        plot(tVrect, Vrect, tVboost, Vboost);
        legend('V_{rect}', 'V_{boost}');
        ylabel('V (V)');
    case 7
        plot(tVcap, Vcap, tVrect, Vrect, tVboost, Vboost);
        legend('V_{cap}', 'V_{rect}', 'V_{boost}');
        ylabel('V (V)');
    case 8
        plot(tVreg, Vreg);
        ylabel('V_{reg}');
    case 9
        plot(tVcap, Vcap, tVreg, Vreg);
        legend('V_{cap}', 'V_{reg}');
        ylabel('V (V)');
    case 10
        plot(tVrect, Vrect, tVreg, Vreg);
        legend('V_{rect}', 'V_{reg}');
        ylabel('V (V)');
    case 11
        plot(tVcap, Vcap, tVrect, Vrect, tVreg, Vreg);
        legend('V_{cap}', 'V_{rect}', 'V_{reg}');
        ylabel('V (V)');
    case 12
        plot(tVboost, Vboost, tVreg, Vreg);
        legend('V_{boost}', 'V_{reg}');
        ylabel('V (V)');
    case 13
        plot(tVcap, Vcap, tVboost, Vboost, tVreg, Vreg);
        legend('V_{cap}', 'V_{boost}', 'V_{reg}');
        ylabel('V (V)');
    case 14
        plot(tVrect, Vrect, tVboost, Vboost, tVreg, Vreg);
        legend('V_{rect}', 'V_{boost}', 'V_{reg}');
        ylabel('V (V)');
    case 15
        plot(tVcap, Vcap, tVrect, Vrect, tVboost, Vboost, tVreg, Vreg);
        legend('V_{cap}', 'V_{rect}', 'V_{boost}', 'V_{reg}');
        ylabel('V (V)');
    case 16
        plot(tVinj, Vinj);
        ylabel('V_{inj}');
    case 17
        plot(tVcap, Vcap, tVinj, Vinj);
        legend('V_{cap}', 'V_{inj}');
        ylabel('V (V)');
    case 18
        plot(tVrect, Vrect, tVinj, Vinj);
        legend('V_{rect}', 'V_{inj}');
        ylabel('V (V)');
    case 19
        plot(tVcap, Vcap, tVrect, Vrect, tVinj, Vinj);
        legend('V_{cap}', 'V_{rect}', 'V_{inj}');
        ylabel('V (V)');
    case 20
        plot(tVboost, Vboost, tVinj, Vinj);
        legend('V_{boost}', 'V_{inj}');
        ylabel('V (V)');
    case 21
        plot(tVcap, Vcap, tVboost, Vboost, tVinj, Vinj);
        legend('V_{cap}', 'V_{boost}', 'V_{inj}');
        ylabel('V (V)');
    case 22
        plot(tVrect, Vrect, tVboost, Vboost, tVinj, Vinj);
        legend('V_{rect}', 'V_{boost}', 'V_{inj}');
        ylabel('V (V)');
    case 23
        plot(tVcap, Vcap, tVrect, Vrect, tVboost, Vboost, tVinj, Vinj);
        legend('V_{cap}', 'V_{rect}', 'V_{boost}', 'V_{inj}');
        ylabel('V (V)');
    case 24
        plot(tVreg, Vreg, tVinj, Vinj);
        legend('V_{reg}', 'V_{inj}');
        ylabel('V (V)');
    case 25
        plot(tVcap, Vcap, tVreg, Vreg, tVinj, Vinj);
        legend('V_{cap}', 'V_{reg}', 'V_{inj}');
        ylabel('V (V)');
    case 26
        plot(tVrect, Vrect, tVreg, Vreg, tVinj, Vinj);
        legend('V_{rect}', 'V_{reg}', 'V_{inj}');
        ylabel('V (V)');
    case 27
        plot(tVcap, Vcap, tVrect, Vrect, tVreg, Vreg, tVinj, Vinj);
        legend('V_{cap}', 'V_{rect}', 'V_{reg}', 'V_{inj}');
        ylabel('V (V)');
    case 28
        plot(tVboost, Vboost, tVreg, Vreg, tVinj, Vinj);
        legend('V_{boost}', 'V_{reg}', 'V_{inj}');
        ylabel('V (V)');
    case 29
        plot(tVcap, Vcap, tVboost, Vboost, tVreg, Vreg, tVinj, Vinj);
        legend('V_{cap}', 'V_{boost}', 'V_{reg}', 'V_{inj}');
        ylabel('V (V)');
    case 30
        plot(tVrect, Vrect, tVboost, Vboost, tVreg, Vreg, tVinj, Vinj);
        legend('V_{rect}', 'V_{boost}', 'V_{reg}', 'V_{inj}');
        ylabel('V (V)');
    case 31
        plot(tVcap, Vcap, tVrect, Vrect, tVboost, Vboost, tVreg, Vreg, tVinj, Vinj);
        legend('V_{cap}', 'V_{rect}', 'V_{boost}', 'V_{reg}', 'V_{inj}');
        ylabel('V (V)');
end

xlabel('Time (s)');
fclose(fp);