function [VV,WW,RI,firstN,T,Ticks] = loadMeasurements(dataList, R, polarity, start)
%LOAD_MEASUREMENTS load measurements from a cell array describing a dataset
%   Each data set is composed of CSV files with a voltage run at for that 

if nargin < 4
  start = 4;
end

rotPerTick = 1/(20.4 * 48);
radPerTick = rotPerTick * 2*pi;

Ticks = [];
VV = [];
WW = [];
RI = [];
T = [];
for i= 1:size(dataList,1)
    D = csvread(strcat('../', dataList{i,2}));
    % discard some initial data points because they are more noisy
    D = D(start:end, :);
    % voltage
    V = dataList{i,1};
    % number of data points
    N = size(D, 1);
    if i == 1
        firstN = N;
    end
    TT = D(:,1)/1e6;
    VV = [VV; ones(N,1)*V];
    
    ticks = D(:,2) .* polarity;
    WW = [WW; ticks./TT .* radPerTick];
    RI = [RI; D(:,3).*R];
    T = [T; TT];
    Ticks = [Ticks; ticks];
end

end

