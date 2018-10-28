%% estimate K and b
rotPerTick = 1/(20.4 * 48);
radPerTick = rotPerTick * 2*pi;
RL = 5.4;
RR = 5.6;

%%
% always strip the first few values because they have high relative noise
leftData = {11.67, 'data/left.1.csv'
10.74 , 'data/left.0.8.csv'
8.98 , 'data/left.0.6.csv'
7.80 , 'data/left.0.5.csv'};
rightData = {6.29 , 'data/right.0.5.csv'
7.65 , 'data/right.0.6.csv'
10.15 , 'data/right.0.8.csv'
11.67 , 'data/right.1.csv'};
rightData = flipud(rightData);


[VL, WL, RIL, maxNL, ~, ~] = loadMeasurements(leftData, RL, -1);
[VR, WR, RIR, maxNR, ~, ~] = loadMeasurements(rightData, RR, 1);

%% put data into the right form

% V - RI = WK
YL = VL - RIL;
YR = VR - RIR;

%% start estimating
KL = WL \ YL;
% estimate variance
YhatL = WL .* KL;
EL = YL - YhatL;
% standard error of regression
sL = sqrt(EL'*EL/(size(EL,1)-1));

KR = WR \ YR;
YhatR = WR .* KR;
ER = YR - YhatR;
sR = sqrt(ER'*ER/(size(ER,1)-1));

fprintf('Left K: %f +/- %f\n', KL, sL);
fprintf('Right K: %f +/- %f\n', KR, sR);

%% steady state noload
LNL = WL(1:maxNL);
LNR = WR(1:maxNR);
wNLL = mean(LNL);
wNLR = mean(LNR);
fprintf('Left w_NL: %f +/- %f\n', wNLL, std(LNL));
fprintf('Right w_NL: %f +/- %f\n', wNLR, std(LNR));

%% friction coefficient
bL = (KL*VL(1)/wNLL - KL^2)/RL;
bLmin = ((KL+sL)*VL(1)/wNLL - (KL+sL)^2)/RL;
bLmax = ((KL-sL)*VL(1)/wNLL - (KL-sL)^2)/RL;

bR = (KR*VR(1)/wNLR - KR^2)/RR;
bRmin = ((KR+sR)*VR(1)/wNLR - (KR+sR)^2)/RR;
bRmax = ((KR-sR)*VR(1)/wNLR - (KR-sR)^2)/RR;
fprintf('Left b: %f +/- %f\n', bL, max(bLmax-bL, bL-bLmin));
fprintf('Right b: %f +/- %f\n', bR, max(bRmax-bR, bR-bRmin));

%% estimate J
leftData = {0, 'data/leftJ.csv'};
rightData = {0 , 'data/rightJ.csv'};
[~, ~, RIL, ~, TL, TicksL] = loadMeasurements(leftData, RL, -1, 2);
[~, ~, RIR, ~, TR, TicksR] = loadMeasurements(rightData, RR, 1, 2);

% instantaneous angular velocity
WL = diff(TicksL) ./ diff(TL) .* radPerTick;
WdotL = diff(WL) ./ diff(TL(1:end-1));

YL = -WL(1:end-1).*bL;
JL = WdotL \ YL;
YhatL = WdotL .* JL;
EL = YL - YhatL;
% standard error of regression
sL = sqrt(EL'*EL/(size(EL,1)-1));

WR = diff(TicksR) ./ diff(TR) .* radPerTick;
WdotR = diff(WR) ./ diff(TR(1:end-1));
YR = -WR(1:end-1).*bR;

JR = WdotR \ YR;
YhatR = WdotR .* JR;
ER = YR - YhatR;
% standard error of regression
sR = sqrt(ER'*ER/(size(ER,1)-1));
fprintf('Left J: %f +/- %f\n', JL, sL);
fprintf('Right J: %f +/- %f\n', JR, sR);