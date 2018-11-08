% Balancebot TF Model
% lead-lag controllers
% root locus design method
clc
close all
clear 

%% constants
DT      = .01;          % 100hz controller loop
m_w     = .03;          % mass of one wheel in Kg MEASURED
m_b     = 1.0;          % balancebot body mass without wheels (TO BE DETERMINED)
R       = .04;          % radius of wheel in m MEASURED
L       = .100;         % center of wheel to Center of mass (TO BE DETERMINED)
I_r     = 0.004;        % Inertia of body about center (not wheel axis) Kg*m^2 (TO BE DETERMINED)
g       = 9.81;         % gravity m/s^2
R_gb    = 20.4;         % gearbox ratio
tau_s   = 0.50;         % Motor output stall Torque @ V_nominal (TO BE DETERMINED)
w_free  = 50;           % Motor output free run speed @ V_nominal (TO BE DETERMINED)
V_n     = 12.0;         % motor nominal drive voltage
I_gb = 100.0*10^-5;     % inertial of motor armature and gearbox (TO BE DETERMINED)

% add inertia of wheels modeled as disks and times two for both sides
I_w = 2 * (I_gb+(m_w*R^2)/2);



%% inner loop plant including motor dynamics
a1 = I_w + (m_b + m_w)*R^2;
a2 = m_b * R * L;
a3 = I_r + m_b*L^2;
a4 = m_b * g * L;

% motor model equation used: t = e*u - f*w
b1 = 2 * tau_s; % stall torque of two motors
b2  = b1 / (w_free);   % constant provides zero torque @ free run

numG1 = [-b1*(a1+a2), 0];
denG1 = [(a1*a3 - a2^2), b2*(a1+a3+2*a2), -a1*a4, -a4*b2];
% Make TF monic 
numG1 = (1/denG1(1))*numG1; 
denG1 = (1/denG1(1))*denG1;
%make the TF
G1 = tf(numG1,denG1)
disp('G1 poles')
roots(denG1)
disp('G1 zeros')
roots(numG1)

%% Plot of poles/zeros show an unstable system
% show desired wc and zeta
figure
rlocus(G1)

% The root locus shows an unstable pole in the RHP and a zero
% at the origin.  We want a controller that cancels this zero and 
% moves the RHP pole to the LHP so we get our desired crossover frequency 
% and damping ratio.  Note the gain must be negative to move that RHP pole
% back into the LHP.

%% Inner Loop Controller
tr1 = 0.1; % design parameter - desired rise time of inner loop
wc1 = 1.8/tr1; % rule of thumb crossover freq
zeta1 = 0.5 % design parameter - damping ratio

% adjust the position of the pole and zero of each controller
% to position the dominant poles with gain
% note this may end up looking like a 3rd order system if the plant pole 
% near -20 moves too far to the right, but this is OK you will just have a 
% slightly longer settling time

lagD1  = tf([1, 20],[1, 0]); % lag controller with pole at origin to cancel zero
leadD1 = tf([1, 8],[1, 60]); % lead controller
K1 = -5; % my choice of gain, note negative gain

D1 = K1*lagD1*leadD1; % controller is lead lag with gain K1
OL1 = minreal(D1*G1); % open loop system
CL1 = feedback(OL1,1); % apply feedback

% show the closed loop root-locus plot.  You can select the gain by
% clicking on the graph and finding the location where you meet the design 
% requirements
figure
rlocus(-1*lagD1*leadD1*G1)
sgrid(zeta1,wc1)
figure
step(CL1)

%% outer loop plant
numG2 = [-(a2+a3), 0, a4];
denG2 = [a1+a2, 0,0];
% make monic
numG2 = (1/denG2(1))*numG2;
denG2 = (1/denG2(1))*denG2;
% make the TF
G2=tf(numG2,denG2)
disp('G2 poles')
roots(denG2)
disp('G2 zeros')
roots(numG2)
figure
rlocus(G2)

%% Outer loop position controller
tr2 = 0.8; % design parameter - desired rise time of outer loop
wc2 = 1.8/tr2;
zeta2 = 0.7; % design parameter - damping ratio
leadD2 = tf([1,.15],[1,5]);
lagD2  = tf([1,1.5],[1, 0]); 
K2 = 0.1;
D2 = K2*leadD2*lagD2;
figure
rlocus(leadD2*lagD2*G2)
sgrid(zeta2,wc2)

%% compare closed loops
% ideal closed loop response with perfect inner loop
CL2S = feedback(minreal(D2*G2),1);
figure 
hold on;
step(CL2S)
% full closed loop response with actual inner loop
CL2 = feedback(minreal(D2*CL1*G2),1);
step(CL2)
hold off;

%% find discrete controller for inner loop
D1z = c2d(D1,DT,'tustin')

%% find discrete controller for outer loop
D2z = c2d(D2,DT,'tustin')
