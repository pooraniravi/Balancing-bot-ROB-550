% Balancebot SS Model and LQR Control design
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



%% statespace model including motor dynamics
a1 = I_w + (m_b + m_w)*R_w^2;
a2 = m_b * R_w * L;
a4 = I_r + m_b*L^2;
a4 = m_b * c1 * L;

% motor equation used: t = e*u - f*w
b1 = 2 * tau_s * R_gb; % stall torque of two motors
b2  = b1 / (w_free/R_gb);   % constant provides zero torque @ free run

%simplifications
c1 = 1 - (a2^2)/(a1*a4);
c2 = 1 + (a2/a4);
c3 = 1 + (a2/a1);

%theta
%thetadot
%phi
%phidot

%% create statespace matricies
A = [0 1 0 0;
     a4/(a4*c1) -(b2*c3)/(a4*c1) 0 (b2*c3)/(a4*c1);
     0 0 0 1;
     -(a2*a4)/(a1*a4*c1) (b2*c2)/(a1*c1) 0 -(b2*c2)/(a1*c1)];
B = [0 -(b1*c3)/(a4*c1) 0 (b1*c2)/(a1*c1) ]';
C = [1 0 0 0;
     0 0 1 0];
D = [0 0]';

states = {'theta' 'theta_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'theta'; 'phi'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_tf = tf(sys_ss)
% check poles and controllability of model
poles = eig(A)
controllability = rank(ctrb(sys_ss))

%%
%LQR Design
%Tune Your Gains Here and see the response
Q = [10 0 0 0
     0 0.1 0 0
     0 0 10 0
     0 0 0 0.8];
R = 1.0;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

% find Nbar for precompensator
Cn = [0 0 1 0];
s = size(A,1);
Z = [zeros([1,s]) 1];
N = inv([A,B;Cn,0])*Z';
Nx = N(1:s);
Nu = N(1+s);
Nbar=Nu + K*Nx

sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs)

t = 0:0.01:4;
dist = 0.1; % move balancebot 10cm
angle = (dist/R_w)*ones(size(t));
angle(1) = 0;
[y,t,x]=lsim(sys_cl,angle,t);
figure
[AX,H1,H2] = plotyy(t,y(:,1),t,R_w*y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','body angle (radians)')
set(get(AX(2),'Ylabel'),'String','body position (meters)')
title('Step Response with LQR Control and Precompensator')

%% find discrete time system
sys_d = c2d(sys_ss,DT,'zoh')
A_d = sys_d.a;
B_d = sys_d.b;
C_d = sys_d.c;
D_d = sys_d.d;
[K_d] = dlqr(A_d,B_d,Q,R)

%% discrete time closed loop
A_dc = [(A_d-B_d*K_d)];
B_dc = [B_d];
C_dc = [C_d];
D_dc = [D_d];

Nbar_d = -0.11 % adjust Nbar for steadystate performance

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'phi'};
outputs = {'x'; 'phi'};
sys_dcl = ss(A_dc,B_dc*Nbar_d,C_dc,D_dc,DT,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:4;
dist = 0.1; % move balancebot 10cm
angle = (dist/R_w)*ones(size(t));
[y,t,x]=lsim(sys_dcl,angle,t);
figure
[AX,H1,H2] = plotyy(t,y(:,1),t,R_w*y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','body angle (radians)')
set(get(AX(2),'Ylabel'),'String','body position (meters)')
title('Discrete Step Response with LQR Control and Precompensator')

