clear all
clc

% Pendolo param
L = 2;          % lunghezza [m]
Km = 1;         % Coef Acc Mag
Kattr = 0.1;    % Attritto viscoso sulla velocità angolare
degStart = 50;  % Angolo di partenza [deg]


% Sim param
frameRate=20; % Frame per Secondo [1/s]
simTime = 20; % Secondi di simulazione [s]
t_span = 0:1/frameRate:simTime;


% Solving ODE
y0 = [deg2rad(degStart),0]; % Start Condition (Theta, Omega)
[t_sim,sol] = ode45(@(t,y) pend_fn(t,y, [L, Kattr, Km]),t_span,y0);

% Display Sim
U = ones(length(t_sim),1);
displaySystem(t_sim, frameRate,sol,U,[L,Km]);
