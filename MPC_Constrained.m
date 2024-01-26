clc
clear
close all
%
A = [-1.93 0 0 0; .394 -.426 0 0; 0 0 -.63 0; .82 -.784 .413 -.426]; 
B = [1.274 1.274 0 0;0 0 0 0;1.34 -.65 .203 .406;0 0 0 0];
C = [0 1 0 0; 0 0 1 0; 0 0 0 1];
D = zeros(3,4);
% Discretize the linear model and save in mod
dt = 2;
[PHI,GAM] = c2dmp(A,B,dt);
minfo = [dt,4,2,1,1,3,0];
imod = ss2mod(PHI,GAM,C,D,minfo);
plotstep(mod2step(imod,30));

% Define controller parameters 
P = 10; % Prediction horizon
M = 3; % Control horizon
ywt = [1,0,1]; % Equal weighting of y(1) and y(3),
% no control of y(2) 
uwt = 0.6*[1 1]; % Equal weighting of u(1) and u(2)
ulim = [-10*[1 1] 10*[1 1] 2*[1 1]];% Constraints on
ylim = [ ]; % No constraints on y
Kest = [ ]; % Use default estimator 

% Simulation using scmpc -- no model error 
pmod=imod; % plant and internal model are identical
setpts = [1 0 0];
% servo response to step in y(1) setpoint
tend = 30; % duration of simulation
figure(1)
[y,u,ym] = scmpc(pmod,imod,ywt,uwt,M,P,tend,setpts,ulim,ylim,Kest);
plotall(y,u,dt)

setpts = [0 0 1]; % servo response to step in y(3) setpoint
figure(2)
[y,u,ym] = scmpc(pmod,imod,ywt,uwt,M,P,tend,setpts,ulim,ylim,Kest);
plotall(y,u,dt) 

ywt = [0.2,0,1]; % Unequal weighting of y(1) and y(3),
% no control of y(2)
setpts = [1 0 0];
% servo response to step in y(1) setpoint
figure(3)
[y,u,ym] = scmpc(pmod,imod,ywt,uwt,M,P,tend,setpts,ulim,ylim,Kest);
plotall(y,u,dt)

setpts = [0 0 0]; % output
%z=[zeros(length(setpts),2) ones(length(setpts),1)]
z=[];
%z=[1 zeros(length(setpts)-1,1)] % measurement noise
figure(4)
v = 1; % measured disturbance
d = 0; % unmeasured disturbance
[y,u,ym] =scmpc(pmod,imod,ywt,uwt,M,P,tend,...
setpts,ulim,ylim,Kest,z,v,d);
plotall(y,u,dt)

setpts = [0 0 0]; % output setpoints
v = 0; % measured disturbance
d = 1; % unmeasured disturbance
figure(5)
[y,u,ym] = scmpc(pmod,imod,ywt,uwt,M,P,tend,setpts,ulim,ylim,Kest,z,v,d);
plotall(y,u,dt)

% Estimator design
Q = 30;
R = 1*eye(3);
Kest = smpcest(imod,Q,R)
% Simulation using scmpc -- no model error
setpts = [0 0 0]; % servo response to step in y1 setpoint
d = 1; % unmeasured disturbance
figure(6)
[y,u,ym] = scmpc(pmod,imod,ywt,uwt,M,P,tend,...
setpts,ulim,ylim,Kest,z,v,d);
plotall(y,u,dt)

% Alternative estimator design -- output disturbances
taus = [5 5 5];
signoise = [10 10 10];
[Kest, newmod] = smpcest(imod,taus,signoise);
% Simulation using scmpc -- no model error 
figure(7)
[y,u,ym] = scmpc(pmod,newmod,ywt,uwt,M,P,tend,...
setpts,ulim,ylim,Kest,z,v,d);
plotall(y,u,dt)
