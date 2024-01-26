% Copyright (c) 1994 by the Mathworks
% $Revision: 1.1 $  $Date: 1994/05/09 15:44:42 $
%
print_num =0;
echo on;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	DEMO PROGRAM - CONTROL OF A FLUID CATALYTIC CRACKING UNIT


% The FCC model used is in state-space form in the file fccdat:
load fccdat

who

% [as,bs,cs,ds] is the state-space realization of the plant
% [ad,bd,cd,ddd] is the state-space realization of the measured disturbance


pause  % STRIKE ANY KEY TO CONTINUE
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                PLANT AND DISTURBANCE MODEL INFORMATION


% the plant is non-square. 
size(ds)
% it has 3 outputs and 2 inputs. The manipulated variables are:

%	vfg   - flue gas slide valve opening (%)
%	vlift - steam governor position on lift air compressor

% the controlled variables are:
%	- flue gas Oxygen concentration (mol %)
%	- riser temperature (deg. F)

% the associated variable is 
%	- lift air compressor surge indicator

% there are 3 modeled disturbances:
%	d1 - variations in ambient temperature (affects compressor efficiency)
%	d2 - variations in feed coking characteristics
%	d3 - reactor pressure variations due to down stream process upsets 

pause  % STRIKE ANY KEY TO CONTINUE
clc;

% look at the plant poles (these are the same as the poles of the disturbance 
% model since as = ad

format long e
eig(as)
format short

% all the poles (eigenvalues of as) are in the open LHP indicating a stable system. 

% Let us determine the step response model :


pause  % STRIKE ANY KEY TO CONTINUE
clc;

% Plant and disturbance step response models assuming a  STABLE SYSTEM

% the truncation time for the step response model is:
tfinal = 60000;   % time in s

% the time increment:
tinc = 200;

% plant model:
% Note - this calculation will take about 1min (on a SPARC 2)

plant = ss2step(as,bs,cs,ds,tfinal,0,tinc);

% examine step response plots for the plant :
figure
plotstep(plant);


% disturbance model:
% Note - this calculation will take about 1min (on a SPARC 2)

dist = ss2step(ad,bd,cd,ddd,tfinal,0,tinc);

% examine step response plots for the modeled disturbance :
figure
plotstep(dist);

clc;
% note that the process settling time is very large (1 day) for all outputs.
% for control, we are only interested in changes that occur on time scales
% of at most 2 hours.

% Let us examine in detail the effect  of the first manipulated 
% variable (vfg) on flue gas oxygen concentration (cO2sg). We will select
% only those rows and columns of the state-space matrices to yield a model 
% from vfg to cO2sg.

as_1 = as;
bs_1 = bs(:,1);
cs_1 = cs(1,:);
ds_1 = ds(1,1);

p11  = ss2step(as_1,bs_1,cs_1,ds_1,tfinal,0,tinc);
figure
plotstep(p11)

% rescale the plot to examine only the first 10000s :

axis([0 10000 -300 0])

% note that the response is very similar to what might be observed for an
% integrating system . 

% The plant and distubances will therefore be modeled as INTEGRATING systems.

pause  % STRIKE ANY KEY TO CONTINUE
clc;

echo off;
t = 0:tinc:tfinal;
subplot(121)
figure
plot(t,[0; p11(1:(max(size(p11))-3))]);
xlabel('time (s)')
ylabel('cO2sg')
title('A')
figure
subplot(122)
figure
plot(t,[0; p11(1:(max(size(p11))-3))]);
axis([0 10000 -300 0])
xlabel('time (s)')
ylabel('cO2sg')
title('B')

if print_num == 1
   print -deps mpc_w_fig1
end
clear as_1 bs_1 cs_1 ds_1 plant dist tfinal tinc
echo on;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	STEP RESPONSE MODEL DERIVATION - INTEGRATING SYSTEM


%	1) convert state-space model to step response model

% the truncation time for the step response model is:
tfinal = 6000;   % time in s

% the time increment is :
tinc = 100;

% convert the state-space models to step response models using ss2step:

pause  % STRIKE ANY KEY TO CONTINUE
clc;

help ss2step

% recall that we will model ALL the outputs as integrating outputs 
% ie nout = [0 0 0]

% determine the number of outputs and inputs:
siz = size(ds);
n_o = siz(1);
n_i = siz(2);
 
nout = zeros(n_o,1);

% plant model:
plant = ss2step(as,bs,cs,ds,tfinal,0,tinc,nout);

% NOTE: 6th input to the function ss2step is 0 - since the original data 
% is for a continuous time system.

% examine step response plots for the plant :
figure
plotstep(plant);

% No plant/model mismatch means that model = plant:
model = plant;

% disturbance model:
dist = ss2step(ad,bd,cd,ddd,tfinal,0,tinc,nout);

% examine step response plots for the modeled disturbance :
figure
plotstep(dist);


% We will now proceed to formulate the control problem ...


pause  % STRIKE ANY KEY TO CONTINUE
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                CONTROL PROBLEM FORMULATION

% INPUT WEIGHTS:
uwt = [0 2];

% OUTPUT WEIGHTS:
ywt = [2 2 0];

% Note that the output weight corresponding to the associated variable 
% (output #3) is 0 as it does not enter the objective function to be
% minimized

% MANIPULATED VARIABLE CONSTRAINTS:
usat = [-1 -1 1 1 1 1];

% prediction horizon:
pp = 12;

% number of manipulated variable moves:
mm = 3;

% the desired setpoint is:
set_pt = zeros(1,n_o);

% disturbance input :
dst = [0 -0.5 -0.75];

% simulation time :
tend = 2500;

% We will now proceed to determine the closed loop response ...

pause  % STRIKE ANY KEY TO CONTINUE
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                CONTROLLER DESIGN - UNCONSTRAINED CASE

% We will start off by assuming that there are no process constraints
% and examine closed loop performance.


% The unconstrained controller gain matrix will be derived using mpccon:

pause    % STRIKE ANY KEY TO CONTINUE
clc;

help mpccon


% implementation :


kmpc = mpccon(plant,ywt,uwt,mm,pp);


pause    % STRIKE ANY KEY TO CONTINUE
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%		CLOSED LOOP RESPONSE - UNCONSTRAINED CASE


% In this section, will examine closed loop response when process
% constraints are not explicitly accounted for.

% time intervals:

t = 0:tinc:tend;

% Closed loop response is determined using mpcsim:

pause    % STRIKE ANY KEY TO CONTINUE
clc;

help mpcsim



% implementation :


[y_u,u_u] = mpcsim(plant,model,kmpc,tend,set_pt,usat,[],dist,[],dst);

% plot the output and manipulated variable moves:

echo off;
y  = real(y_u);
u  = real(u_u);
a = [1250 1500];
b1= [1.4 1.4];
b2= [1 1];
b3= [0.6 0.6];
subplot(211)
figure
plot(t,y(:,1),t,y(:,2),'--',t,y(:,3),'-.',a,b1,a,b2,'--',a,b3,'-.')
xlabel('time (s)')
ylabel('output variables')

text(1530,1.4,'cO2sg')
text(1530,1.0,'Tr')
text(1530,0.6,'surge indicator')
title('Disturbance Rejection, UNCONSTRAINED DESIGN')

uu = mpcstair(u_u);
uu = uu(1:max(size(uu))-1,:);
tt = mpcstair(t');
tt = tt(2:max(size(tt)));

subplot(212)
plot(tt,uu(:,1),tt,uu(:,2),'--')
xlabel('time (s)')
ylabel('manipulated variables')
text(1000,0.15,'V_fg')
text(1060,-0.55,'V_lift')

if print_num == 1
   print -deps mpc_w_fig2
end

clear aao bbo cco ddo b1 b2 b3 a kmpc nout siz u y y_lb1;
clear y_lb2 y_ub1 y_ub2 ;
echo on;


pause    % STRIKE ANY KEY TO CONTINUE
clc;
% Note that the controlled variables and associated variable exceeds 
% their saturation limit. The lift air compressor surges during the 
% disturbance transient and this is unacceptable from an operational 
% point of view. 

% The control law must therefore be designed to take the constraints 
% into account

% specify output variable saturation limits:
echo off;
y_lb1 = [-1 -1 -1];
y_lb2 = [-inf -inf -inf ];
y_ub1 = [1  1 inf];
y_ub2 = [inf inf inf];
y_lb  = [y_lb1;y_lb1;y_lb2];
y_ub  = [y_ub1;y_ub1;y_ub2];
echo on;

ysat  = [y_lb y_ub]



% We will examine the closed loop response when process constraints are
% explicitly accounted for in controller design.

% the program "cmpc" achieves this.

pause    % STRIKE ANY KEY TO CONTINUE
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%		CLOSED LOOP RESPONSE - CONSTRAINED CASE


% In this section, will examine closed loop response when process
% constraints are explicitly accounted for.

% Note that all the design parameters like input and output weights are 
% identical to the ones used for the unconstrained case just examined.




pause    % STRIKE ANY KEY TO CONTINUE
clc;

help cmpc





% implementation

[y_c,u_c,ym] = cmpc(plant,model,ywt,uwt,mm,pp,tend,set_pt,usat,ysat,[],dist,[],dst);


% plot the closed loop response:
echo off;
y  = real(y_c);
u  = real(u_c);
a = [1250 1500];
b1= [1.4 1.4];
b2= [1 1];
b3= [0.6 0.6];
subplot(211)
figure
plot(t,y(:,1),t,y(:,2),'--',t,y(:,3),'-.',a,b1,a,b2,'--',a,b3,'-.')
xlabel('time (s)')
ylabel('output variables')
title('Disturbance Rejection, CONSTRAINED DESIGN')
text(1530,1.4,'cO2sg')
text(1530,1.0,'Tr')
text(1530,0.6,'surge indicator')

uu = mpcstair(u_u);
uu = uu(1:max(size(uu))-1,:);
tt = mpcstair(t');
tt = tt(2:max(size(tt)));

subplot(212)
figure
plot(tt,uu(:,1),tt,uu(:,2),'--')
xlabel('time (s)')
ylabel('manipulated variables')
text(1000,0.1,'V_fg')
text(1000,-0.60,'V_lift')

if print_num == 1
   print -deps mpc_w_fig3
end
echo on;

% Note that the lift air compressor does not surge during the process
% transient and the overall operation is safe.




pause    % STRIKE ANY KEY TO CONTINUE
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%		COMPARE CONSTRAINED AND UNCONSTRAINED RESPONSE
%			IN THE ASSOCIATED VARIABLE:


% Plot on the same graph the response of the surge indicator for both
% the constrained case and the unconstrained case.

echo off;
a = [1250 1500];
b1= [-0.2 -0.2];
b2= [-0.4 -0.4];
figure
plot(t,y_u(:,3),t,y_c(:,3),'--',a,b1,a,b2,'--')

text(1530,-0.2,'Unconstrained Response')
text(1530,-0.4,'Constrained Response')
xlabel('time (s)')
ylabel('associated variable')
title('Associated Variable Response to Constrained & Unconstrained Control Laws')

if print_num == 1
   print -deps mpc_w_fig4
end
echo on;

%	*************	END FCC DEMO	*************	

echo off;


