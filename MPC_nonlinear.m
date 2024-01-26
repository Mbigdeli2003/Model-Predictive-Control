%% create MPC controller object with sample time
mpc1 = mpc(mpc1_plant_C_1, 0.25);
%% specify prediction horizon
mpc1.PredictionHorizon = 10;
%% specify control horizon
mpc1.ControlHorizon = 2;
%% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = [0;0;0;0];
mpc1.Model.Nominal.Y = [0;0;0];
%% specify constraints for MV and MV Rate
mpc1.MV(1).Min = -10;
mpc1.MV(1).Max = 10;
mpc1.MV(2).Min = -10;
mpc1.MV(2).Max = 10;
%% specify weights
mpc1.Weights.MV = [0 0];
mpc1.Weights.MVRate = [0.4 0.4];
mpc1.Weights.OV = [0.2 0 1];
mpc1.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'on';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc1, 121, mpc1_RefSignal_5, mpc1_MDSignal_5, options);
disp('Press any key to continue...');
pause(1)
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc1, 121, mpc1_RefSignal_6, mpc1_MDSignal_6, options);
disp('Press any key to continue...');
pause(1)
%% specify simulation options
options = mpcsimopt();
options.UnmeasuredDisturbance = mpc1_UDSignal_3;
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc1, 121, mpc1_RefSignal_7, mpc1_MDSignal_7, options);
disp('Press any key to continue...');
pause(1)
%% specify simulation options
options = mpcsimopt();
options.UnmeasuredDisturbance = mpc1_UDSignal_4;
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc1, 121, mpc1_RefSignal_8, mpc1_MDSignal_8, options);
disp('Press any key to continue...');
pause(1)
%% specify simulation options
options = mpcsimopt();
options.UnmeasuredDisturbance = mpc1_UDSignal_5;
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc1, 121, mpc1_RefSignal_9, mpc1_MDSignal_9, options);
