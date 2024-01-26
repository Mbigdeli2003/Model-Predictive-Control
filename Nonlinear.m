clc
clear 
close all
edit mpc_pmmodel;
A = [-1.9300        0        0        0
      0.3940  -0.4260        0        0
           0        0  -0.6300        0
      0.8200  -0.7840   0.4130  -0.4260];
B = [1.2740   1.2740        0        0
          0        0        0        0
     1.3400  -0.6500   0.2030   0.4060
          0        0        0        0];
C = [0   1.0000        0        0
     0        0   1.0000        0
     0        0        0   1.0000];
D = zeros(3,4);
PaperMach = ss(A,B,C,D);
PaperMach.InputName = {'G_p','G_w','N_p','N_w'};
PaperMach.OutputName = {'H_2','N_1','N_2'};
PaperMach.TimeUnit = 'minutes';

step(PaperMach)
mpcDesigner
%open_system('mpc_papermachine')
