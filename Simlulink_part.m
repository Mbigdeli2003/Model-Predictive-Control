clc
close all
open_system('mpc_papermachine')
mpc1
mpc_pmmodel;
sim('mpc_papermachine');
figure(1);
plot(time,Y(:,1),'r','linewidth',3.5);hold on
plot(time,Y(:,2),'g--','linewidth',3);hold on;
plot(time,Y(:,3),'b--','linewidth',2);grid on
legend('Y1','Y2','Y3')
figure(2);

plot(1:length(MV_s),MV_s(:,2),'b--','linewidth',2), hold on;
plot(1:length(MV_s),MV_s(:,1),'r','linewidth',2);grid on
legend('U2','U1');grid on
