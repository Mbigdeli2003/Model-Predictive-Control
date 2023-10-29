clear
close all
clc
global u_store
%% Defining Coninous and diceret system
% Contionus system
num=0.9; % Num of the tf
den=[0.2 1 ]; %den of tf
Sys_C=tf(num,den,'inputdelay',0.1) % Contionus System

%Discrete system
Tz=0.05;
Sys_D=c2d(Sys_C,Tz,'foh')

%% Step Response of the system for DMC rule
spl=0.01; %Sampling steps
Ts=0:spl:1.6;
[g_D,Td]=step(Sys_D);
[g,t]=step(Sys_C,Ts);
% Plotting step Response of the Systems
subplot(2,1,1);
plot(t,g,'linewidth',2);
title('Step Response of Continous System','Color','b');
xlabel('time','color','b');
ylabel('Amplitude','color','b');
grid on
subplot(2,1,2);
plot(Td,g_D,'ro','linewidth',2);
title('Step Response of Discrete System','Color','r');
xlabel('time sample', 'color','r');
ylabel('Amplitude','color','r');
grid on
%% Designing and Applying DMC Rule
figure(2)
p=40; %% Predict Horizon
m=15;  %%% Control Horizon
g1=g(1:p);
G=zeros(p,m);

%%control Parameters
 alfa=0.1;
 landa=0.8;
for j=1:m
   G(:,j)=[zeros(j-1,1) ; g1(1:p-j+1,:)];
end 
 y0=0;
 u_store=zeros(140,1);
 f=zeros(p,1);
 w=ones(p,1);
  for Count=1:155;
     for k=1:p
     %w(k,1)=1-(alfa)^(k)*(y0-1);    
     w(k,1)=alfa*w(k,1)+(1-alfa)*y0;
     Sigma=0;
       for i=1:120  %%% stable Response for sys after a20 steps 
         Delta_G=g(i+k)-g(i);
         Delta_U=u_store(i)-u_store(i+1);
         Sigma=Delta_G*Delta_U+Sigma;
       end
     f(k)=y0+Sigma;
     
   end

delta_u=inv(G'*G+landa*eye(m,m))*G'*(w-f);
u_new=delta_u(1)+u_store(1);
U=u_new;
u_store=[u_new;u_store];
u_store=u_store(1:140);
tspan=[t( Count) t( Count+1) t(Count+2)] ;
[tt,y1]=ode45(@mpc_diff,tspan,y0);
y0=y1(2); % Updating estimated y
hold on
plot(t(Count+1),y0,'b.');
title('Estimated Y','color','b');
xlabel('Time Step','color','r');
ylabel('Amplitude','color','r')
grid on
grid on
  end
%plot u_store  
figure(3);
plot(t(1:140),u_store,'r.');
title('Control Effort','color','b');
xlabel('Time Step','color','r');
ylabel('Amplitude of Controler','color','r')
 grid on
  



