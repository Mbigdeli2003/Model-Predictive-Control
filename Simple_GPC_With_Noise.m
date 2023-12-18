clc
clear
close all

%% Curve Reaction for TF estimation
clc
clear
close all
fig = openfig('Step_Response.fig');
all_ax = findobj(fig, 'type', 'axes');
% all_titles = cellfun(@(T) T.String, get(all_ax, 'title'), 'uniform', 0);
 all_lines = arrayfun(@(A) findobj(A, 'type', 'line'), all_ax, 'uniform', 0);
 all_XData = cellfun(@(L) get(L,'XData'), all_lines, 'uniform', 0);
 all_YData = cellfun(@(L) get(L,'YData'), all_lines, 'uniform', 0);
t=all_XData{1,1}{3,1}(1:end-1);
y=all_YData{1,1}{3,1}(1:end-1);
%%Curve Reaction process
[~,t2_idx] = (min(abs(y - (63.2/100)*max(y))))
[~,t1_idx] = (min(abs(y - (28.3/100)*max(y))))
Tau = 1.5*abs(t(t1_idx)-t(t2_idx))
%slope = gradient(y,t);
Tau_d = 1.5*(t(t1_idx)-(1/3*t(t2_idx)))
slope=(y(t1_idx)-0)/(t(t1_idx)-Tau_d)
%[tslope,idx] =slope;
tIP = t(t1_idx);
yIP = y(t1_idx);
yTangentLine = slope*(t-tIP) + yIP;
hold on
plot(t(t1_idx),y(t1_idx),'r*')
plot(t(t2_idx),y(t2_idx),'r*')
plot(Tau+Tau_d+0.1,0,'r*')
plot(t,yTangentLine,'r')
ylim([0 2]);
% gain
K = y(end)/1;

plot(Tau_d,0,'g.','MarkerSize',15)
% Define the variable 's' for the continuous time transfer function
s = tf('s');
% Construct the transfer function
GA = ((K * exp(-s*Tau_d))/(1+Tau*s)) % G(s) = (K*(e^(-s*Td))/(1+Ti*S))
%GA = tf(K,[Tau 1],'InputDelay',Tau_d)
[z,t] = step(GA);
plot((slope))
plot(t,z,'k--')
xline(Tau_d)
xline(Tau_d+Tau)
xline(t(t2_idx))
xline(t(t1_idx))
yline((28.3/100)*max(y))
yline((63.2/100)*max(y))

legend('Actual Process','InflectionPoint 28.3%','InflectionPoint 63.2%','Tau+Tau_d','Tangent at IP','Dead Time','Approximated FOPTD','Location','best')

Num=GA.Numerator{1,1};
Den=GA.Denominator{1,1};
%% Discrete system by Camacho Formulation
T=0.5;
a=exp(-T/Tau_d)
b=K*(1-a)
d=floor(Tau_d/T)
G=tf([0 b],[1 -a],T,'InputDelay',d)

%% Controller Implemenation
%% Lambda config
lambda=[0 0.2 0.5 0.7 1 1.5]
%kk=length(lambda)
%k11=zeros(kk,kk);
for i =  2:length(lambda)
    
    k11(i) = -exp(0.3598-0.9127*lambda(i)+0.3165*(lambda(i))^2);
    k21(i) = -exp(0.0875-1.2309*lambda(i)+0.5086*(lambda(i))^2);
    k31(i) = 1.05;
    k12(i) = exp(-1.7383-0.40403*lambda(i));
    k22(i) = exp(-0.32157-0.8192*lambda(i)+0.3109*(lambda(i))^2);
    k32(i) = 1.045;
    
    ly1(i) = k11(i) + k21(i)*(a/(k31(i)-a));
    ly2(i) = k12(i) + k22(i)*(a/(k32(i)-a));
    lr1(i) = - ly1(i) - ly2(i);
    
    %%
    T0 = 0 ;
    Tf = 30 ;
    Ts = 0.5 ;
    t = T0:Ts:Tf ; % From T0 to Final time of simulation with step of Ts
    
    Nt = numel(t) ;
    du = zeros(size(t)) ;
    u = zeros(size(t))  ;
    y = zeros(size(t))  ;
    r = [ones(fix(Nt/2)+5 , 1); -ones(fix(Nt/2)+5 , 1)] ;
    
    for j = 2:Nt-1
        du(j) = (ly1(i) * y(j+d-1) + ly2(i) * y(j+d-2) + lr1(i)*r(j))/K;
        u(j) = u(j-1) + du(j) ;
        y(j+1) = (1+a) * y(j+d-1)-a*y(j+d-2)+b*du(j);
        if (j>=fix(Nt/2)-20) && (j<=fix(Nt/2)-15) 
         y(j+1) = (1+a) * y(j+d-1)-a*y(j+d-2)+b*du(j)+0.1*rand(1,1);
        end
        if (j>=fix(Nt/2)+10) && (j<=fix(Nt/2)+15) 
         y(j+1) = (1+a) * y(j+d-1)-a*y(j+d-2)+b*du(j)+0.1*rand(1,1);
        end
    end
    
    %% plot results %%
    u(end) = u(end-1) ;
    figure(i);
%     str = sprintf('just an example of %lambda(i) that isnt working', lambda(i))
%     title(str)
    %title('Lambda is ',num2str(lambda(i)))
    subplot(2,2,[1 2]) ;
    plot( t , r(1:Nt) , t , y , 'LineWidth' , 2) ;
     %title(['Lambda is ',])
    xlabel('Time  (second)') ;
    ylabel('Amplitude  y') ;
    title('Simple GPC algorithm, Lambda is =', num2str(lambda(i)),'Color','red') ;
    %legned('y','y_hat')
    grid on
    
    figure(i) ;
    subplot(2,2,3) ;
    plot( t , du , 'LineWidth' , 2) ;
    xlabel('Time  (second)') ;
    ylabel('Amplitude  du') ;
    title('Incrementing Control Effort') ;
    grid on
    
    
    figure(i) ;
    subplot(2,2,4) ;
    plot( t , u , 'LineWidth' , 2) ;
    xlabel('Time  (second)') ;
    ylabel('Amplitude  U') ;
    title('Control Effort') ;
    grid on


end
%% plotting Controller Parameters
figure(length(lambda)+1)

plot(ly1(2:end),'LineWidth' , 2)
title('Controller Paramenetrs')
hold on
plot(ly2(2:end),'LineWidth' , 2)
hold on
plot(lr1(2:end),'LineWidth' , 2)
legend('ly_1','ly_2','l_r')
grid on

figure(length(lambda)+2)

plot(k11(2:end),'LineWidth' , 2)
title('Controller Paramenetrs')
hold on
plot(k12(2:end),'LineWidth' , 2)
hold on
plot(k21(2:end),'LineWidth' , 2)
hold on
plot(k22(2:end),'LineWidth' , 2)
legend('k11','k12','k21','k22')
grid on

