
 clc
 clear;
 close all;
% 
% %% Defining System
% %Gneral Equiation fo Systems: A(z^-1)*y(t)=z^-d*B(Z^-1)*u(t-1)+c(z^-1)e(t)
% % A(z^-1)=1+a_1*z^-1+a_2*z^-2+....
% %B(z^-1)=b_0+b_1*z^-1+b_2*z^-2+....
% %C(z^-1)=1+c_1*(z^-1)+c_2(z^-2)+...
% 
% %First Order System_Best peridction without noise and Delay
% 
% % A Coeficeints
 a_1=-0.7;
% %B coeficinets
b_0=1.5;
b_1=1.5;

%Colored Noise Coeffs
C=[0.01 0.2 0.03];
t=500;
varN=0.1;% noise variance
r=random('normal',0,varN,t,1);
noise_minor=[zeros(1,3) r']';
%Matrix A and B for definig System
 A=[1 a_1];
 B=[b_0 b_1];
 n_a=length(A);
 n_b=length(B);
 zi = filter(B,A,0);
%% building A_tild for diophantine equation
 delta=[1 -1]; % delta term coeffs
 A_tild_1=zeros(length(A)+1,length(A)+1); % considering matrix with oprator Z as delays in coeffs
 for i=1:length(A_tild_1)
A_1=[A 0];
     A_tild_1(i,i:i+1)=A_1(i).*delta;
 end
 A_tild_2=sum(A_tild_1);
 A_tild=A_tild_2(1:length(A)+1); % obtaining A_tild for diophantine Equation solving

 %% Diophantine Equtaion 1/A_tild 
%Function Input
d=100;
n_1=d+1;
n_2=5;    %Prediction Horizon  
n_u=5;   %Controller Horizon based on camacho assumption for control horizon
%making matrix with delay oprator Z for divide in diohantine 
A_ph=zeros(length(A_tild),length(A_tild));
for i=1:length(A_tild)
    for j=1:length(A_tild)
        if i==j
A_ph(i,j)=A_tild(i);
        end
    end
end
% % A_phi=cell(n_2+2,n_2+2);
% % for i=1:length(size(A_phi))
% %     A_phi{i,1}=[ A_ph
  A_ph=[ A_ph zeros(max(size(A_ph)),n_2-1);];
  A_ph=[A_ph; zeros(n_2-1,max(size(A_ph)))];
 %Numerator
NUM=cell(n_2+2,1);
for i=1:length(size(NUM))
NUM{i}=zeros(n_2+2,n_2+2);
end

%Remainder for F
RE_1=cell(n_2,1);
for i=1:max(size(RE_1))
RE_1{i,1}=zeros(n_2+2,n_2+2);
end
%Remainder
RE_2=cell(n_2,1);
for i=1:max(size(RE_2))
RE_2{i,1}=zeros(n_2+2,n_2+2);
end

% F Cell
F=cell(n_2,1);
for i=1:max(size(F))
    F{i,1}=zeros(n_2,n_2);
end

%Qoutinet for E
QQ=cell(n_2,1);
for i=1:max(size(QQ))
QQ{i,1}=zeros(n_2+2,n_2+2);
end
%E Cell 
E=cell(n_2,1);
for i=1:max(size(E))
    E{i,1}=zeros(n_2+2,n_2+2);
end

NUM{1,1}(1,1)=A_ph(1);

 for i=1:n_2
%    k=k+1 
    QQ{i,1}(i,i)=NUM{i,1}(i,i)./A(1,1);
    A_ph_1=[zeros(n_2+2,i-1) A_ph(:,1:end+1-i)];
    A_ph_2=[zeros(i-1,n_2+2); A_ph_1(1:end+1-i,:)];
    RE_1{i,1}=QQ{i,1}(i,i).*A_ph_2;  
    RE_2{i,1}=NUM{i,1}-RE_1{i,1}; % Remainder of each step for E
    NUM{i+1,1}=RE_2{i,1}; 
   
  E{i,1}=QQ{i,1}; %% Building E for Controller
    if i>1
    E{i,1}=E{i,1}(1:n_2,1:n_2)+E{i-1,1}(1:n_2,1:n_2);
    end
     F{i,1}=RE_2{i,1}(i+1:end,i+1:end);
 end
B_1=zeros(max(size(B)));

%% G Matrix

%making the matrix B in delay form to multiply by E
B_1=zeros(n_2+1);
for i=1:max(size(B))
    B_1(i,i)=B(i);
end

% G Matirix For controller
G_1=cell(n_2,1);
for i=1:max(size(G_1))
    G_1{i,1}=zeros(n_2+1,n_2+1);
end


for i=1:max(size(E))
%         A_ph_1=[zeros(n_2+2,i-1) A_ph(:,1:end+1-i)];
%     A_ph_2=[zeros(i-1,n_2+2); A_ph_1(1:end+1-i,:)];
if i<=1
 G_1{i,1}=E{i,1}(i,i).*B_1;
end
if i>1
    B_2=[zeros(n_2+1,i-1) B_1(:,1:end+1-i)];
    B_3=[zeros(i-1,n_2+1); B_2(1:end+1-i,:)];
    G_1{i,1}=(E{i,1}(i,i).*B_3)+G_1{i-1,1};
end
end
% Final G for coding the controller
G=zeros(n_2,n_2);
for i=1:max(size(G))
k=toeplitz(diag(G_1{i,1})');
G(i,1:i)=k(i,1:i);
end

%% Controller
 lambda=10; %Adjusting Parameter
Gp_1=zeros(n_2,1);
for i=1:n_2
    Gp_1(i,1)=G_1{i,1}(i+1,i+1);
end
F_1=zeros(n_2,2);
for i=1:n_2
F_1(i,1)=F{i,1}(1,1);
F_1(i,2)=F{i,1}(2,2);
end

H=2*(G'*G+lambda*eye(length(G'*G))); % H matrix from formulation
K=inv(G'*G+lambda*eye(length(G'*G)))*G'; %K= MAtrix from formulation
K_c=K(1,:); % first row of K for applying control law
T=200; %time s
Ts=1; %Sample time
nts=T/Ts; % number of samples
Gp=K_c*Gp_1;
F_n=K_c*F_1;
%% Applying Contoller law and estimation
R_1=[1 Gp];
S_1=K_c*F_1;
T_1=sum(K_c);
Bz1=conv(B,[0 1]);
num=conv(Bz1,T_1);
num_u=conv(A,T_1);
At=conv(A,delta);
den=conv(R_1,At)+conv(Bz1,S_1);
yrst=filter(num,den,[0 ones(1,nts+1)]);
urst=filter(num_u,den,[0 ones(1,nts+1)]);
if n_b>1
    dup=zeros(n_b-1,1);
end
u0=0;
y=0;
yp=zeros(n_a,1);
for k=1:nts+1
yp=[y(k);yp(1:end-1)];
if n_b==1
    fr = F_1*yp;
else
fr = F_1*yp + Gp_1*dup;    
end
w = 1*ones(n_2,1); % refrence
 du(k) = K_c*(w-fr);
 u(k) = du(k) + u0;
 [y(k+1), zf] = filter(B,A,u(k),zi);
 zi = zf;
 y(k+1)=y(k+1)+C(1,1)*noise_minor(k)+C(1,2)*noise_minor(k+1)+C(1,3)*noise_minor(k+2);
 if n_b>1
 dup = du(k); dup(1:end-1);
 end
 u0 = u(k);
end
ygpc=y; ugpc=u;
t = 0:Ts:T;
plot(t,ygpc(1:end-1),'b','linewidth',2),

xlabel('Time Step','color','r');
ylabel('Amplitude','color','r');
hold on
plot(t,ugpc,'g','linewidth',2), grid on
plot(t,du,'r','linewidth',2)
legend('y(t)','u(t)','Control effort');    
    
    
    
