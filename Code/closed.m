clc;
clear;
I_x = 0.0196; %kg m^2
I_y = 0.0196;
I_z = 0.0264; %kg m^2
d = 0.25; %m
M = 0.5; %kg
c = 0.1; %(force to moment sclaing factor)
g = -9.81;

A = zeros(6,6);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

B = zeros(6,4);
B(4,2) = d/I_x;
B(4,4) = -d/I_x;
B(5,1) = d/I_y;
B(5,3) = -d/I_y;
B(6,1) = -c/I_z;
B(6,2) = c/I_z;
B(6,3) = -c/I_z;
B(6,4) = c/I_z;

C = zeros(3,6);
C(1,1) = 1;
C(2,2) = 1;
C(3,3) = 1;

D = zeros(3,4);

t = 0:0.01:2;
poles = eig(A);
x0 = [0.01;0.02;0;0;0;0];
u = zeros(length(t),4);
u(:,4)=1;
sys = ss(A,B,C,D);
%phi-roll angle
%theta-pitch
%psi-yaw
%{
[y,t,x] = lsim(sys,u,t,x0);
plot(t,y(:,1))
title('Open-Loop Response to Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Roll angle')
%}
P=[-4+3i -4-3i -20 -30 -40 -50];
K=place(A,B,P);
Acl=A-B*K;
syscl=ss(Acl,B,C,D);

[y,t,x] = lsim(syscl,u,t,x0);


plot(t,y(:,3))
title('Closed-Loop Response to Non-Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Yaw angle')


Op=[-40,-41,-42,-43,-44,-45];
L=place(A',C',Op)';


At = [ A-B*K             B*K
       zeros(size(A))    A-L*C ];

Bt = [    B
       zeros(size(B)) ];

Ct = [ C    zeros(size(C)) ];

sys_obs = ss(At,Bt,Ct,0);
x_obs=[x0;x0];
[y_obs,t,x] = lsim(sys_obs,u,t,x_obs);
%{
n = 6;
e = x(:,n+1:end);
x = x(:,1:n);
x_est = x - e;

% Save state variables explicitly to aid in plotting
h = x(:,1); h_dot = x(:,2); i = x(:,3);
h_est = x_est(:,1); h_dot_est = x_est(:,2); i_est = x_est(:,3);

plot(t,x(:,1),'-r',t,x_est(:,1),':r')
xlabel('Time (sec)')
ylabel("Roll")
%}
