clc;
clear;
%Define system dynamics
I_x = 0.0196;
I_y = 0.0196;
I_z = 0.0264; 
d = 0.25; 
M = 0.5; 
c = 0.1;    
g = -9.81;

%Define state space representations
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

%Open loop response of pitch for zero initial state and step input in F1
t = 0:0.01:50;
u = zeros(length(t),4);
u(:,1)=1;
x0 = [0;0;0;0;0;0];

sys = ss(A,B,C,D);
rank_ctr=rank(ctrb(sys));
rank_obs=rank(obsv(sys));
[y,t,x] = lsim(sys,u,t,x0);
figure(1)
plot(t,y(:,3))
title('Open-Loop Response to Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Pitch angle')

x0 = [0.01;0.02;0;0;0;0];
u(:,1)=1;
%Closed loop response of pitch for non-zero initial state and step input in
%F1 and for smaller timestep
t = 0:0.01:2;
u = zeros(length(t),4);

P=[-4+3i -4-3i -20 -30 -40 -50];
K=place(A,B,P);
Acl=A-B*K;
syscl=ss(Acl,B,C,D);
 
[y,t,x] = lsim(syscl,u,t,x0);

figure(2)
plot(t,y(:,2))
title('Closed-Loop Response to Non-Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Pitch angle')

%Design of Observer to estimate the states and plot the response curve
Op=[-40,-41,-42,-43,-44,-45];
L=place(A',C',Op)';


At = [ A-B*K             B*K
       zeros(size(A))    A-L*C ];

Bt = [    B
       zeros(size(B)) ];

Ct = [ C    zeros(size(C)) ];

sys_obs = ss(At,Bt,Ct,0);
x_obs=[x0;x0];
[y_obs,t,x_obs] = lsim(sys_obs,u,t,x_obs);

figure(3);
plot(t,y_obs(:,2))
title('Observer Response to Non-Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Pitch angle')

%Plotting the graph of estimated state and actual state for pitch rate
n = 6;
e = x_obs(:,n+1:end);
x = x_obs(:,1:n);
x_est = x - e;


figure(4)
plot(t,x(:,5),'-r',t,x_est(:,5),'-g')
title('Estimated vs Actual State')
legend('x','xest')
xlabel('Time (sec)')
ylabel("Pitch Rate")
