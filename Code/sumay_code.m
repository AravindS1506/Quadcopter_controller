

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
u = zeros(length(t),4);
x0 = zeros(6,1);    % Initial state
%{
u(:,2) = 1;   % Adding step input to one component of U.
sys = ss(A,B,C,D);


[y,t,x] = lsim(sys,u,t,x0);
plot(t,y(:,1))
title('Open-Loop Response to Non-Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Roll Angle')
%}

T_s = 1; %settling time
damp_ratio = 0.8;
freq = 4/(damp_ratio * T_s);   % for 2% tolerence

s1 = -4+3i;
s2 = -4-3i;
s3 = -20;
s4 = -30;
s5 = -40;
s6 = -50;

%P = [s1 s2 s3 s4 s5 s6];
%K = place(A,B,P);

P=[-4+3i -4-3i -20 -30 -40 -50];
K=place(A,B,P);
disp(K)
%N_bar = rscale(sys, K)
A_cl = A-B*K;
sys_cl = ss(A_cl, B, C, D);

u(:,1) = 1;

[y,t,x] = lsim(sys_cl,u,t,x0);
plot(t,y(:,2))
title('Closed-Loop Response to Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Pitch Angle')
%{
s_o_1 = -100;
s_o_2 = -101;
s_o_3 = -102;
s_o_4 = -103;
s_o_5 = -104;
s_o_6 = -105;

P_obs = [s_o_1 s_o_2 s_o_3 s_o_4 s_o_5 s_o_6];

L = place(A', C', P_obs)';


At = [ A-B*K             B*K ;
       zeros(size(A))    A-L*C ];

Bt = [    B ;
       zeros(size(B)) ];

Ct = [ C    zeros(size(C)) ];

sys_obs_cl = ss(At, Bt, Ct, 0);

u(:,1) = 1;

[y,t,x] = lsim(sys_obs_cl,u,t,[x0;x0]);
plot(t,y(:,2))
title('Closed-Loop Response to Non-Zero Initial Condition (with observer)')
xlabel('Time (sec)')
ylabel('Pitch Angle')

%}


