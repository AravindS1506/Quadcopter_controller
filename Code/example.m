A=[0 1 0; 980 0 -2.8;0 0 -100];
B=[0;0;100];
C=[1 0 0];
D=0;

poles=eig(A);

t = 0:0.01:2;
u = zeros(size(t));
x0 = [0.01 0 0];

sys = ss(A,B,C,0);

[y,t,x] = lsim(sys,u,t,x0);
plot(t,x)
title('Open-Loop Response to Non-Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Ball Position (m)')
rank(ctrb(sys));
rank(obsv(sys));

p1 = -20 + 20i;
p2 = -20 - 20i;
p3 = -100;

K = place(A,B,[p1 p2 p3]);
sys_cl = ss(A-B*K,B,C,0);

lsim(sys_cl,u,t,x0);
xlabel('Time (sec)')
ylabel('Ball Position (m)')
t = 0:0.01:2;
u = 0.001*ones(size(t));

sys_cl = ss(A-B*K,B,C,0);

lsim(sys_cl,u,t);
xlabel('Time (sec)')
ylabel('Ball Position (m)')
axis([0 2 -4E-6 0])
%{
k=place(A,B,[-20+20i;-20-20i;-100]);
sys_cl=ss(A-B*k,B,C,D);
figure(2)
lsim(sys_cl,u,t,x0)

eig(A-B*k);
obs=obsv(A,C);
Ro=rank(obs);
l=place(A',C',[-15,-16,-20])';
at=[A-B*k B*k;zeros(size(A)) A-l*C];
bt=[B;zeros(size(B))];
ct=[C zeros(size(C))];
syso=ss(at,bt,ct,0);
z1=[0.01 0.5 -5];
figure(3)
[y,t,x]=lsim(syso,u,t,[x1 x1]);
e=x(:,n+1:end);
x=x(:,1:n);
xe=x-e;
x1=x(;,1);x2=x(;,2);x3=x()
%}