%% Simulation of Virtual Model Control
% I haven't yet tuned all the gains and I haven't implemented derivative
% control, but a general idea goes on like this. Notice that the system 
% will not converge with only proportional control.

clc;
clear;
close all;

%% Initialization
tf=100;
x0_s=[-80 -20 0 0];              %Setting initial conditions for the state vector [q1, q2, q1_dot, q2_dot]
k_spring=[400 0];                %proportional and derivative gain for the spring [kp, kd]
k_position=[100 0];              %proportional and derivative gain for the spring position [kp kd]

%% Built-in Link Properties, no need to worry

I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1; 
g=9.8;
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

system = {I1,I2,m1,m2,l1,l2,r1,r2,g};  %System List


%% Implement the Virtual Model control, this is crucial
params = {k_spring, k_position};
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,W] = ode45(@(t,w) VirtualModelControl(t,w,system,params),[0 tf],x0_s, options);


figure(1);
plot(T, W(:,1),'r-');
title('Theta1 Evolution')
xlabel('Time')
ylabel('Theta1')
hold on
grid on
% 
figure(2);
plot(T, W(:,2),'r--');
title('Theta2 Evolution')
xlabel('Time')
ylabel('Theta2')
hold on
grid on
