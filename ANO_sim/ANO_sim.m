clear; clc; close all;
%% parameters
m = 70; % mass
k = 500; % spring constant
epsilon = 0.5; % learning rate
omega_n = sqrt(k/m); % natural frequency
%% simulation
tspan = [0 30];
x0 = [0.5 0 0 0.5];
y0 = [2 1 0 0.1];
[t,x] = ode45(@(t,x) ANO(t,x,m,k,epsilon), tspan, x0);
[ty,y] = ode45(@(ty,y) ANO(ty,y,m,k,epsilon), tspan, y0);
subplot(2,1,1);
plot(t,x(:,1),t,cos(x(:,3))); legend('x_r', 'x_d'); grid on; ylabel('Position [m]')
subplot(2,1,2);
plot(t,x(:,4)); grid on; ylabel('Frequency [rad/s]'); title('\omega_n = ',num2str(omega_n));
%% state space
figure;
plot(x(:,1), x(:,2)); grid on; hold on;
plot(y(:,1), y(:,2));
plot(x(1,1), x(1,2), 'xk', y(1,1), y(1,2), 'xk');
legend('trajectory 1', 'trajectory 2', 'starting point 1', 'starting point 2');
xlabel('x_1 (position)'); ylabel('x_2 (velocity)');
