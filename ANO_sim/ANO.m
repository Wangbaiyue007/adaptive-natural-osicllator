function [dxdt, F] = ANO(t, x, m, k, epsilon)
%ANO: state space of adaptive neural oscillator
%   simulation of a single digree of freedom spring mass system, with input
%   state x, mass m, spring constant k, and learning rate epsilon.
%   Question: does it converge?
    omega_n = sqrt(k/m);
    p = 1 + epsilon*m*x(4)*sin(x(3))*cos(x(3));
%     omega_dot = 0;
    omega_dot = (1/p)*(-epsilon*x(4)*(x(4)-omega_n)*(x(4)+omega_n)*cos(x(3))^2);
%     F = 0;
    F = (k-m*x(4)^2)*cos(x(3)) - m*omega_dot*sin(x(3));
    % PD control
    F = F - 50*(x(1)-sin(x(3))) - 1*(x(2)+sin(x(3))*x(4));
    % impact disturbance
%     if t >= 15 && t < 17
%         F = F + 500;
%     end
    % Gaussian disturbance
    F = awgn(F, 0.1, 10);
    dxdt = zeros(4,1);
    dxdt(1) = x(2);
    dxdt(2) = 1/m*(F-k*x(1));
    dxdt(3) = x(4);
    dxdt(4) = omega_dot;
end

