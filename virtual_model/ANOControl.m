function dx = ANOControl(t,x,measurements,params,m,epsilon,A)
%% Controller with ANO

%% User parameters that should be tuned later
% Getting K_P and K_D as a positive definite, symmetric, diagonal matrix
% from the parameters:
K_spring = params{1};
K_sp = K_spring(1);
K_sd = K_spring(2);
K_turque = params{2};
K_tp = K_turque(1);

%% Built-in physics, no need to worry
% Measurements of the 2-link arm
I1 = measurements{1};
I2 = measurements{2};
m1 = measurements{3};
m2 = measurements{4};
l1 = measurements{5};
l2 = measurements{6};
r1 = measurements{7};
r2 = measurements{8};
g = measurements{9};

a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;
% the actual dynamic model of the system:
Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
Gmat =  [m1*g*r1*cos(x(1))+m2*g*(l1*cos(x(1))+r2*cos(x(1)+x(2)));
m2*g*r2*cos(x(1)+x(2))];
invM = inv(Mmat);

%% The major part of VM control
% Jacobian matrix calculation. This matrix J (5 by 2) is composed of 
% Jv (2 by 2) and Jw (3 by 2)
q1 = x(1)*pi/180 + pi/2;
q2 = x(2)*pi/180;
O_12 = [r1*cos(q1+q2); r2*sin(q1+q2)];
O_02 = [r1*cos(q1)+r2*cos(q1+q2); r1*sin(q1)+r2*sin(q1+q2)];
I_skew = [0 -1; 1 0];
Jv = [I_skew*O_02 I_skew*O_12];
Jw = [[0 0 1]' [0 0 1]'];
J = [Jv; Jw];

% Current spring position
L_spring = norm(O_02, 2); % virtual spring length
v = J * [x(3); x(4)];
v = v(1:2);
L_spring_rate = O_02' * v / (L_spring);
q_spring = atan(O_02(2)/O_02(1));

% Desired spring position
L_0 = 0.8;
q_spring_des = 0;

% Error vector
e = [L_0-L_spring q_spring_des-q_spring];
xr = e(1);

% 
omega_n = sqrt(K_sp/m);
p = 1 + epsilon*m*(-L_spring_rate)*xr/A;
omega_square = L_spring_rate^2 / abs(A^2 - xr);
z = -epsilon*m^2*A*(-L_spring_rate)*(omega_square - omega_n^2)*xr^2/(A^2*p);
F_ANO = (-m*omega_square + K_sp)*xr - z;

% Command endpoint force according to desire. F_spring is the virtual
% spring force, T_spring is the turque to adjust angular position
F_spring = F_ANO.*O_02./L_spring; % magnitude and direction, 2-d vector
T_spring = e(2)*(K_tp); % turque magnitude, a force tangent to spring
T_spring = T_spring.*I_skew*O_02./L_spring; % magnitude and direction, 3-d vector
F = [F_spring+0; [0 0 0]']; % composition of force and torque

% Input controller u for two joints
u = J'*F;

%% Simulation process calculated by ode. No need to worry.

% From state-space representation, dx = [q_dot q_double_dot]^T
dx = zeros(4,1);

% q_dot can be taken from the current state variable x
q_dot = x(3:4);
dx(1:2) = q_dot;

% q_double_dot comes from the dynamics of the arm and setting the torque as
% the input controller u
q_double_dot = invM * (u - Cmat*q_dot - 0);
dx(3:4) = q_double_dot;
end

