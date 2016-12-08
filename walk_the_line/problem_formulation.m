% matlab code written when solving the exercises for
% https://onderwijsaanbod.kuleuven.be/syllabi/e/H04U1CE.htm#activetab=doelstellingen_idp1353952
% Jeroen De Maeyer 2016
clear;
close all;
clc;

% This script describes a two link robot
% moving from a point (1.5, 0) to (0, 1.5) over a straight line.

% robot parameters
% ================
% mass; length; mass moment of inertia; location cg; max motor torque;
m1 = 1; l1 = 1; I1 = 0.5; lc1 = 0.5; tau1 = 30;
m2 = 1; l2 = 1; I2 = 0.5; lc2 = 0.5; tau2 = 15;
par.l1 = l1;
par.l2 = l2;
par.mu = [m1 m2;
          m1*lc1 m2*lc2;
          m1*lc1^2 + I2 m2*lc2^2 + I2];
      
% trajectory input
% ================
% trajectory is line from (1.5, 0) to (0, 1.5)
% Yp = (xp, yp)'

M = 50; % samples to calculate inverse kinematics
Yp = [linspace(1.5, 0, M); linspace(0, 1.5, M)];

% calculate path in joint space with inverse kinematics
[a1, a2] = invers_kinematics(Yp(1, :), Yp(2, :), 1, par);

% get path in the model form qp(s) with M samples
s = linspace(0, 1, M);
qp = [a1; a2]; % discrete path in joint space, M samples

figure()
subplot(121)
plot(Yp(1, :), Yp(2, :), 'b.')
axis([-1 2 0 2])
% axis equal;
xlabel('x [m]')
ylabel('y [m]')

% plot arm during movement
hold on;
[x2, y2, x1, y1] = forward_kinematics(a1, a2, par);
for i = 1:5:M
    plot([0 x1(i) x2(i)], [0 y1(i) y2(i)], 'k-o')
end
hold off;

time = linspace(0, 2, M);
subplot(122)
plot(time, [a1; a2]')
xlabel('time [s]')
ylabel('[rad]')
legend('q_1', 'q_2')
