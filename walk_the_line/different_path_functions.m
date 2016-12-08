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
% yp = (xp, yp)'
M = 50; % samples to calculate inverse kinematics
yp = [linspace(1.5, 0, M); linspace(0, 1.5, M)];
[a1, a2] = invers_kinematics(yp(1, :), yp(2, :), 1, par);

% get path in the model form qp(s) with M samples
sM = linspace(0, 1, M);
qpM = [a1; a2]; % discrete path in joint space, M samples

%% Linear time movement
% discretize time from 0 to total motion time T
N = 30;
T = 2;
h = T/N;
t = linspace(0, T, N);

% calculate s(t), in this case linear s(0) = 0, s(T) = 1
s = linspace(0, 1, N);
% s = [0, 0.1, 0.15, 0.20, 0.25, 0.30, 0.8, 0.85, 0.90, 1];

% resample path in joint space
qp = interp1(sM', qpM', s')';

% resample path in cartesian space
[x2, y2, x1, y1] = forward_kinematics(qp(1,:), qp(2, :), par);

% plot s(t)
figure()
hold off;
subplot(311)
plot(t, s)
xlabel('t [s]')
ylabel('s [-]')
title('Function s(t)')

subplot(312)
plot(s, qp, 'o-')
xlabel('s [-]')
ylabel('q [rad]')
title('Path in joint space Q(s)')
legend('q_1(s)', 'q_2(s)')

subplot(313)
plot(t, qp, 'o-')
xlabel('time [s]')
ylabel('q [rad]')
title('Path in joint space Q(t)')
legend('q_1(t)', 'q_2(t)')

% plot arm during movement
figure()
plot([0 x1(1) x2(1)], [0 y1(1) y2(1)], 'b-o')
axis([-2 2 -2 2])
% axis equal;
xlabel('x [m]')
ylabel('y [m]')
hold on;
for i = 2:N
    cla();
    plot([0 x1(i) x2(i)], [0 y1(i) y2(i)], 'b-o')
    pause(h);
end
hold off;

%% s(t) = -1/2 * (cos(pi/T * t) + 1)
% discretize time from 0 to total motion time T
N = 30;
T = 2;
h = T/N;
t = linspace(0, T, N);

% calculate s(t) with s(0) = 0, s(T) = 1
s = -1/2 * cos(pi/T * t) + 1/2;
s = -1/2 * cos(pi * s) + 1/2; % compress some more !

% resample path in joint space
qp = interp1(sM', qpM', s')';

% resample path in cartesian space
[x2, y2, x1, y1] = forward_kinematics(qp(1,:), qp(2, :), par);

% plot s(t)
figure()
subplot(311)
plot(t, s)
xlabel('t [s]')
ylabel('s [-]')
title('s(t)')

subplot(312)
plot(s, qp, 'o-')
xlabel('s [-]')
ylabel('q [rad]')
title('Path in joint space Q(s)')

subplot(313)
plot(t, qp, 'o-')
xlabel('time [s]')
ylabel('q [rad]')
title('Path in joint space Q(s)')
legend('q_1(t)', 'q_2(t)')

% plot arm during movement
figure()
plot([0 x1(1) x2(1)], [0 y1(1) y2(1)], 'b-o')
axis([-2 2 -2 2])
% axis equal;
xlabel('x [m]')
ylabel('y [m]')
hold on;
for i = 2:N
    cla();
    plot([0 x1(i) x2(i)], [0 y1(i) y2(i)], 'b-o')
    pause(h);
end
hold off;

%% s(t) weird piecewise function
% discretize time from 0 to total motion time T
N = 30;
T = 2;
h = T/N;
t = linspace(0, T, N);

% calculate s(t) with s(0) = 0, s(T) = 1
N_dont_move = 15; % more like almost don't move, only a little
N_move = N - N_dont_move;
s1 = linspace(0, 0.5, floor(N_move / 2));
s2 = linspace(0.5, 0.55, N_dont_move);
s3 = linspace(0.55, 1, floor(N_move / 2) + mod(N_move, 2));
s = [s1, s2, s3];

% check size of s
if size(s, 2) ~= N
    error('No succes in getting s with the right size');
end

% resample path in joint space
qp = interp1(sM', qpM', s')';

% resample path in cartesian space
[x2, y2, x1, y1] = forward_kinematics(qp(1,:), qp(2, :), par);

% plot s(t)
figure()
subplot(311)
plot(t, s)
xlabel('t [s]')
ylabel('s [-]')
title('s(t)')

subplot(312)
plot(s, qp, 'o-')
xlabel('s [-]')
ylabel('q [rad]')
title('Path in joint space Q(s)')

subplot(313)
plot(t, qp, 'o-')
xlabel('time [s]')
ylabel('q [rad]')
title('Path in joint space Q(s)')
legend('q_1(t)', 'q_2(t)')

% plot arm during movement
figure()
plot([0 x1(1) x2(1)], [0 y1(1) y2(1)], 'b-o')
axis([-2 2 -2 2])
% axis equal;
xlabel('x [m]')
ylabel('y [m]')
hold on;
for i = 2:N
    cla();
    plot([0 x1(i) x2(i)], [0 y1(i) y2(i)], 'b-o')
    pause(h);
end
hold off;
