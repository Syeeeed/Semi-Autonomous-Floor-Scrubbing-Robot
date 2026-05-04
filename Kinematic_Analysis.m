clear;
clc;

interval = 1;
loopCount = 360 / interval;

r1 = -10;
r2 = 15;
r3 = 40;

theta2 = 0;
theta2dot = 30;
theta2doubledot = 0;

for i = 1:loopCount

    % Angular displacement (Link 3) and displacement (Link 4)
    theta3 = 180 - asind(((-r1) + (r2 * sind(theta2))) / r3);
    r4 = (r2 * cosd(theta2)) - (r3 * cosd(theta3));

    % Angular velocity (Link 3) and velocity (Link 4)
    theta3dot = (r2 * theta2dot * cosd(theta2)) / (r3 * cosd(theta3));
    r4dot = (r2 * theta2dot * sind(theta3 - theta2)) / cosd(theta3);

    % Angular acceleration (Link 3) and acceleration (Link 4)
    theta3doubledot = ((-r2 * theta2dot^2 * sind(theta2)) + ...
                      (r2 * theta2doubledot * cosd(theta2)) + ...
                      (r3 * theta3dot^2 * sind(theta3))) / (r3 * cosd(theta3));

    r4doubledot = ((-r2 * theta2dot^2 * cosd(theta3 - theta2)) + ...
                  (r2 * theta2doubledot * sind(theta3 - theta2)) + ...
                  (r3 * theta3dot^2)) / cosd(theta3);

    % Store values
    theta2Matrix(i) = theta2;

    theta3Matrix(i) = theta3;
    theta3dotMatrix(i) = theta3dot;
    theta3doubledotMatrix(i) = theta3doubledot;

    r4Matrix(i) = r4;
    r4dotMatrix(i) = r4dot;
    r4doubledotMatrix(i) = r4doubledot;

    % Increment input angle
    theta2 = theta2 + interval;

end

%% Plotting

figure(1); clf;

subplot(1,3,1);
title('Angular Displacement for Link 3');
plot(theta2Matrix, theta3Matrix, 'b');
xlabel('Theta2 (deg)');
ylabel('Theta3 (deg)');
grid on;

subplot(1,3,2);
title('Angular Velocity for Link 3');
plot(theta2Matrix, theta3dotMatrix, 'b');
xlabel('Theta2 (deg)');
ylabel('Theta3dot (rad/s)');
grid on;

subplot(1,3,3);
title('Angular Acceleration for Link 3');
plot(theta2Matrix, theta3doubledotMatrix, 'b');
xlabel('Theta2 (deg)');
ylabel('Theta3ddot (rad/s^2)');
grid on;


figure(2); clf;

subplot(1,3,1);
title('Displacement for Link 4');
plot(theta2Matrix, r4Matrix, 'b');
xlabel('Theta2 (deg)');
ylabel('r4 (cm)');
grid on;

subplot(1,3,2);
title('Velocity for Link 4');
plot(theta2Matrix, r4dotMatrix, 'b');
xlabel('Theta2 (deg)');
ylabel('r4dot (cm/s)');
grid on;

subplot(1,3,3);
title('Acceleration for Link 4');
plot(theta2Matrix, r4doubledotMatrix, 'b');
xlabel('Theta2 (deg)');
ylabel('r4ddot (cm/s^2)');
grid on;
