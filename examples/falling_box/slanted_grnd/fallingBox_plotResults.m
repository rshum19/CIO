function fallingBox_plotResults(t,x,lambda)

% --- State
figure
% X postion
subplot(3,1,1)
plot(t,x(1,:));
ylabel('x-pos [m]');

% Y postion
subplot(3,1,2)
plot(t,x(2,:));
ylabel('y-pos [m]');

% Theta orientation
subplot(3,1,3)
plot(t,x(3,:));
ylabel('\theta-ornt. [m]');
xlabel('time [sec]');

% --- Contact Forces
figure
% Lamnda_x forces
subplot(3,2,1)
plot(t,lambda(1,:))
title('\lambda_{x}')
ylabel('Contact 1, Force [Nm]')

subplot(3,2,3)
plot(t,lambda(2,:))
ylabel('Contact 2, Force [Nm]');

subplot(3,2,5)
plot(t,lambda(3,:))
xlabel('time [sec]');
ylabel('Contact 3, Force [Nm]')

% Lamnda_y forces
subplot(3,2,2)
plot(t,lambda(4,:))
title('\lambda_{y}');

subplot(3,2,4)
plot(t,lambda(5,:))

subplot(3,2,6)
plot(t,lambda(6,:))
xlabel('time [sec]');
end

