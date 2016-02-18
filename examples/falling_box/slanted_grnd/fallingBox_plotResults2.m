function fallingBox_plotResults2(t,x,lambda)

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
subplot(1,2,1)
plot(t,lambda(1,:))
title('\lambda_{x}')
ylabel('Contact 1, Force [Nm]')
xlabel('time [sec]');

% Lamnda_y forces
subplot(1,2,2)
plot(t,lambda(2,:))
title('\lambda_{y}');
xlabel('time [sec]');
end

