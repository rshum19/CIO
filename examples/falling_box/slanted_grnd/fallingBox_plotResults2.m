function fallingBox_plotResults2(t,x,lambda)

% --- State
figure
% X postion
subplot(3,1,1)
plot(t,x(1,:));
ylabel('x-pos [m]');
title('State variables evolution')
% Y postion
subplot(3,1,2)
plot(t,x(2,:));
ylabel('y-pos [m]');

% Theta orientation
subplot(3,1,3)
plot(t,x(3,:));
ylabel('\theta orientation [m]');
xlabel('time [sec]');

% --- Contact Forces
figure
% Lamnda_x forces
subplot(1,2,1)
plot(t,lambda(1,:))
title('\lambda_{x}, tangential force')
ylabel('Force [Nm]')
xlabel('time [sec]');

% Lamnda_y forces
subplot(1,2,2)
plot(t,lambda(2,:))
title('\lambda_{y}, normal force');
xlabel('time [sec]');
end

