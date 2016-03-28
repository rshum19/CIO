function fallingBox_plotResults(t,x,u,lambda)

% --- Read input
nTime = length(t);
nStates = size(x,1)/2;
nLambda = size(lambda,1)/2;

% --- State
figure
% X postion
subplot(nStates,1,1)
plot(t,x(1,:));
ylabel('x-pos [m]');
xlabel('time [sec]');

% Y postion
subplot(nStates,1,2)
plot(t,x(2,:));
ylabel('y-pos [m]');
xlabel('time [sec]');

if nStates > 2
    % Theta orientation
    subplot(nStates,1,3)
    plot(t,x(3,:));
    ylabel('\theta-ornt. [m]');
    xlabel('time [sec]');
end

% --- Control inputs
figure
plot(t,u(1,:))
xlabel('time [sec]');
ylabel('input')
title('control input');

% --- Contact Forces
figure
subplot(3,1,1)
plot(t,lambda(1,:))
xlabel('time [sec]');
ylabel('Force [N]')
title('\lambda_{y}');

subplot(3,1,2)
plot(t,lambda(2,:))

subplot(3,1,3)
plot(t,lambda(3,:))


end

