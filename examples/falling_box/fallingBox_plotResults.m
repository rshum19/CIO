function fallingBox_plotResults(t,x,lambda)

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

% --- Contact Forces
figure
idx = 1:2:2*nLambda;
for i = 1:nLambda
    
    % Lamnda_x forces 
    subplot(nLambda,2,idx(i))
    plot(t,lambda(i,:))
    title('\lambda_{x}')
    xlabel('time [sec]');
    ylabel('Force [N]')
    if i == 1
        title('\lambda_{x}');
    end
    
    % Lamnda_y forces
    subplot(nLambda,2,idx(i)+1)
    plot(t,lambda(i+nLambda,:))
    xlabel('time [sec]');
    ylabel('Force [N]')
    if i == 1
        title('\lambda_{y}');
    end
end

end

