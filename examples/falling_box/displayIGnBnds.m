function displayIGnBnds(guess,Bnds,nGrid)


% Build boundaries
% Mux lower bounds
time.lb = linspace(Bnds.initTime.lb, Bnds.finalTime.lb, nGrid);
state.lb = [Bnds.initState.lb, Bnds.state.lb*ones(1,nGrid-2), Bnds.finalState.lb];
ctrl.lb = Bnds.control.lb*ones(1,nGrid);
lambda.lb = Bnds.lambda.lb*ones(1,nGrid);

% Mux upper bounds
time.ub = linspace(Bnds.initTime.ub, Bnds.finalTime.ub, nGrid);
state.ub = [Bnds.initState.ub, Bnds.state.ub*ones(1,nGrid-2), Bnds.finalState.ub];
ctrl.ub = Bnds.control.ub*ones(1,nGrid);
lambda.ub = Bnds.lambda.ub*ones(1,nGrid);

% Initial guess
figure
subplot(4,1,1)
plot(guess.time,guess.state(2,:),guess.time,state.lb(2,:),guess.time,state.ub(2,:))
xlabel('Time [sec]');
ylabel('Mass M1 height [m]');

subplot(4,1,2)
plot(guess.time,guess.state(4,:))
xlabel('Time [sec]');
ylabel('Mass M1 velocity [m/sec]');

subplot(4,1,3)
plot(guess.time,guess.control)
xlabel('Time [sec]');
ylabel('U control input');

subplot(4,1,4)
plot(guess.time,guess.lambda(1,:),guess.time,guess.lambda(2,:))
xlabel('Time [sec]');
ylabel('Lambda');
end

