% Process noise covariance matrix. Represents system noisiness
varQ = 0.02;
Q = [varQ 0; 0 varQ];

% Measurement noise covariance matrix. Represents measurement noisiness
varR = 1;
R = [varR 0; 0 varR];

% Start time and time step
t = 0;
dt = 0.001;

% Real, estimated, and modeled <position, vecocity> state vectors
x = [0; 10];
xHat = [0; 10];
xModel = [0; 10];

% State update matrix
F = [1 dt; 0 1];

% Control input matrix
B = [dt^2/2; dt];

% Measurement-state transformation
H = [1 0; 0 1];

% Error covariance matrix
P = [1 0; 0 1];

% Data lists
times = [];
states = [];
measurements = [];
outputs = [];
model = [];

% Loop until car comes to a complete stop (velocity = 0)
while x(2) > 0
    % State update
    t = t + dt;
    u = -2; % Braking force applied at this time step

    x = F*x + B*u + normrnd(0, diag(Q));
    times = [times t];
    states = [states x];
    
    % Update model (ideal system with no noise)
    xModel = F*xModel + B*u;
    model = [model xModel];
    
    % Predict
    xHat = F*xHat + B*u;
    P = F*P*F.' + Q;
    
    % Kalman gain
    K = P*H.'*inv(H*P*H.' + R);
    
    % Measurement
    z = H * x + normrnd(0, diag(R));
    measurements = [measurements z];
    
    % Update
    xHat = xHat + K*(z - H*xHat);
    P = (eye(2) - K*H)*P;
    
    outputs = [outputs xHat];
end

hold on;

% Position
plot(times, measurements(1,:), 'cyan');
% plot(times, model(1,:), 'b');
plot(times, states(1,:), 'black');
plot(times, outputs(1,:), 'r');


% Velocity
% plot(times, measurements(2,:), 'y');
% plot(times, outputs(2,:), 'r');
% plot(times, model(2,:), 'b');
% plot(times, states(2,:), 'black');

xlabel('Time');
ylabel('Position')
%legend('Measurement','Model', 'Reality', 'Kalman Output')




