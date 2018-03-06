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
m = 10; % arbitrary mass of car
B = [dt^2/(2*m); dt/m];                         

% State-measurement transformation
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
    % Braking force applied at this time step
    u = -20;
                                                      %vv ADDED FOR (d) vv%
    if mod(floor(t), 2) == 1
        u = 0;
    end
                                                      %^^ ADDED FOR (d) ^^%
    % Update model
    xModel = F*xModel + B*u;

    % Simulate real state update, including process noise
    t = t + dt;
    x = F*x + B*u + normrnd(0, diag(Q));
    
    % Simulate a noisy measurement
    z = H * x + normrnd(0, diag(R));
                                              
    % Predict
    xHat = F*xHat + B*u;
    P = F*P*F.' + Q;
    
    % Kalman gain
    K = P*H.'*inv(H*P*H.' + R);
    
    % Update
    xHat = xHat + K*(z - H*xHat);
    P = (eye(2) - K*H)*P;   
    
                                                      %vv ADDED FOR (c) vv%
    times = [times t];
    states = [states x];
    model = [model xModel];
    measurements = [measurements z];
    outputs = [outputs xHat];
                                                      %^^ ADDED FOR (c) ^^%
end


                                                     
% Plot position data
hold on;
plot(times, measurements(1,:), 'cyan');
plot(times, model(1,:), 'b');
plot(times, states(1,:), 'black');
plot(times, outputs(1,:), 'r');

xlabel('Time');
ylabel('Position')
legend('Measurement','Model', 'Reality', 'Kalman Output')
legend('Location', 'southeast')
                                                    



