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
F = [1 dt; 0 1];                                 %<----- FILLED IN FOR (b)%

% Control input matrix
m = 10 % arbitrary mass of car
B = [dt^2/(2*m); dt/m];                          %<----- FILLED IN FOR (b)%

% Measurement-state transformation
H = [1 0; 0 1];

% Error covariance matrix
P = [1 0; 0 1];

% Loop until car comes to a complete stop (velocity = 0)
while x(2) > 0
    % Braking force applied at this time step
    u = -20;
    
    % Update model
    xModel = F*xModel + B*u;

    % Simulate real state update, including process noise
    t = t + dt;
    x = F*x + B*u + normrnd(0, diag(Q));
    
    % Simulate a noisy measurement
    z = H * x + normrnd(0, diag(R));
    
                                                  %vv FILLED IN FOR (b) vv%
    % Predict
    xHat = F*xHat + B*u;
    P = F*P*F.' + Q;
    
    % Kalman gain
    K = P*H.'*inv(H*P*H.' + R);
    
    % Update
    xHat = xHat + K*(z - H*xHat);
    P = (eye(2) - K*H)*P;                                                                 
                                                  %^^ FILLED IN FOR (b) ^^%
end



