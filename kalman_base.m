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
F = %FILL ME IN%

% Control input matrix
m = 10; % arbitrary mass of car
B = %FILL ME IN%

% State-measurment transformation
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
    
    % FILL
    % ME
    % IN
end



