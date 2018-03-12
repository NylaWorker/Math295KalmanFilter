% This file contains a Kalman filter implementation for UAV localization.
% Authors: Nyla Worker and Kiran Tomlinson
% 3/12/18

% Read in 3DR Solo data
data = csvread('solo_data.csv', 1, 0);
lat = data(:, 1);
long = data(:, 2);
pitch = data(:,3);
roll = data(:,4);

% Convert lat/lon data into local x/y corrdinates
n = size(data, 1);
coord = ones(n,2);
coord(1,1) = 0; coord(1,2) = 0 ;

for i=1:n-1
    longdif = long(i)-long(i+1);
    latdif = lat(i)-lat(i+1);
    
    coord(i+1,1) = coord(i,1) + 111111*longdif;
    coord(i+1,2) = coord(i,2) + 111111*latdif;
end

% Add gaussian noise to the GPS measurement
noisyCoord = coord + normrnd(0, varR, m, 2);


dt = 1/20;
t = 0;

% Initial conditions
x = [0;0;0;0];
xHat = [0;0;0;0];

% State update matrix
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; 

% State-measurement transformation
H = [1 0 0 0; 0 1 0 0];

% Control input matrix: (pitch/roll input)
effect = 0.5;
B = [0 0; 0 0;effect 0; 0 effect];

% Error covariance matrix
P = eye(4); 


% Process noise covariance matrix. Represents system noisiness
varQ = 0.001;
Q = [varQ 0  0 0; 0 varQ 0 0; 0 0 varQ 0; 0 0 0 varQ];

% Measurement noise covariance matrix. Represents measurement noisiness
varR = 0.1;
R = [varR 0; 0 varR ];

times = [];
states = [];
measurements = [];
outputs = [];


for i=1:n-1
    % Un-skew pitch/roll data
    u = [sin(pitch(i)+0.05)*cos(roll(i)-0.05); -sin(roll(i)-0.05)];
    
    % Use skewed pitch/roll data
%     u = [sin(pitch(i))*cos(roll(i)); -sin(roll(i))];

    % Update time and state
    t = t + dt;
    times = [times t];
    states = [states coord(i,1:2)'];
    
    % Predict
    xHat = F*xHat + B*u;
    P = F*P*F.' + Q;
    
    % Kalman gain
    K = P*H.'*inv(H*P*H.' + R);
    
    % Measurement
    z = noisyCoord(i,1:2)';
    measurements = [measurements z];
    
    % Update
    xHat = xHat + K*(z - H*xHat);
    P = (eye(4) - K*H)*P;
   
    outputs = [outputs xHat];
end


% Plot output and measurement
hold on;
plot1 = plot( measurements(1,:), measurements(2,:), 'r');
plot1.Color(4) = 0.5;
plot(outputs(1,:), outputs(2,:), 'b');

xlabel('x');
ylabel('y')




