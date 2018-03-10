data = csvread('solo_data.csv', 1, 0);
n = size(data, 1);
% p = plot(data(:, 1), data(:, 2), 'LineWidth', 3);
% cd = [uint8(jet(n)*255) uint8(ones(n,1))].';
% drawnow
% set(p.Edge, 'ColorBinding','interpolated', 'ColorData',cd)

lat = data(:, 1); long = data(:, 2); pitch = data(:,3); roll = data(:,4);
m = length(lat);
dt = 1/20;
t = 0;

% Initial conditions
x = [0;0;0;0];
xHat = [0;0;0;0];
xModelPR = [0; 0; 0; 0];

% State update matrix
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; 


% State-measurement transformation
H = [1 0 0 0; 0 1 0 0];

% Control input matrix: sensor input matrix 
B = [0 0; 0 0; 1 0; 0 1];

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
modelPR = [];

m = length(lat);
coord = ones(m,2);
coord(1,1) = 0; coord(1,2) = 0 ;


for i=1:m-1
    longdif = long(i)-long(i+1);
    
    latdif = lat(i)-lat(i+1);
    coord(i+1,1) =  coord(i,1)+ 111111*longdif;coord(i+1,2) =coord(i,2)+111111*latdif;
end

noisyCoord = coord + normrnd(0, varR, m, 2);


for i=1:m-1
%     u = [sin(pitch(i)+0.05)*cos(roll(i)-0.05); -sin(roll(i)-0.05)];
    u = [sin(pitch(i))*cos(roll(i)); -sin(roll(i))];

    t = t + dt;
    
    xModelPR = F*x + B*u + normrnd(0, diag(Q));
    
    times = [times t];
    states = [states coord(i,1:2)'];
    
    xModelPR = F*xModelPR + B*u;
    modelPR= [modelPR xModelPR];
    
    % Predict
    xHat = F*xHat + B*u;
    P = F*P*F.' + Q;
    
    % Kalman gain
    K = P*H.'*inv(H*P*H.' + R);
    
    % Measurement
    z = noisyCoord(i,1:2)';
%     z = H * xcur + normrnd(0, diag(R));
    measurements = [measurements z];
    
    % Update
    xHat = xHat + K*(z - H*xHat);
    P = (eye(4) - K*H)*P;
    
    
    outputs = [outputs xHat];
    
    
   
end



hold on;

% Position
plot( measurements(1,:), measurements(2,:), 'cyan');
% plot(times, model(1,:), 'b');
plot(states(1,:), states(2,:), 'black');
plot(outputs(1,:), outputs(2,:), 'r');


% Velocity
% plot(times, measurements(2,:), 'y');
% plot(times, outputs(2,:), 'r');
% plot(times, model(2,:), 'b');
% plot(times, states(2,:), 'black');

xlabel('Time');
ylabel('Position')
%legend('Measurement','Model', 'Reality', 'Kalman Output')




