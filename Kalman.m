varR = 0.5;
R = [varR 0; 0 varR];

varQ = 0.01;
Q = [varQ 0; 0 varQ];

mass = 10;

t = 0;
dt = 0.001;
x = [0; 10];
xHat = [0; 10];
F = [1 dt; 0 1];
B = [dt^2/2; dt];
H = [1 0; 0 1];
P = [-10 0; 0 1];

xModel = [0; 10];

times = [];
states = [];
measurements = [];
outputs = [];
model = [];

while x(2) > 0
    % State update
    t = t + dt;
    u = -20/mass;
    x = F*x + B*u + normrnd(0, diag(Q));
    times = [times t];
    states = [states x];
    
    % Model
    xModel = F*xModel + B*u
    model = [model xModel];
    
    % Kalman gain
    K = P*H.'*inv(H*P*H.' + R);
    
    % Measurement
    z = H * x + normrnd(0, diag(R));
    measurements = [measurements z];
    
    % Update estimate
    xHat = xHat + K*(z - H*xHat);
    
    % Update covariance
    P = (eye(2) - K*H)*P;
    
    outputs = [outputs xHat];
    
    % Predict next step
    xHat = F*xHat;
    P = F*P*F.' + Q;
end

hold on;

% Position
plot(times, measurements(1,:), 'y');
plot(times, model(1,:), 'b');
plot(times, states(1,:), 'black');
plot(times, outputs(1,:), 'r');


% Velocity
% plot(times, measurements(2,:), 'y');
% plot(times, outputs(2,:), 'r');
% plot(times, model(2,:), 'b');
% plot(times, states(2,:), 'black');

xlabel('Time');
ylabel('Position')
legend('Measurement','Model', 'Reality', 'Kalman Output')




