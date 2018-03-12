% Read in the data
data = csvread('solo_data.csv', 1, 0);
n = size(data, 1);

xnoise = normrnd(0, 0.0000005, n, 1);
ynoise = normrnd(0, 0.0000005, n, 1);

p = plot(data(:, 3) , data(:, 4), 'LineWidth', 3);


cd = [uint8(autumn(n)*255) uint8(ones(n,1))].';

drawnow
set(p.Edge, 'ColorBinding','interpolated', 'ColorData',cd);
title('Attitude')
xlabel('Roll Angle')
ylabel('Pitch Angle')
