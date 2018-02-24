% Read in the data
data = csvread('solo_data.csv', 1, 0);
n = size(data, 1);
p = plot(data(:, 1), data(:, 2), 'LineWidth', 3);


cd = [uint8(jet(n)*255) uint8(ones(n,1))].';

drawnow
set(p.Edge, 'ColorBinding','interpolated', 'ColorData',cd)