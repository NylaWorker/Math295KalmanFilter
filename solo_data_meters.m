% Read in the data
data = csvread('solo_data.csv', 1, 0);
n = size(data, 1);
% p = plot(data(:, 1), data(:, 2), 'LineWidth', 3);
% cd = [uint8(jet(n)*255) uint8(ones(n,1))].';
% drawnow
% set(p.Edge, 'ColorBinding','interpolated', 'ColorData',cd)

lat = data(:, 1); long = data(:, 2); pitch = data(:,3); roll = data(:,4);
x = 0; y = 0;  % intial x y 

m = length(lat);
coord = ones(m,2);
coord(1,1) = 0; coord(1,2) = 0 ;

for i=1:m-1
    longdif = long(i)-long(i+1);
    
    latdif = lat(i)-lat(i+1);
    coord(i+1,1) =  coord(i,1)+ 111111*longdif;coord(i+1,2) =coord(i,2)+111111*latdif;
end
    

% p2 = plot(coord(:, 1), coord(:, 2), 'LineWidth', 3);
% hold on;
% 
% cd = [uint8(jet(n)*255) uint8(ones(n,1))].';
% 
% drawnow
% set(p2.Edge, 'ColorBinding','interpolated', 'ColorData',cd)
% hold on;


coord2 = ones(m,3);
coord2(1,1) = 0; coord2(1,2) = 0 ; coord2(1,3) = 0 ;

v=0.004;
dt = 1/20;
vdisp = [v*dt; v*dt; 0];
for i=1:m-1
    rotmat = roty(-pitch(i))*rotx(-roll(i));
    change = rotmat*vdisp;
    coord2(i+1,:) = coord2(i,:) + change';
    
%     coord2(i+1,:) = (rotmat*coord2(i,:)')'+vdisp';
%     coord(i+1,1) =  coord(i,1)+ 111111*latdif;coord(i+1,2) =coord(i,2)+111111*longdif;
end

p3 = plot(coord2(:, 1), coord2(:, 2), 'LineWidth', 3);
hold on;

cd = [uint8(jet(n)*255) uint8(ones(n,1))].';

drawnow
set(p2.Edge, 'ColorBinding','interpolated', 'ColorData',cd)
hold on;

