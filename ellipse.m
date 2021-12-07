clc;
%xc=50; %xCenter
xc = input('Enter the coord of xCenter: ');
%yc=50; %yCenter
yc = input('Enter the coord of yCenter: ');
a=25; %xRad
b=100; %yRad
m = 1000;
x = zeros(m,1);
y = zeros(m,1);
theta = linspace(0,2*pi,m);
for k = 1:m
        x(k) = a * cos(theta(k));
        y(k) = b * sin(theta(k));
end
alpha = input('Enter the rotation angle: ');
R  = [cos(alpha) -sin(alpha); ...
      sin(alpha)  cos(alpha)];
rCoords = R*[x' ; y'];   
xr = rCoords(1,:)';      
yr = rCoords(2,:)';      
plot(x+xc,y+yc,'r');
grid on;
hold on;
axis equal;
plot(xr+xc,yr+yc,'b');
xAgent = xr + xc;
yAgent = yr + yc;
plot(xAgent(1), yAgent(1), 'h');