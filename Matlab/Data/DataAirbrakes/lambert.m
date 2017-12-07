clc
clear all 
close all

g = 9.81;
% speed in m/s
v = 1:10:350;
% alttidue in m
h = 1:10:2500;
%h = linspace(0,2500,length(v));
m = 17.6;
rho = 1.225;
Aref = pi*(0.12/2)^2;
factor = (2*m)/(rho*Aref);

for i=1:length(v) 
    for j=1:length(h)
        arg = (-2*g*h(j)*exp(-2*g*h(j)/(v(i)^2) ))/(v(i)^2);
        Cd(i,j) = -factor*lambertw(0, arg )/(2*h(j)) - g/v(i)^2;
    end
end


   
%%
surf(h,v,real(Cd))
xlabel('h');
ylabel('v');
zlabel('Cd');
zlim([-2 10]);
caxis([-2 10]);

