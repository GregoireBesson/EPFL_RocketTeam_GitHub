function [ mx, my, mz ] = cal_mag( mx, my, mz )
%CAL_MAG Summary of this function goes here
% biased input
% spherefit
% unbiased output
scatter3(mx, my, mz,'.','DisplayName','Raw Data');
hold on
axis equal
[Center,Radius] = sphereFit([mx my mz]);
[x,y,z] = sphere(50);
x = x*Radius + Center(1);
y = y*Radius + Center(2);
z = z*Radius + Center(3);
lightGrey = 0.8*[1 1 1]; % It looks better if the lines are lighter
surface(x,y,z,'FaceColor', 'none','EdgeColor',lightGrey,...
        'DisplayName','Spherical Regression');

mx = mx-Center(1);
my = my-Center(2);
mz = mz-Center(3);

scatter3(mx, my, mz,'.','r','DisplayName','Centered Data');
[x,y,z] = sphere(50);
x = x*Radius;
y = y*Radius;
z = z*Radius;
lightGrey = 0.8*[0 1 0]; % It looks better if the lines are lighter
surface(x,y,z,'FaceColor', 'none','EdgeColor',lightGrey,...
        'DisplayName','Spherical Regression');
legend show
title('Magnetic Field Vector [\muT]')

end

