% RORO Data 
%clear all;

dataRaw = csvread('ms5611.csv', 1, 0);


% figure
% plot(dataRaw(:,1), dataRaw(:,2))
% xlabel('t(m)')
% ylabel('Height (m)')
t=dataRaw(:,1);
h=dataRaw(:,2);
altitude = 44330.8 - 4946.54 * power(h,0.1902632);

figure
plot(t,altitude)
xlabel('t(m)')
ylabel('Height (m)')