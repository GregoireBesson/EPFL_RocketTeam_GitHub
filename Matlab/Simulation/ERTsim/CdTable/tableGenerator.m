clc
clear all
close all

load('Cd0.mat')
load('Cd01.mat')
load('Cd02.mat')
load('Cd03.mat')
load('Cd04.mat')
load('Cd05.mat')
load('Cd06.mat')
load('Cd07.mat')
% load('Cd08.mat')
% load('Cd09.mat')
% load('Cd1.mat')

[~,I] = max(s0 ); s0  = s0 (I:end); h0  = h0 (I:end);
[~,I] = max(s01); s01 = s01(I:end); h01 = h01(I:end);
[~,I] = max(s02); s02 = s02(I:end); h02 = h02(I:end);
[~,I] = max(s03); s03 = s03(I:end); h03 = h03(I:end);
[~,I] = max(s04); s04 = s04(I:end); h04 = h04(I:end);
[~,I] = max(s05); s05 = s05(I:end); h05 = h05(I:end);
[~,I] = max(s06); s06 = s06(I:end); h06 = h06(I:end);
[~,I] = max(s07); s07 = s07(I:end); h07 = h07(I:end);
% [~,I] = max(s08); s08 = s08(I:end); h08 = h08(I:end);
% [~,I] = max(s09); s09 = s09(I:end); h09 = h09(I:end);
% [~,I] = max(s1 ); s1  = s1 (I:end); h1  = h1 (I:end);


cd0 = 0*ones(length(s0),1);
cd01 = 0.1*ones(length(s01),1);
cd02 = 0.2*ones(length(s02),1);
cd03 = 0.3*ones(length(s03),1);
cd04 = 0.4*ones(length(s04),1);
cd05 = 0.5*ones(length(s05),1);
cd06 = 0.6*ones(length(s06),1);
cd07 = 0.7*ones(length(s07),1);
% cd08 = 0.8*ones(length(s08),1);
% cd09 = 0.8*ones(length(s09),1);
% cd1 = ones(length(s1),1);

hold on
grid on
scatter3(h0,s0,cd0,'.');
scatter3(h01,s01,cd01,'.');
scatter3(h02,s02,cd02,'.');
scatter3(h03,s03,cd03,'.');
scatter3(h04,s04,cd04,'.');
scatter3(h05,s05,cd05,'.');
scatter3(h06,s06,cd06,'.');
scatter3(h07,s07,cd07,'.');
% scatter3(h08,s08,cd08,'.');
% scatter3(h09,s09,cd09,'.');
% scatter3(h1,s1,cd1,'.');
view(-60,60)

legend('Cd = 0','Cd = 0.1','Cd = 0.2','Cd = 0.3','Cd = 0.4','Cd = 0.5','Cd = 0.6','Cd = 0.7') 
xlabel('distance to apogee [m]')
ylabel('speed [m/s]')
zlabel('Cd')

%%
figure
hold on
grid on
scatter(h0,s0,'.');
scatter(h01,s01,'.');
scatter(h02,s02,'.');
scatter(h03,s03,'.');
scatter(h04,s04,'.');
scatter(h05,s05,'.');
scatter(h06,s06,'.');
scatter(h07,s07,'.');
% scatter(h08,s08,cd08,'.');
% scatter(h09,s09,cd09,'.');
% scatter(h1,s1,cd1,'.');

legend('Cd = 0','Cd = 0.1','Cd = 0.2','Cd = 0.3','Cd = 0.4','Cd = 0.5','Cd = 0.6','Cd = 0.7') 
xlabel('distance to apogee [m]')
ylabel('speed [m/s]')

%% resample

Cd = [0 .1 .2 .3 .4 .5 .6 .7]
vmax = min([max(s0) max(s01) max(s02) max(s03) max(s04) max(s05) max(s06) max(s07)]);
vmin = 0;

v = vmin:1:vmax;
table(:,1) = interp1(s0, h0, v)';
table(:,2) = interp1(s01, h01, v)';
table(:,3) = interp1(s02, h02, v)';
table(:,4) = interp1(s03, h03, v)';
table(:,5) = interp1(s04, h04, v)';
table(:,6) = interp1(s05, h05, v)';
table(:,7) = interp1(s06, h06, v)';
table(:,8) = interp1(s07, h07, v)';
table(isnan(table)) = 0;

%% test

currentV = 160;
currentH = 100;

iv = find(v>currentV,1,'first');
'closest higher speed in the table:'
v(iv)
'heigth available:'
table(iv,:)
iCd = find(table(iv,:)<currentH,1,'first')
if length(iCd) == 0
    iCd = 8;
end
'choosen heigth:'
table(iv,iCd)
'corresponding Cd:'
Cd(iCd)




