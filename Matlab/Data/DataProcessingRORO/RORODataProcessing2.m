clear; clc;
%%Mag data
datamagRaw = csvread('hmc5883l.csv', 1, 0);

%%
meanx = (max(datamagRaw(:,2))+min(datamagRaw(:,2)))/2;
meany = (max(datamagRaw(:,3))+min(datamagRaw(:,3)))/2;
meanz = (max(datamagRaw(:,4))+min(datamagRaw(:,4)))/2;
%

liftoff = 2399.3;
touchdown = 81.8500;
separation = 10.4700;
burnout = 1.7;
toffset= liftoff- 40;
t = datamagRaw(:,1)-toffset; 
% relevent data 2399-40 sec after turning on chip
indStart = find(t>0,1); 
indEnd = find(t>52,1);
%cliped data
datamag = datamagRaw(indStart:indEnd,:);
t=datamag(:,1)-toffset;
indextEnd = length(t);
mx=datamag(:,2)-meanx;
my=datamag(:,3)-meany;
mz=datamag(:,4)-meanz;

mag = zeros(indextEnd,1);
for i=1:indextEnd
    mag(i) = sqrt(mx(i)^2+my(i)^2+mz(i)^2);
end


figure
plot(t,mx); 
hold on;  
plot(t,my); 
plot(t,mz);

plot(t,mag);
line([separation+40  separation+40],[-12e-3 6e-3],'Color',[0 1 0])
line([40  40],[-12e-3 6e-3],'Color',[1 0 0])
line([burnout+40  burnout+40],[-12e-3 6e-3],'Color',[0 0 1])
legend('x','y','z','mag')




%% Calcualtes the mean vector
bias_t = 35;
ind= find(t>bias_t, 1);
magMean = [mean(mx(1:ind)); mean(my(1:ind)); mean(mz(1:ind))];
%plotVector(magMean)

% for i= 1:indextEnd
%     
%     if t(i)>40
%         plotVector([mx(i); my(i); mz(i)]);
%         pause(0.01);
%     end
% end

%
