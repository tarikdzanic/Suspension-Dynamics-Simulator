clear all
close all
data = csvread('output.csv', 1, 0);
frontcurves = csvread('FRONT_curves.csv', 1, 0);
rearcurves = csvread('REAR_curves.csv', 1, 0);

t = data(:,1);
yawgradient = data(:,2);
roll = data(:,3);
FLnf = data(:,4);
FRnf = data(:,5);
RLnf = data(:,6);
RRnf = data(:,7);

figure
plot(frontcurves(:,1), frontcurves(:,2))
hold on
plot(rearcurves(:,1), rearcurves(:,2))
grid on
title('Camber curves')
ylabel('Camber (deg)')
xlabel('\Delta z (in.)')
legend('Front camber curve', 'Rear camber curve')


figure
plot(t,yawgradient)
grid on
title('Yaw gradient response')
ylabel('Yaw gradient (deg/s)')
xlabel('Time (s)')

figure
plot(t,roll)
grid on
title('Roll response')
ylabel('Roll (deg)')
xlabel('Time (s)')


figure
hold on
subplot(4,1,1)
plot(t, FLnf)
title('Front left tire normal force response')
xlabel('Time (s)')
ylabel('Force (lbs)')

subplot(4,1,2)
plot(t, FRnf)
title('Front right tire normal force response')
xlabel('Time (s)')
ylabel('Force (lbs)')

subplot(4,1,3)
plot(t, RLnf)
title('Rear left tire normal force response')
xlabel('Time (s)')
ylabel('Force (lbs)')

subplot(4,1,4)
plot(t, RRnf)
title('Rear left tire normal force response')
xlabel('Time (s)')
ylabel('Force (lbs)')
