% Script to calculate the natural frequency and damping ratio of 

clear
close all

load('motor_held.mat')

alpha = detrend(theta);
Ts = 0.002;
t = [0:Ts:(length(alpha)-1)*Ts]';

[pks,locs] = findpeaks(alpha, 500, 'MinPeakHeight',0.0, 'MinPeakDistance',0.5);
f = fit(locs, pks, 'exp1');
figure
hold on
plot(t, alpha)
plot(locs,pks,'o')
plot(t, f.a*exp(f.b*t))
xlabel('Time (s)')
ylabel('Pendulum angle (rad)')
title('Free-swinging pendulum response')

period = sum(locs(end)-locs(1))/length(locs);
wn_Hz = 1/period;
wn_rad_sec = wn_Hz * 2*pi;

delta = log(pks(6)/pks(5)); % log decrement
zeta = 1/(sqrt(1+(2*pi/delta)^2));

wd = wn_rad_sec * sqrt(1 - zeta^2);