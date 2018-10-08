% clear;clc; close all;

% load('SysID_Inverted_Data');

T = 0.002;
Fs = 1/T;             % Sampling frequency
L = length(theta_trunc(:,1)); % Length of signal

U = fft(detrend(theta_trunc(:,1)));
Y = fft(detrend(theta_trunc(:,2)));

P2 = Y./U;
P1 = P2(1:L/2+1);

f = Fs*(0:(L/2))/L;
Gfrd = frd(P1,f*2*pi);
G(1) = fitfrd(Gfrd, 2,0,1,2)
bode(Gfrd, G(1), {0.01,40})

K = [-2.2361 37.6175 -1.5007 3.3789];
DLPF = [1 0
    0 1
    tf([50 0], [1 50]) 0
    0 tf([50 0], [1 50])];

C = K*DLPF

P(1) = (1/G - 1)/C(1);

U = fft(detrend(alpha_trunc(:,1)));
Y = fft(detrend(alpha_trunc(:,2)));

P2 = Y./U;
P1 = P2(1:L/2+1);

f = Fs*(0:(L/2))/L;
Gfrd = frd(P1,f*2*pi);
G(2) = fitfrd(Gfrd, 2)
bode(Gfrd, G(2), {0.01,50*pi})

P(2) = (1/G(2) - 1)/C(2);