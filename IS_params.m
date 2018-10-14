wn = 11.3*1.2;
zeta = 0.0032;

K = exp(-zeta*pi/sqrt(1-zeta^2));
A0 = 1/(1+2*K+K^2);
A1 = 2*K/(1+2*K+K^2);
A2 = K^2/(1+2*K+K^2);
wd = wn*sqrt(1-zeta^2);
t1 = pi/wd;
t2 = 2*pi/wd;