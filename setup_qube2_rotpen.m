%% Load Model
qube2_rotpen_param;
% Set open-loop state-space model of rotary single-inverted pendulum (SIP)
QUBE2_ROTPEN_ABCD_eqns;
% Display matrices
A;
B;

Q = diag([5,1,1,1]);
R = 1;

K = lqr(A,B,Q,R)
