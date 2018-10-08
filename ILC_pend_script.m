setup_qube2_rotpen %setup SS model and lqr controller K
Cp = [1 0 0 0;
      0 1 0 0];
Dp = [0; 0];
sysSS = ss(A,B,Cp,Dp);

g1 = [1 0;
      0 1;
      tf([50 0], [1 50]) 0;
      0 tf([50 0], [1 50])]
Ctrl = K*g1;

T = 0.002;
Ctrld = c2d(Ctrl,T)
Pd = c2d(sysSS,T) %converting to discrete system
ztf = tf('z',T);
z = ss(ztf);
alpha = 0.0001;
temp1 = Pd*Ctrld
Plant = (eye(2)-feedback(Pd*Ctrld,eye(2)))*Pd;

temp = 1-z*alpha*Plant
Hnorm = norm(temp, 'inf')

time = [0:T:10]';
length(time)
u_ilc = [time zeros(length(time),1)];

N = 50;
L = [K(1) K(2)];
e = [];
for iN = 1:N
    sim('ILC_pend');
    e = [e;std(Response(1:end-1,2)-Response(1:end-1,3))];
    for ii = 1:length(time)-1
        u_ilc(ii,2) = u_ilc(ii,2)+alpha*L*[(Response(ii+1,2)-Response(ii+1,3)); Res2(ii+1,2)-Res2(ii+1,3)];
    end
end
figure(2)
plot(Response(1:end-1,1),Response(1:end-1,2)-Response(1:end-1,3))
figure(3)
plot(e)     