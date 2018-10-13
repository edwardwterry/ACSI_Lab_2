T = 0.002;
time = [0:T:10]';
count = 1;
u_ilc = [];
u_ilc(:,:,count) = [time zeros(length(time),1)];
e = [];
u_ilc_input = u_ilc(:,:,1);
