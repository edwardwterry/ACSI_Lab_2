alpha = 2;
count = count + 1;
e(count-1,1)= std(err(1:end,1)-err(1:end,2));
for ii = 2:length(time)-1
    u_ilc(:,1,count) = time;
    u_ilc(ii,2,count) = u_ilc(ii,2,count-1)+alpha*[(err(ii,1)-err(ii,2))];
end
u_ilc_input = u_ilc(:,:,count);
