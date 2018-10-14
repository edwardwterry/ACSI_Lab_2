input = u_ilc_input;
u_max = max(input(:,2));
u_min = min(input(:,2));
resolution = (u_max - u_min) / 256;
input(:,2) = input(:,2) - u_min;
input(:,2) = round(input(:,2) / resolution);
u_ilc_ard = input(:,2);

test = input(:,2) * resolution + u_min;
