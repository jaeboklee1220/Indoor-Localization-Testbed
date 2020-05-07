% computing code
clc;clear;
run('RTLS_200504_4_8_id.m')
run('RTLS_200504_4_8_loc.m')

a_loc_r = [10.54 11.21
    4.54 11.21
    10.54 5.21
    4.54 5.21];
clc
a_idd = a_id;
a_locc = a_loc;

n_t = 4;
id_size = size(a_id);
id_len = id_size(2);
mae_v = [];
for k = 1:n_t
    id = a_idd(1,:);
    b = a_idd == id;
    b = sum(b,2);
    b = b == id_len;
    eval(['a' num2str(k) ' = a_locc(b,:);'])
    a_idd = a_idd(~b,:);
    a_locc = a_locc(~b,:);
    eval(['v' num2str(k) ' = var(a' num2str(k) ');'])
    eval(['m' num2str(k) ' = mean(a' num2str(k) ');'])
    eval(['ma = (a' num2str(k) ' - a_loc_r(k,:));'])
%     if k == 3
%         eval(['mak = (a' num2str(k) ' - a_loc_r(k,:));'])
%     end
    eval(['mae' num2str(k) ' = mean(hypot(ma(:,1),ma(:,2)));';])
    eval(['mae_v = [mae_v;hypot(ma(:,1),ma(:,2))];';])
    
end

m = [];
v = [];
mae = [];

for k = 1:n_t
    eval(['m = [m; m' num2str(k) '];'])
    eval(['v = [v v' num2str(k) '];'])
    eval(['mae = [mae mae' num2str(k) '];'])
end

mae