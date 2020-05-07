% computing code

a_loc_r = [4.8 4.2
    3.6 4.2
    4.8 0.6
    1.2 5.4
    2.4 4.2
    1.2 1.8
    3.6 0.6
    3.6 3.0
    2.4 1.8
    1.2 0.6
    2.4 5.4
    1.2 4.2
    1.2 3.0
    3.6 1.8
    4.8 3.0
    4.8 1.8
    4.8 5.4
    3.6 5.4
    2.4 0.6];
clc
a_idd = a_id;
a_locc = a_loc;

n_t = 19;
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