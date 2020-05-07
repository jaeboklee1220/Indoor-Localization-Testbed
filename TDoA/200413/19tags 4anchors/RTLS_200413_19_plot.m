% computing code
mae_mat=[a_loc_r mae'];
close all
hold on

scatter(a_loc_r(:,1),a_loc_r(:,2),'filled')
scatter(m(:,1),m(:,2),80,'x')

err = mae_mat(:,3);
err = floor(100*err)/100;

textt = 1:19;

anchor = [-0.6 -1.1
    7.2 -1.1
    6.93 6.6
    -0.6 6];
scatter(anchor(:,1),anchor(:,2),100,'filled')

% text(mae_mat(:,1)-0.1,mae_mat(:,2)-0.1,num2str(err))
text(m(:,1)+0.1,m(:,2)-0.1,num2str(err))
text(m(:,1)-0.35,m(:,2)-0.1,'MAE = ')

text(mae_mat(:,1)-0.08,mae_mat(:,2)+0.15,[num2str(textt')])

box on 
grid on
axis([-1.2 7.2 -1.2 7.2])
xlabel('X-axis')
ylabel('Y-axis')
legend('True position','Estimated position','Anchors')
xticks([-2.4:0.6:8.4])
yticks([-2.4:0.6:8.4])

