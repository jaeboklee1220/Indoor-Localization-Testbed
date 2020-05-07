% plotting code
hold on
scatter(aa(:,2),aa(:,3))

scatter(b(:,2),b(:,3))
grid on
yticks([0:14])
xticks([-6:6])
legend('Estimated position','True position')
xlabel('X-axis')
ylabel('Y-axis')
box on