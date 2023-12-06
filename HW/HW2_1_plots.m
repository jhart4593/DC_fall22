tiledlayout(2,1)

%50Hz
nexttile
plot(out.y_cont_50,'b')
hold on
plot(out.y_digital_50,'r-o')
xlim([0 2])
title('50 Hz')
xlabel('Time (sec)')
ylabel('Unit Step Response')
grid on
legend('Continuous','Discrete')

%10Hz
nexttile
plot(out.y_cont_10,'b')
hold on
plot(out.y_digital_10,'r-o')
xlim([0 2])
ylim([-100 100])
title('10 Hz')
xlabel('Time (sec)')
ylabel('Unit Step Response')
grid on
legend('Continuous','Discrete')