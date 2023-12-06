tiledlayout(2,2)

%10Hz
nexttile
plot(out.y,'b')
hold on
plot(out.y_10hz_digital,'g-o')
title('')
xlabel('Time (sec)')
ylabel('Unit Step Response')
grid on
legend('Analog Control','Digital Control (10Hz)')

%30Hz
nexttile
plot(out.y,'b')
hold on
plot(out.y_30hz_digital,'g-x')
title('')
xlabel('Time (sec)')
ylabel('Unit Step Response')
grid on
legend('Analog Control','Digital Control (30Hz)')

%5Hz 1
nexttile
plot(out.y,'b')
hold on
plot(out.y_5hz_digital,'g-h')
title('')
xlabel('Time (sec)')
ylabel('Unit Step Response')
grid on
legend('Analog Control','Digital Control (5Hz)')

%5Hz 2
nexttile
plot(out.y,'b')
hold on
plot(out.y_5hz_digital,'g-h')
hold on
plot(out.y_5hz_cont,'r')
title('')
xlabel('Time (sec)')
ylabel('Unit Step Response')
grid on
legend('Analog Control','Sampled Digital Output (5Hz)','Actual Output')
