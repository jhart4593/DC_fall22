%% s-plane
clear
syms s

zeta = [0.5:0.01:0.9];
w_n = [0:0.1:20];

pts = cell(length(w_n),length(zeta));

for i = 1:length(zeta)
    for j = 1:length(w_n)
        eqn = s^2+2*zeta(i)*w_n(j)*s+w_n(j)^2 == 0;

        pts{j,i} = vpasolve(eqn,s);
    end
end

for g = 1:length(zeta)
    for h = 1:length(w_n)
        plot(pts{h,g}(1,1),'b*');
        hold on
        plot(pts{h,g}(2,1),'b*');
        hold on
    end
end
grid on
title('s-domain')
xlabel('Real')
ylabel('Imaginary')
xlim([-20 0]);



%%
%z-plane
clear
syms z

T = 0.1;
s = ((2*z-2)/(T*z+T));
zeta = [0.5:0.01:0.9];
w_n = [0:0.1:20];

pts = cell(length(w_n),length(zeta));

for i = 1:length(zeta)
    for j = 1:length(w_n)
        eqn = s^2+2*zeta(i)*w_n(j)*s+w_n(j)^2 == 0;

        pts{j,i} = vpasolve(eqn,z);
    end
end

for g = 1:length(zeta)
    for h = 1:length(w_n)
        plot(pts{h,g}(1,1),'b*');
        hold on
        plot(pts{h,g}(2,1),'b*');
        hold on
    end
end
grid on
title('z-domain')
xlabel('Real')
ylabel('Imaginary')


%%





