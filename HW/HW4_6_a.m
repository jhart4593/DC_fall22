%% s-plane
clear

sigma = [-20:0.1:-5];
w = [-6:0.1:6];

pts = zeros(length(w),length(sigma));

for i = 1:length(sigma)
    for k = 1:length(w)
        pts(k,i) = sigma(i)+w(k)*1i;
    end
end

for g = 1:length(sigma)
    for h = 1:length(w)
        plot(pts(h,g),'b*');
        hold on
    end
end
grid on
title('s-domain')
xlabel('Real')
ylabel('Imaginary')
ylim([-7 7]);
xlim([-25 0]);

%%
%z-domain
clear

syms z

T = 0.1;
s = ((2*z-2)/(T*z+T));

sigma = [-20:0.1:-5];
w = [-6:0.1:6];

pts = zeros(length(w),length(sigma));

for i = 1:length(sigma)
    for k = 1:length(w)
        eqn = s == sigma(i)+w(k)*1i;
        pts(k,i) = vpasolve(eqn,z);
    end
end

for g = 1:length(sigma)
    for h = 1:length(w)
        plot(pts(h,g),'b*');
        hold on
    end
end
grid on
title('z-domain')
xlabel('Real')
ylabel('Imaginary')
ylim([-.4 .4]);
xlim([-.2 .8]);

%%
