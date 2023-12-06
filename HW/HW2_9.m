k = [2:21];
x = 1.25-1.25*(0.2).^(k-2);

stairs(k,x)

xlabel('k')
ylabel('x(k)')