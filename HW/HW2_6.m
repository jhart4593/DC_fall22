% Generate a 0.95Hz "original" signal
for i=1:401
t(i)=0.05*(i-1); % Use a time step of 50 msec
end
x=1.0*sin(0.95*2*pi*t);
% sample the "original" signal with sample rate of 2.0 Hz (T = 500 msec)
for i=1:41
t_sample(i) = t(1+10*(i-1));
x_sample(i) = x(1+10*(i-1));
end
% Define Nyquist frequency
wN=2*pi;
% Reconstruct the signal using the interpolation formula.
% Reconstruct original signal at the original 1 msec interval
% using 500 msec sampled data.
% j is index for ‘continuous time’ (20Hz)
% k is index for discrete time (2Hz)
for j=1:401
x_reconstruct(j)=0;
for k=1:length(t_sample)
temp = wN * (t(j)-t_sample(k));
% Use this if-statement to avoid divide-by-zero
if abs(temp) > 1e-9 
x_reconstruct(j) = x_reconstruct(j) + ...
(x_sample(k)*sin(temp)/temp);
else
x_reconstruct(j) = x_reconstruct(j) + x_sample(k);
end
end
end
% Plot the results
plot(t,x,'b-',t,x_reconstruct,'r-.',t_sample,x_sample,'d');
xlabel('Time (sec)');ylabel('Amplitude');
legend('Original Signal','Reconstructed Signal','Sampled Signal');