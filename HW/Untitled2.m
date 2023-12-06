% Generate a 1Hz plus 6Hz "original" signal
for i=1:501
t(i)=0.001*(i-1); % Use a time step of 1 msec
end
x=1.0*sin(2*pi*t)+0.2*cos(6*2*pi*t);
% sample the "original" signal with sample rate of 100 Hz (T = 10 msec)
for i=1:51
t_sample(i) = t(1+10*(i-1));
x_sample(i) = x(1+10*(i-1));
end
% Define Nyquist frequency
wN=100*pi;
% Reconstruct the signal using the interpolation formula.
% Reconstruct original signal at the original 1 msec interval
% using 10 msec sampled data.
% j is index for ‘continuous time’ (1000Hz)
% k is index for discrete time (100Hz)
for j=1:501
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
