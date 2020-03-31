fs=48000;
Ts=1/fs;
fc=0.1;
L = 5*fs;
t = (0:L-1)*Ts;
alpha=(2*pi*Ts*fc)/(2*pi*Ts*fc+1)
alpha=13/2^17

h_num=alpha;
h_den=[1 (alpha-1)];
figure(1)
freqz(h_num,h_den,fs,fs);


xn = ones(1,L);
f1 = 100;           % Signal frequency components
f2 = 300;           %
f3 = 900;           %
f4 = 1100;          %
f5 = 1700;          %
f6 = 1900;          %
xn = 2 + sin(2*pi*f1*t) + sin(2*pi*f2*t) + sin(2*pi*f3*t) + sin(2*pi*f4*t) + sin(2*pi*f5*t) + sin(2*pi*f6*t);   %Signal

xn_filt = filter(h_num,h_den,xn);
figure(2)
plot(t,xn)
hold on
plot(t,xn_filt)
hold off
