clear

%-- common --
fs = 48000;
L = 10*48000;
t = (0:L-1)/fs;

%-- signal --
f1 = 100;           % Signal frequency components
f2 = 300;           %
f3 = 900;           %
f4 = 1100;          %
f5 = 1700;          %
f6 = 1900;          %
xn = sin(2*pi*f1*t) + sin(2*pi*f2*t) + sin(2*pi*f3*t) + sin(2*pi*f4*t) + sin(2*pi*f5*t) + sin(2*pi*f6*t);   %Signal
xn_norm = xn / max(xn);

plot(t, xn)
hold on
plot(t, xn_norm)

audiowrite('signal.wav',xn_norm,fs)