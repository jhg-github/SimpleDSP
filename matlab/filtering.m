close all
clear

%-- common --
Fs = 48000;           % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 480;             % Length of signal
t = (0:L-1)*T;        % Time vector
f1 = 200;
f2 = 400;
f3 = 1000;
f4 = 1200;
f5 = 1800;
f6 = 2000;
hlow_fpass = 600;
hlow_fstop = 800;
hband_fstop1 = 600;
hband_fpass1 = 800;
hband_fpass2 = 1400;
hband_fstop2 = 6600;
hhigh_fstop = 1400;
hhigh_fpass = 1600;
nfir = 100;

%-- signal --
xn = sin(2*pi*f1*t) + sin(2*pi*f2*t) + sin(2*pi*f3*t) + sin(2*pi*f4*t) + sin(2*pi*f5*t) + sin(2*pi*f6*t);

%-- signal fft --
f = Fs*(0:(L/2))/L;
Xm = fft(xn);
P2 = abs(Xm/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

%-- lowpass filter --
hl_f =   [0 (hlow_fpass/Fs) (hlow_fstop/Fs) 1];
hl_mag = [1 1               0               0];
hl_k = fir2(nfir,hl_f,hl_mag);

%-- lowpass filter fft --
Hlm = fft( [hl_k zeros(1, L - length(hl_k))] );
Hlm_P2 = abs(Hlm/1);    % don't divide because filter coefficients are already scaled ???
Hlm_P1 = Hlm_P2(1:L/2+1);
Hlm_P1(2:end-1) = Hlm_P1(2:end-1);


%-- plots --
figure
subplot(5,2,1)
plot(t, xn)
grid on
title('Signal')
xlabel('t (s)')
ylabel('x(n)')

subplot(5,2,2)
plot(f,P1) 
grid on
title('Single-Sided Amplitude Spectrum of Signal')
xlabel('f (Hz)')
ylabel('|P1(f)|')

subplot(5,2,3)
stem((0:nfir), hl_k)
grid on
title('Lowpass filter coefficients')
xlabel('k')
ylabel('hl(k)')

subplot(5,2,4)
plot(f,Hlm_P1) 
grid on
title('Single-Sided Amplitude Spectrum of Hl(m)')
xlabel('f (Hz)')
ylabel('|Hlm_P1(f)|')
