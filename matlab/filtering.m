% close all
clear

%-- common --
fs = 48000;         % Sampling frequency                    
Ts = 1/fs;          % Sampling period
fnyq = fs / 2;      % Nyquist frequency
L = 1024;           % Length of signal
t = (0:L-1)*Ts;     % Time vector

%-- signal --
f1 = 100;           % Signal frequency components
f2 = 300;           %
f3 = 900;           %
f4 = 1100;          %
f5 = 1700;          %
f6 = 1900;          %
xn = sin(2*pi*f1*t) + sin(2*pi*f2*t) + sin(2*pi*f3*t) + sin(2*pi*f4*t) + sin(2*pi*f5*t) + sin(2*pi*f6*t);   %Signal

%-- signal fft --
f = fs*(0:(L/2))/L;             % Freq. vector
xn_win = xn'.*hanning(L);       % Windowing signal
Xm = fft(xn_win);               % FFT
P2 = abs(Xm/L);                 %
P1 = P2(1:L/2+1);               % Single sided FFT
P1(2:end-1) = 2*P1(2:end-1);    %
P1_db = 20*log10(P1);           %

%-- decimation filter --
decimation = 8;                     % Decimation factor. 48000Hz -> 6000Hz
B = 2200;                           % Pass band for decimate filter
fs_dec = fs / decimation;           % Sampling frequency after decimation
fnyq_dec = fs_dec / 2;              % Nyquist frequency after decimation
fstop = fs_dec - B;                 % At fstop the attenuation must be aprox 60dB (10 ENOB) to avoid aliasing
n_hdec = 50;                        % Decimation filter order
hdec_k = fir1(n_hdec,(B/fnyq));     % Decimation filter coeficients

%-- decimation filter fft --
Hdecm = fft( [hdec_k zeros(1, L - length(hdec_k))] ); % Single sided FFT
Hdecm_P2 = abs(Hdecm/1);                % Don't divide because filter coefficients are already scaled
Hdecm_P1 = Hdecm_P2(1:L/2+1);           % 
Hdecm_P1(2:end-1) = Hdecm_P1(2:end-1);  %
Hdecm_P1_db = 20*log10(Hdecm_P1);       %

%-- decimate signal --
xn_dec = filter(hdec_k,1,xn);               % Filter signal
xn_dec = downsample(xn_dec, decimation);    % Downlsample signal
t_dec = downsample(t, decimation);          % Decimate time vector

%-- decimate signal fft --
L_dec = length(xn_dec);                 % Lenght of decimated signal
f_dec = fs_dec*(0:(L_dec/2))/L_dec;     % Decimated frequency vector
xn_dec_win = xn_dec'.*hanning(L_dec);   % Windowing
Xm_dec = fft(xn_dec_win);               % Single sided FFT
P2_dec = abs(Xm_dec/L_dec);             %
P1_dec = P2_dec(1:L_dec/2+1);           %
P1_dec(2:end-1) = 2*P1_dec(2:end-1);    %
P1_dec_db = 20*log10(P1_dec);           %

%-- lowpass filter --
hl_fpass = 400;                             % Lowpass filter cutoff=500Hz
n_hl = 50;                                  % Lowpass filter order
hl_k = fir1(n_hl,(hl_fpass/fnyq_dec));      % Lowpass filter coeficients

%-- lowpass filter fft --
Hlm = fft( [hl_k zeros(1, L_dec - length(hl_k))] );     % Lowpass frequency response
Hlm_P2 = abs(Hlm/1);                                    %
Hlm_P1 = Hlm_P2(1:L_dec/2+1);                           %
Hlm_P1(2:end-1) = Hlm_P1(2:end-1);                      %
Hlm_P1_db = 20*log10(Hlm_P1);                           %

%-- lowpass signal --
xn_low = filter(hl_k,1,xn_dec);

%-- lowpass signal fft --
xn_low_win = xn_low'.*hanning(L_dec);   % Windowing
Xm_low = fft(xn_low_win);               % Single sided FFT
P2_low = abs(Xm_low/L_dec);             %
P1_low = P2_low(1:L_dec/2+1);           %
P1_low(2:end-1) = 2*P1_low(2:end-1);    %
P1_low_db = 20*log10(P1_low);           %

%-- bandpass filter --
hb_fpass1 = 800;                            % Bandpass filter cutoff=800Hz
hb_fpass2 = 1200;                           % Bandpass filter cutoff=1200Hz
n_hb = 50;                                  % Bandpass filter order
hb_k = fir1(n_hb,[(hb_fpass1/fnyq_dec) (hb_fpass2/fnyq_dec)]);      % Bandpass filter coeficients

%-- bandpass filter fft --
Hbm = fft( [hb_k zeros(1, L_dec - length(hb_k))] );     % Bandpass frequency response
Hbm_P2 = abs(Hbm/1);                                    %
Hbm_P1 = Hbm_P2(1:L_dec/2+1);                           %
Hbm_P1(2:end-1) = Hbm_P1(2:end-1);                      %
Hbm_P1_db = 20*log10(Hbm_P1);                           %

%-- bandpass signal --
xn_band = filter(hb_k,1,xn_dec);

%-- bandpass signal fft --
xn_band_win = xn_band'.*hanning(L_dec);   % Windowing
Xm_band = fft(xn_band_win);               % Single sided FFT
P2_band = abs(Xm_band/L_dec);             %
P1_band = P2_band(1:L_dec/2+1);           %
P1_band(2:end-1) = 2*P1_band(2:end-1);    %
P1_band_db = 20*log10(P1_band);           %


%-- highpass filter --
hh_fpass = 1600;                            % Highpass filter cutoff=1600Hz
n_hh = 50;                                  % Highpass filter order
hh_k = fir1(n_hh,(hh_fpass/fnyq_dec),'high');      % Highpass filter coeficients

%-- highpass filter fft --
Hhm = fft( [hh_k zeros(1, L_dec - length(hh_k))] );     % Highpass frequency response
Hhm_P2 = abs(Hhm/1);                                    %
Hhm_P1 = Hhm_P2(1:L_dec/2+1);                           %
Hhm_P1(2:end-1) = Hhm_P1(2:end-1);                      %
Hhm_P1_db = 20*log10(Hhm_P1);                           %

%-- highpass signal --
xn_high = filter(hh_k,1,xn_dec);

%-- highpass signal fft --
xn_high_win = xn_high'.*hanning(L_dec);   % Windowing
Xm_high = fft(xn_high_win);               % Single sided FFT
P2_high = abs(Xm_high/L_dec);             %
P1_high = P2_high(1:L_dec/2+1);           %
P1_high(2:end-1) = 2*P1_high(2:end-1);    %
P1_high_db = 20*log10(P1_high);           %



%-- plots --
%figure
subplot(5,4,1)
plot(t, xn)
grid on
title('Signal')
xlabel('t (s)')
ylabel('x(n)')

subplot(5,4,2)
plot(f,P1_db) 
% plot(f,P1) 
grid on
title('Single-Sided Spectrum of Signal')
xlabel('f (Hz)')
ylabel('P1(f) dB')

%-- decimate

subplot(5,4,5)
stem((0:n_hdec), hdec_k)
grid on
title('Decimation filter coefficients')
xlabel('k')
ylabel('hdec(k)')

subplot(5,4,6)
plot(f,Hdecm_P1_db) 
grid on
title('Single-Sided Amplitude Spectrum of Hdec(m)')
xlabel('f (Hz)')
ylabel('Hdecm_P1(db)')

subplot(5,4,7)
plot(t_dec, xn_dec)
grid on
title('Signal decimated')
xlabel('t_dec(s)')
ylabel('xdec(n)')

subplot(5,4,8)
% plot(f_dec,P1_dec) 
plot(f_dec,P1_dec_db) 
grid on
title('Single-Sided Spectrum of Decimated Signal')
xlabel('f (Hz)')
ylabel('P1_dec(f) dB')

%-- lowpas

subplot(5,4,9)
stem((0:n_hl), hl_k)
grid on
title('Lowpass filter coefficients')
xlabel('k')
ylabel('hl_k(k)')

subplot(5,4,10)
plot(f_dec,Hlm_P1_db) 
grid on
title('Single-Sided Spectrum of Hl(m)')
xlabel('f (Hz)')
ylabel('Hl_P1(db)')

subplot(5,4,11)
plot(t_dec, xn_low)
grid on
title('Signal Lowpass')
xlabel('t_dec(s)')
ylabel('xn_low(n)')

subplot(5,4,12)
% plot(f_dec,P1_low) 
plot(f_dec,P1_low_db) 
grid on
title('Single-Sided Spectrum of Lowpass Signal')
xlabel('f (Hz)')
ylabel('P1_low(f) dB')

%-- bandpass

subplot(5,4,13)
stem((0:n_hb), hb_k)
grid on
title('Bandpass filter coefficients')
xlabel('k')
ylabel('hb_k(k)')

subplot(5,4,14)
plot(f_dec,Hbm_P1_db) 
grid on
title('Single-Sided Spectrum of Hb(m)')
xlabel('f (Hz)')
ylabel('Hb_P1(db)')

subplot(5,4,15)
plot(t_dec, xn_band)
grid on
title('Signal Bandpass')
xlabel('t_dec(s)')
ylabel('xn_band(n)')

subplot(5,4,16)
% plot(f_dec,P1_band) 
plot(f_dec,P1_band_db) 
grid on
title('Single-Sided Spectrum of Bandpass Signal')
xlabel('f (Hz)')
ylabel('P1_band(f) dB')



%-- highpas

subplot(5,4,17)
stem((0:n_hh), hh_k)
grid on
title('Highpass filter coefficients')
xlabel('k')
ylabel('hh_k(k)')

subplot(5,4,18)
plot(f_dec,Hhm_P1_db) 
grid on
title('Single-Sided Spectrum of Hh(m)')
xlabel('f (Hz)')
ylabel('Hh_P1(db)')

subplot(5,4,19)
plot(t_dec, xn_high)
grid on
title('Signal Highpass')
xlabel('t_dec(s)')
ylabel('xn_high(n)')

subplot(5,4,20)
% plot(f_dec,P1_low) 
plot(f_dec,P1_high_db) 
grid on
title('Single-Sided Spectrum of Highpass Signal')
xlabel('f (Hz)')
ylabel('P1_high(f) dB')


%-- prints --

%-- input signal
xn_str = sprintf('%ef,', xn);
xn_str = xn_str(1:end-1);
xn_str = strcat('float src[FILTER_DEC_SRC_BLOCK_SIZE] = {', xn_str , '};');

%-- decimation filter coefficients
hdec_k_str = sprintf('%ef,', hdec_k);
hdec_k_str = hdec_k_str(1:end-1);
hedc_k_len_str = sprintf('%d', length(hdec_k));
hdec_k_str = strcat('float filter_dec_coeffs[FILTER_DEC_NTAPS] = {', hdec_k_str , '};');

%-- decimation signal
xn_dec_str = sprintf('%ef,', xn_dec);
xn_dec_str = xn_dec_str(1:end-1);
xn_dec_str = strcat('float test_dst[FILTER_DEC_DEST_BLOCK_SIZE] = {', xn_dec_str , '};');

%-- lowpass filter coefficients
hlow_k_str = sprintf('%ef,', hl_k);
hlow_k_str = hlow_k_str(1:end-1);
hlow_k_len_str = sprintf('%d', length(hl_k));
hlow_k_str = strcat('float filter_low_coeffs[FILTER_LOW_NTAPS] = {', hlow_k_str , '};');

%-- lowpass signal
xn_low_str = sprintf('%ef,', xn_low);
xn_low_str = xn_low_str(1:end-1);
xn_low_str = strcat('float test_low[FILTER_LOW_BLOCK_SIZE] = {', xn_low_str , '};');

%-- bandpass filter coefficients
hband_k_str = sprintf('%ef,', hb_k);
hband_k_str = hband_k_str(1:end-1);
hlband_k_len_str = sprintf('%d', length(hb_k));
hband_k_str = strcat('float filter_band_coeffs[FILTER_BAND_NTAPS] = {', hband_k_str , '};');

%-- bandpass signal
xn_band_str = sprintf('%ef,', xn_band);
xn_band_str = xn_band_str(1:end-1);
xn_band_str = strcat('float test_band[FILTER_BAND_BLOCK_SIZE] = {', xn_band_str , '};');

%-- highpass filter coefficients
hhigh_k_str = sprintf('%ef,', hh_k);
hhigh_k_str = hhigh_k_str(1:end-1);
hhigh_k_len_str = sprintf('%d', length(hh_k));
hhigh_k_str = strcat('float filter_high_coeffs[FILTER_HIGH_NTAPS] = {', hhigh_k_str , '};');

%-- highpass signal
xn_high_str = sprintf('%ef,', xn_high);
xn_high_str = xn_high_str(1:end-1);
xn_high_str = strcat('float test_high[FILTER_HIGH_BLOCK_SIZE] = {', xn_high_str , '};');

%-- create text file
fid = fopen('c_arrays.txt','wt');
fprintf(fid, '%s\n\n', hdec_k_str);
fprintf(fid, '%s\n\n', xn_str);
fprintf(fid, '%s\n\n', xn_dec_str);
fprintf(fid, '%s\n\n', hlow_k_str);
fprintf(fid, '%s\n\n', xn_low_str);
fprintf(fid, '%s\n\n', hband_k_str);
fprintf(fid, '%s\n\n', xn_band_str);
fprintf(fid, '%s\n\n', hhigh_k_str);
fprintf(fid, '%s\n\n', xn_high_str);
fclose(fid);
