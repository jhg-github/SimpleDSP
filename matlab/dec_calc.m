B       = 2000;
fs_old  = 48000;
P_total = 5;    % 5 is optimal
Atten   = 60;

%-- single stage decimation --
fs_new = fs_old / P_total;
fstop = fs_new - B;
fpass = B;
N_single = Atten / ( 22 * ( fstop/fs_old - fpass/fs_old ) )

%-- two stage decimation --
% P_1 = 4;
% P_2 = 2;
% fs_new1 = fs_old / P_1;
% fstop1 = fs_new1 - B;
% N_1 = Atten / ( 22 * ( fstop1/fs_old - fpass/fs_old ) )
% 
% fs_new2 = fs_new1 / P_2;
% fstop2 = fs_new2 - B;
% N_2 = Atten / ( 22 * ( fstop2/fs_new1 - fpass/fs_new1 ) )

%-- final filter --
fpass_f = 500;
fstop_f = 900;
N_f = Atten / ( 22 * ( fstop_f/fs_new - fpass_f/fs_new ) )

N = 2*N_single + N_f
