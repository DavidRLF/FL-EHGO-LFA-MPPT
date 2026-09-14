clc; clear all; close all;

%% PV array based on Canadian Solar CS6X-280P solar modules
No_Cadenas_Paralelo = 2;
No_Modulos_PorCadena = 18;
Irr_in=0;
Npar=No_Cadenas_Paralelo;
impp_STC = 15.72/2;
G_STC = 1000;

%% Boost converter parameters and operating point
% PV array (T = 25, Pmp = 10.07 kW, Vmp = 640.8 V)
vpv = 640.8;       % Minimum input voltage
vdc = 1000;        % Desired output voltage (dc-bus voltage)
Fs = 5e3;          % Switching frequency
Ppv = 10.07e3;     % Input power
ipv = Ppv/vpv;     % Input current
iL = ipv;          % The inductor current is assumed equal to ipv
d =(vdc-vpv)/vdc;  % Duty cycle
Rpv = vpv/ipv;     % Rpv=Rmppt
Idc = ipv*(1-d);   % Load-current calculation
Rdc = vdc/(Idc);   % Load-resistance calculation
Lmin =(d*(1-d)^2*Rdc)/(2*Fs); % Minimum inductance required for continuous conduction mode (CCM)
L = 100e-3;        % Selected to ensure CCM operation
ILmax = vpv/((1-d)^2*Rdc)+(vpv*d)/(2*L*Fs); % Maximum inductor current
ILmin = vpv/((1-d)^2*Rdc)-(vpv*d)/(2*L*Fs)  % Minimum inductor current
Cdc = 470e-6;               % Output capacitor
dVdc = (d*vdc)/(Rdc*Cdc*Fs) % Output-voltage ripple of 1 V
Cpv = 200e-6;                % Input capacitor; Cpv = Cin
b = 16;                     % Number of DPWM bits
Ucmax = (2^b-1);            % Maximum control signal (0-Ucmax)
Kdpwm = 1/Ucmax;            % DPWM gain (digital PWM modulator)
UC = d/Kdpwm;               % Define the operating point for UC
X1 = iL;                    % Define the operating point for state X1
X2 = vdc;                   % Define the operating point for state X2
Ref = ipv;                  % Reference to be tracked by the controller

%% Continuous-time open-loop model
% x1 = ipv (PV-array current), x2 = vdc (output voltage)
% State-space matrices
Amc = [0 -(1-Kdpwm*UC)/L
      (1-Kdpwm*UC)/Cdc -1/(Cdc*Rdc)];
Bmc = [(Kdpwm*X2)/L
      -(Kdpwm*X1)/Cdc];
Cmc = [1/2 0     % To control x1 = ipv = iL
      0 1];      % To control x2 = vdc
Dmc = [0
      0];
states = {'x1' 'x2'};
inputs = {'uc'};
outputs = {'x1=ipv' 'x2=vdc'};
Sys_Boost_Modelo_Continuo = ss(Amc,Bmc,Cmc,Dmc,'statename',states,'inputname',inputs,'outputname',outputs)
tf(Sys_Boost_Modelo_Continuo)
Gs_uc_to_x1 = (ans(1,1))
Gs_uc_to_x2 = (ans(2,1))
[num_Gs_uc_to_x1, den_Gs_uc_to_x1] = tfdata(Gs_uc_to_x1, 'v');
[num_Gs_uc_to_x2, den_Gs_uc_to_x2] = tfdata(Gs_uc_to_x2, 'v');

%% Discrete-time open-loop model of the Boost converter
% x1 = ipv (PV-array current), x2 = vdc (output voltage)
% State-space matrices
Ts1 = 1/Fs;
Amd = [1 (Ts1*(Kdpwm*UC-1))/L
   -(Ts1*(Kdpwm*UC-1))/Cdc 1-Ts1/(Cdc*Rdc)];
Bmd = [(Kdpwm*Ts1*X2)/L
    -(Kdpwm*Ts1*X1)/Cdc];
Cmd = [1/2 0      % To control x1 = ipv = iL
       0 1];      % To control x2 = vdc
Dmd = [0
      0];
states = {'x1' 'x2'};
inputs = {'uc'};
outputs = {'x1=ipv' 'x2=vdc'};
Sys_Boost_Modelo_Discreto = ss(Amd,Bmd,Cmd,Dmd,Ts1,'statename',states,'inputname',inputs,'outputname',outputs)
tf(Sys_Boost_Modelo_Discreto)
Gz_uc_to_x1=(ans(1,1));
[num_Gz_uc_to_x1, den_Gz_uc_to_x1] = tfdata(Gz_uc_to_x1, 'v');

%% Discrete-time Boost converter control design with SF + integral action
% System
Cmd_Control = [1 0]; % Control of y = x1 = ipv
Dmd_Control = [0];
Amd = [1 -(Ts1*(1-Kdpwm*UC))/L
   (Ts1*(1-Kdpwm*UC))/Cdc 1-Ts1/(Cdc*Rdc)];
Bmd = [(Kdpwm*Ts1*X2)/L
    -(Kdpwm*Ts1*X1)/Cdc];

% Augmented matrices Aamd and Bamd
% Aamd for the Simulink environment using a block-based model.
% Since the integral action is represented by a third state, it is
% explicitly implemented using an accumulator block with sampling-time
% gain Ts according to x3[k+1]=x3[k]+Ts(X2ref[k]-x2(k))
Aamd = [ 1                        -(Ts1*(1-Kdpwm*UC))/L             0;
        (Ts1*(1-Kdpwm*UC))/Cdc   1-Ts1/(Cdc*Rdc)                    0;
        -Ts1                       0                                1 ];

% Aamd for the script environment, since the integral action is represented
% by a third state according to x3[k+1]=x3[k]+(X2ref[k]-x2(k))
% without explicitly including Ts because it is already considered in ss(..)
Aamdprima = [ 1                        -(Ts1*(1-Kdpwm*UC))/L             0;
             (Ts1*(1-Kdpwm*UC))/Cdc   1-Ts1/(Cdc*Rdc)                  0;
             -1                       0                                1 ];
Bamd = [ (Kdpwm*Ts1*X2)/L;
         -(Kdpwm*Ts1*X1)/Cdc;
         0 ];
Eamd = [zeros(size(Aamd,1)-1,1); 1];
Camd = [Cmd_Control 0];

% Desired poles in the z-domain (converted from continuous time)
Testd = 0.0036*2.8;      % 0.0036*2.8, no overshoot (test with rt = 11.3 ms)
zeta_d = 1.0;
p3 = 40;                 % 40, 0.0036*2.8, no overshoot (test with rt = 11.3 ms)
wn_d = 4 / (zeta_d * Testd);
s1 = -zeta_d * wn_d;
s2 = -zeta_d * wn_d;
s3 = -p3;
z1 = exp(s1 * Ts1);
z2 = exp(s2 * Ts1);
z3 = exp(s3 * Ts1);
Polos_Des_Boost_Mod_Disc = [z1 z2 z3];   % Following the design method

% If the trial-and-error method is used
% z1p = exp(s1p * Ts1); % If the trial-and-error method is used
% z2p = exp(s2p * Ts1); % If the trial-and-error method is used
% z3p = exp(s3p * Ts1); % If the trial-and-error method is used
% Polos_Des_Boost_Mod_Disc = [z1p z2p z3p]; % If the trial-and-error method is used

% Check controllability
Co_Boost_Modelo_Disc = ctrb(Aamd, Bamd);
if rank(Co_Boost_Modelo_Disc) < size(Aamd,1)
    error('The augmented system is not fully controllable.')
end

% Compute the gains using acker as implemented in the Simulink environment
Kmd = acker(Aamd, Bamd, Polos_Des_Boost_Mod_Disc);
K1y2md = Kmd(1:end-1) % For closed-loop implementation in Simulink

% Reverse the sign of Ki to avoid an opposite integral action,
% since the error is defined as Vref-Vo. If the error were defined
% as Vo-Vref, this sign reversal would not be necessary.
Kimd = -Kmd(end) % For closed-loop implementation in Simulink using a block-based model
                 % When a MATLAB Function block is used in Simulink,
                 % Kimd must be multiplied by Ts due to the formulation
                 % adopted in Aamd and the implementation of the
                 % integrator in the function block as:
                 % Acc_Int = Ki * ek * Ts + Acc_Int_1;

% Compute the gains using acker as implemented in the script environment
Kmdprima = acker(Aamdprima, Bamd, Polos_Des_Boost_Mod_Disc); % As used in the script

% Determine the closed-loop system for evaluation in the script
Afmd = Aamdprima-Bamd*Kmdprima;

% Compute the closed-loop eigenvalues (denominator roots) using the Kmd
% gains from the Simulink environment. These must match the desired poles.
Polos_Boost_Lazo_Cerrado_Disc = eig(Aamd-Bamd*Kmd)

% Closed-loop Boost converter with state feedback using Kmdprima
% for the script environment
SLC_Boost_m_SF_Disc = ss(Afmd,Eamd,Camd,0,Ts1);

% Open- and closed-loop responses of the Boost converter with SF
% and integral action
figure(1)

% Open loop (x1 vs u1)
subplot(2,1,1)
hold on
step((Ucmax-UC) * Gz_uc_to_x1, 'b')     % Discrete
legend('Discrete')
title('Boost Converter Open-Loop Model (x1/uc = ipv/(d/Kdpwm))')
ylabel('PV Current i_{pv} [A]')
grid on

% Closed loop (x1 vs u1)
subplot(2,1,2)
hold on
step(Ref * SLC_Boost_m_SF_Disc, 'b')       % Discrete
legend('Discrete')
title('Closed-Loop Boost Converter with SF + Integral Action')
ylabel('PV Current i_{pv} [A]')
xlabel('Time [s]')
grid on
hold off;

%% DC-AC inverter
Fc = 5e3;               % Base frequency of the inverter triangular carrier waveform (Hz)
fs = 60;                % Grid fundamental frequency (Hz)
ws = 2*pi*fs;           % Grid angular frequency (rad/s)
vdc = 1000;             % DC-bus base voltage (V)
S = 10e3*3/2;           % System apparent-power base (VA)
VL = (381.051/2);       % Line-to-line base voltage (rms)
Vf = VL/sqrt(3);        % Phase-voltage base value (rms)
vdc_ref = vdc;          % Desired dc-bus voltage (V)
Lbase = vdc^2/(ws*S);   % Base inductance of the L filter (H)

% Final inductor value (H). If fc = 2 kHz and THD < 10%, then:
L_pu = 0.3;
Linv = L_pu*Lbase;
Linv = 0.035;            % Value selected by trial and error; Lf = Linv
Rinv = (Linv*377/fs/2);  % Internal resistance of Linv
Rinv = 0.0417;           % Rf = Rinv
Cinv = 470e-6;           % Inverter input capacitor
Ts4 = 1/(4*Fc);          % Sampling time (s)
Td = Ts4/2;              % PWM update delay (s)
Ts_inv = 1/(100*Fc);     % Sampling time of the PWM and other non-controller blocks
Vp = 1;                  % Peak voltage of the PWM carrier (V)

%% Digital PI controller design based on the DC-AC inverter frequency
% The PWM update delay is neglected

% dq-axis current PI controllers
Porcentaje1 = 1.0;                    % Desired maximum overshoot Mp (%)
Mp1 = Porcentaje1/100;
Zeta1 = sqrt(log(Mp1)^2/(log(Mp1)^2+pi^2)); % Damping ratio
tset1 = 10e-3;                        % Desired settling time (s)
Porcentaje = 2;                       % Maximum allowable error at tset1
E1 = Porcentaje/100;
wn1 = -log(E1)/(Zeta1*tset1);         % Natural frequency of the system (rad/s)
Kp2 = 2*Linv*Vp*Zeta1*wn1-Rinv*Vp;   % Proportional gain
Ki2 = Linv*Vp*wn1^2;                  % Integral gain

% PI controller for the voltage loop when a low-pass filter is applied to id
Tlpf = 0.02;                          % Low-pass-filter response time
Alpha = 2;                            % Maximum Mp of 5%, according to the Naslin method
Kp3 = (Cinv)/(Alpha*Tlpf);            % Proportional gain
Ki3 = (Cinv)/(Alpha^3*Tlpf^2);        % Integral gain

% PI controller for the voltage loop without a low-pass filter applied to id
Porcentaje = 1;                       % Desired maximum overshoot Mp (%)
Mp2 = Porcentaje/100;                 % Damping-ratio calculation parameter
Zeta2 = sqrt(log(Mp2)^2/(log(Mp2)^2+pi^2));
n = 0.6;                              % Number of times slower than the dq current controller
tset2 = tset1*n;                      % Settling time (s)
Porcentaje = 4;                       % Maximum allowable error at tset2
E2 = Porcentaje/100;
wn2 = -log(E2)/(Zeta2*tset2);         % Natural frequency of the system (rad/s)
Kp4 = 2*Cinv*Zeta2*wn2;               % Proportional gain
Ki4 = Cinv*wn2^2;                     % Integral gain

% PI controller for the PLL
Zeta3 = 0.707;                        % Damping ratio
wn3 = (120*pi)/2;                     % Grid angular frequency
Kp5 = (2*Zeta3*wn3);                  % Proportional gain
Ki5 = wn3^2;                          % Integral gain

% Calculation of the digital-controller gains
KI2 = Ki2*Ts4;
KP2 = Kp2-KI2/2;
KI3 = Ki3*Ts4;
KP3 = Kp3-KI3/2;
KI4 = Ki4*Ts4;
KP4 = Kp4-KI4/2;
KI5 = Ki5*Ts_inv;
KP5 = Kp5*Ts_inv-KI5/2;

%% 50-kVA transformer (representative of a small building); No used
Ptrafo   = 100e3;   % Rated power [W]
Ftrafo   = fs;      % Operating frequency [Hz]
VLLsec   = 33000;   % Grid-side line-to-line voltage [Vrms]
VLLprim  = VL;      % Inverter-side line-to-line voltage [Vrms]

%% Electrical grid
VLngrid = VL/sqrt(3);    % Line-to-neutral grid voltage [rms]
Fred    = fs;            % Grid frequency [Hz]
In = 1e6/(1.73*(Vf));
Icc_calc = In/(5/100);

%% Discrete-time feedback-linearizing control design without integral action
% Boost-converter model transformed by the feedback-linearizing control law
Atd=[1];
Btd=[Ts1];
Ctd=[1];   % To control z(k) = ipv(k)
Dtd=[0];
states = {'z(K)'};
inputs = {'vt(k)'};
outputs = {'ipv(k)'};
Sys_d = ss(Atd,Btd,Ctd,Dtd,Ts1,'statename',states,'inputname',inputs,'outputname',outputs);
tf(Sys_d)
Gz_vt_to_ipv=ans(1,1)
Ctd=[1]; % Control of ipv(k) = z(k)
Dtd=[0];
sys_boost_Gz_vt_to_ipv=ss(Atd,Btd,Ctd,Dtd,Ts1);

% Design parameters
Testcl = 0.0055*3.73;    % Desired settling time (test with rt = 11.3 ms)
zeta = 1;                % Desired damping ratio (not required here, but retained for consistency)
omega_n = 4 / (zeta*Testcl);

% Desired pole
p1 = -omega_n;  % Negative real pole for the desired dynamics

% Conversion of the pole to discrete time using the exponential mapping
p_dis = exp(p1*Ts1);

% Compute the state-feedback gain
Kt_acker_d = acker(Atd, Btd, p_dis);  % A single gain is obtained in this case
K1 = Kt_acker_d;

% Define the desired reference for z(k), i.e., ipv(k) in this case
z1_ref_acker = Ref; % Adjust according to the desired reference

% Modify the control law to include the reference term
u_ref_acker_d = Kt_acker_d*[z1_ref_acker]; % Only z1_ref contributes to the control input

% New closed-loop system including the reference term
Aft_acker_d = Atd-Btd*Kt_acker_d;
Bft_acker_d = Btd*u_ref_acker_d; % Reference contribution is included at the input

% Compute the closed-loop eigenvalues (denominator roots)
% These must match the desired poles
Polos_boost_d_lazo_cerrado_acker=eig(Aft_acker_d)

% Closed-loop transformed Boost converter with state feedback
% and without integral action
slc_boost_d_SF_conREF_acker=ss(Aft_acker_d,Bft_acker_d,Ctd,0,Ts1);
slc_boost_d_SF_sinREF_acker=ss(Aft_acker_d,Btd,Ctd,0,Ts1);

% Open- and closed-loop responses of the transformed Boost converter
% with SF and without integral action
figure(2)

subplot(311)
step(((Ucmax-UC))*Gz_uc_to_x1)
title('Discrete-Time Boost Converter Open-Loop Response')

subplot(312)
step(1.0*slc_boost_d_SF_conREF_acker)
title('Discrete-Time Feedback-Linearized Closed-Loop Response with Reference Tracking')

subplot(313)
step(1.0*slc_boost_d_SF_sinREF_acker)
title('Discrete-Time Feedback-Linearized Closed-Loop Response without Reference Tracking')

%% High-gain observer poles
% The dominant observer poles should be located farther to the left in the
% complex plane (i.e., have more negative real parts) than the poles of the
% transformed closed-loop system under the control action, so that the
% observer dynamics are faster and do not affect the controller performance.

ep=0.0001; % A smaller value makes the response more closely approximate
           % the model, although the behavior becomes closer to
           % state-feedback control

%alpha1 = 2;
%alpha2 = 1;
Zeta_obs=1;
Test_obs=4;
wn_obs = 4 / (Zeta_obs*Test_obs);
Alpha1 = 2*Zeta_obs*wn_obs;
Alpha2 = wn_obs;
wp_sist_cont=sqrt((Kdpwm*UC-1)^2/(Cdc*L));
Ep_cont=1/(48.6*wp_sist_cont)   % N = 48.6 (EHGO bandwith factor)
Epsilon=Ep_cont/Ts1
K1disc=Alpha1/(Epsilon*Ts1);
K2disc=Alpha2^2/(Epsilon^2*Ts1^2);

% Warning for the F28069M board
% An arbitrary threshold of 1e38 is used as a reference for double-precision overflow
umbral_f28069m = 1e38;
if abs(K1disc) > umbral_f28069m || abs(K2disc) > umbral_f28069m
    disp('Warning: At least one observer gain exceeds the reference double-precision range of the F28069M.')
end
polos_obs_cont = roots([1 Alpha1/Ep_cont Alpha2/Ep_cont^2])
polos_obs_dig = exp(polos_obs_cont*Ts1)


%% Operating point for the feedback-linearizing control
X2;
L;
Ts=Ts1;


%% Continuous- and discrete-time PI controller design
% Using SISO TOOL
%       a*(1+b*s)
% Cpi = ----------
%           s
a =  2400*30;  % rt = 11.3 ms and ts = 0.269 (final test with rt)
b = 1/30;
Kp1 = a*b
Ti1 = Kp1/a

% Controller discretization
KI1=Kp1/Ti1*Ts1;
KP1=Kp1-KI1/2;
Cs_PI_SISO=tf([Kp1 Kp1/Ti1],[1 0]);
Cz_PI=zpk(tf([(KP1+KI1) -KP1],[1 -1],Ts1))   % Controller discretization
TFclz=feedback(Cz_PI*Gz_uc_to_x1,1);

figure(3)
step(Ref * TFclz, 'b');

legend('Discrete', 'Location', 'Best');
title('Boost Converter Closed-Loop Step Response with PI Control')
xlabel('Time [s]')
ylabel('PV Current i_{pv} [A]')
grid on;

%% Response of all controllers
figure(4)
hold on;

% Step response with state feedback (SF)
step(Ref * SLC_Boost_m_SF_Disc, 'b');

% Step response with PI control
step(Ref * TFclz, 'r');

% Step response with discrete-time feedback-linearizing control
step(1.0 * slc_boost_d_SF_conREF_acker, 'g');

legend('State Feedback', ...
       'PI Control', ...
       'Feedback Linearization', ...
       'Location', 'Best');

title('Boost Converter Closed-Loop Response with Different Controllers')
xlabel('Time [s]')
ylabel('PV Current i_{pv} [A]')
grid on;
hold off;


%% DC-DC converter parameters
% Nominal parasitic parameters
ESRCpv=0.2;
RL=0.3;
Ronfet=0.05;
Vfdiode=0.7;
Rondiode=0.05;
ESRCdc=0.2;
Fact1 = 0.7;
Fact2 = 1.3;
Cpv=Cpv;
L=L;
Ronfet=0.001;
Vfdiode=0.0;
Rondiode=0.001;
Cdc=Cdc;

%% Adaptive-Fuzzy Control

% Width of the Gaussian membership function A1
a = 9;

% Desired settling-time bounds
ts_min = 0.00030;
ts_max = ts_min*100;

% Lyapunov-admissible decay-rate bounds
alpha_min = 4/ts_max;
alpha_max = 4/ts_min;

%% Fixed-Gain Control

% Constant N according to the selected settling criterion
N = 3;            % N = 4 for a 2% criterion (4*tau), N = 3 for a 5% criterion (3*tau)
ts_f = 0.0044415; % Desired settling time
alpha = N / ts_f;
Rho = exp(-alpha*Ts1);        % Exact discrete-time contraction factor
Kf = (1-Rho)/Ts1;
Lf = (1-Rho)/Ts1;

%% MPPT Method

% Current-ripple compensation factor
dIL = (vpv/L)*d*Ts;

% Use of dIL in the reference: Iref = Impp - dIL
% dIL represents the peak-to-peak inductor-current ripple under CCM.
% In this control loop, the reference is synchronized with the VALLEY of
% the current ripple (sampling at the end of Ts). Therefore, the
% sawtooth-shaped ripple remains ABOVE the reference and shifts the
% operating point above Impp.
%
% By subtracting dIL, the reference is shifted downward so that the upper
% ripple envelope does not exceed Impp. This reduces the power bias and
% mitigates undershoot when the irradiance decreases.
%
% Inductor-current ripple estimation for the Boost converter under CCM:
%   di_on  = (Vpv/L)       * D       * Ts;    % Current increase during Ton
%   di_off = (Vpv-Vdc)/L   * (1-D)   * Ts;    % Current decrease during Toff (<0)
%   dIL    = max(di_on,-di_off);              % Peak-to-peak ripple amplitude
%
% Note: dIL can be updated online using Vpv, D, L, and Fs, or by directly
% estimating the ripple using a high-pass filter, to maintain the
% compensation under varying operating conditions.