clc; clear all; close all;

% =========================================================================
%   DC-SIDE LOW-POWER EXPERIMENTAL SETUP - PARAMETERS AND CONTROL DESIGN
%
%   This script defines the operating conditions and controller parameters
%   used for the dc-side experimental validation. The nominal voltage,
%   current, and power levels differ from those of the full-scale PV system
%   considered in the PIL study because the experimental validation is
%   performed on a low-power laboratory setup.
%
%   The script includes the parameterization and design of the benchmark
%   SF, PI, and FL-EHGO controllers, as well as the proposed FL-EHGO
%   structure with Lyapunov-fuzzy auxiliary (LFA) control.
% =========================================================================


%% ========== LOW-POWER DC-SIDE BOOST-CONVERTER PARAMETERS ================
% Operating point and converter parameters used for the dc-side
% experimental setup. These voltage and power levels intentionally differ
% from those of the full-scale PV system used in the PIL evaluation.

vpv = 6;       % Initial input-voltage value
vpv = 7;       % Selected experimental input voltage
vdc = 12;      % Desired dc-bus voltage
Fs = 5e3;      % Switching frequency

Ppv = 12;      % Initial input-power value
Ppv = 20;      % Selected experimental input power

ipv = Ppv/vpv;     % Input current at the selected operating point
iL = ipv;          % Inductor current assumed equal to the input current
d =(vdc-vpv)/vdc;  % Steady-state boost-converter duty cycle
Rpv = vpv/ipv;     % Equivalent input resistance at the operating point
Idc = ipv*(1-d);   % Estimated dc-side output current
Rdc = vdc/(Idc);   % Equivalent dc-side load resistance

% Minimum inductance required for continuous-conduction mode (CCM)
Lmin =(d*(1-d)^2*Rdc)/(2*Fs);

% Selected experimental inductance
%L = 68.2729*Lmin;
L = 4.4e-3;

% Maximum and minimum inductor currents at the operating point
ILmax = vpv/((1-d)^2*Rdc)+(vpv*d)/(2*L*Fs);
ILmin = vpv/((1-d)^2*Rdc)-(vpv*d)/(2*L*Fs);

% DC-side capacitors
Cdc = 3300e-6;               % Output dc-bus capacitance
dVdc = (d*vdc)/(Rdc*Cdc*Fs)  % Estimated dc-bus voltage ripple
Cpv = 100e-6;                % Input-side capacitance

% Digital PWM parameters
b = 16;                      % DPWM resolution in bits
Ucmax = (2^b-1);             % Maximum digital control command
Ucmax = 9000;                % Selected experimental control-command limit
Kdpwm = 1/Ucmax;             % Digital PWM gain

% Nominal operating point
UC = d/Kdpwm;                % Nominal digital control command
X1 = iL;                     % Nominal state x1 = iL
X2 = vdc;                    % Nominal state x2 = vdc
Ref = ipv;                   % Current-tracking reference


%% ================= CONTINUOUS-TIME BOOST MODEL ===========================
% Small-signal continuous-time model of the boost converter around the
% selected low-power experimental operating point.
%
% States:
%   x1 = ipv : input/PV-side current
%   x2 = vdc : dc-bus voltage

% State-space matrices
Amc = [0 -(1-Kdpwm*UC)/L
      (1-Kdpwm*UC)/Cdc -1/(Cdc*Rdc)];

Bmc = [(Kdpwm*X2)/L
      -(Kdpwm*X1)/Cdc];

Cmc = [1/2 0     % Output associated with x1 = ipv = iL
      0 1];      % Output associated with x2 = vdc

Dmc = [0
      0];

states = {'x1' 'x2'};
inputs = {'uc'};
outputs = {'x1=ipv' 'x2=vdc'};

Sys_Boost_Modelo_Continuo = ...
    ss(Amc,Bmc,Cmc,Dmc,'statename',states,...
    'inputname',inputs,'outputname',outputs)

tf(Sys_Boost_Modelo_Continuo)

Gs_uc_to_x1 = (ans(1,1))
Gs_uc_to_x2 = (ans(2,1))

[num_Gs_uc_to_x1, den_Gs_uc_to_x1] = ...
    tfdata(Gs_uc_to_x1, 'v');

[num_Gs_uc_to_x2, den_Gs_uc_to_x2] = ...
    tfdata(Gs_uc_to_x2, 'v');


%% ================== DISCRETE-TIME BOOST MODEL ============================
% Discrete-time representation of the boost converter at the switching
% sampling period Ts1.

Ts1 = 1/Fs;

Amd = [1 (Ts1*(Kdpwm*UC-1))/L
   -(Ts1*(Kdpwm*UC-1))/Cdc 1-Ts1/(Cdc*Rdc)];

Bmd = [(Kdpwm*Ts1*X2)/L
    -(Kdpwm*Ts1*X1)/Cdc];

Cmd = [1/2 0      % Output associated with x1 = ipv = iL
       0 1];      % Output associated with x2 = vdc

Dmd = [0
      0];

states = {'x1' 'x2'};
inputs = {'uc'};
outputs = {'x1=ipv' 'x2=vdc'};

Sys_Boost_Modelo_Discreto = ...
    ss(Amd,Bmd,Cmd,Dmd,Ts1,'statename',states,...
    'inputname',inputs,'outputname',outputs)

tf(Sys_Boost_Modelo_Discreto)

Gz_uc_to_x1=(ans(1,1));

[num_Gz_uc_to_x1, den_Gz_uc_to_x1] = ...
    tfdata(Gz_uc_to_x1, 'v');


%% =========== DISCRETE SF CONTROLLER WITH INTEGRAL ACTION ================
% Design of the state-feedback (SF) benchmark controller with integral
% action for current-reference tracking.

Cmd_Control = [1 0];     % Controlled output: y = x1 = ipv
Dmd_Control = [0];

Amd = [1 -(Ts1*(1-Kdpwm*UC))/L
   (Ts1*(1-Kdpwm*UC))/Cdc 1-Ts1/(Cdc*Rdc)];

Bmd = [(Kdpwm*Ts1*X2)/L
    -(Kdpwm*Ts1*X1)/Cdc];

% Augmented model used for the Simulink implementation.
% The integral action is represented explicitly as a third state:
%
% x3[k+1] = x3[k] + Ts1*(xref[k] - x1[k])
%
% This formulation is consistent with the explicit discrete-time
% integrator implemented in the controller block.
Aamd = [ 1                        -(Ts1*(1-Kdpwm*UC))/L             0;
        (Ts1*(1-Kdpwm*UC))/Cdc   1-Ts1/(Cdc*Rdc)                  0;
        -Ts1                      0                                1 ];

% Alternative augmented model used for script-based closed-loop analysis.
% Here, the integral state is expressed without explicitly including Ts1
% in the augmented-state equation.
Aamdprima = [ 1                        -(Ts1*(1-Kdpwm*UC))/L             0;
             (Ts1*(1-Kdpwm*UC))/Cdc   1-Ts1/(Cdc*Rdc)                  0;
             -1                       0                                1 ];

Bamd = [ (Kdpwm*Ts1*X2)/L;
         -(Kdpwm*Ts1*X1)/Cdc;
         0 ];

Eamd = [zeros(size(Aamd,1)-1,1); 1];
Camd = [Cmd_Control 0];

% Desired closed-loop poles obtained from continuous-time performance
% specifications and mapped to the z-plane.
%Testd = 0.06245;
Testd = 0.008;
Testd = 0.007;

zeta_d = 1.0;

p3 = 52;
p3 = 85;

wn_d = 4 / (zeta_d * Testd);

s1 = -zeta_d * wn_d;
s2 = -zeta_d * wn_d;
s3 = -p3;

z1 = exp(s1 * Ts1);
z2 = exp(s2 * Ts1);
z3 = exp(s3 * Ts1);

Polos_Des_Boost_Mod_Disc = [z1 z2 z3];

% Optional pole locations for trial-and-error tuning
% z1p = exp(s1p * Ts1);
% z2p = exp(s2p * Ts1);
% z3p = exp(s3p * Ts1);
% Polos_Des_Boost_Mod_Disc = [z1p z2p z3p];

% Verify controllability of the augmented system
Co_Boost_Modelo_Disc = ctrb(Aamd, Bamd);

if rank(Co_Boost_Modelo_Disc) < size(Aamd,1)
    error('The augmented system is not completely controllable.')
end

% State-feedback gains used in the Simulink implementation
Kmd = acker(Aamd, Bamd, Polos_Des_Boost_Mod_Disc);

% State-feedback gains associated with the converter states
K1y2md = Kmd(1:end-1)

% Integral gain.
% The sign is inverted because the tracking error is defined as
% e[k] = xref[k] - x1[k].
Kimd = -Kmd(end)

% In the MATLAB Function implementation, Kimd is multiplied by Ts1
% because the discrete integral action is explicitly implemented as:
%
% Acc_Int = Ki * ek * Ts + Acc_Int_1;

% Gains used for script-based closed-loop analysis
Kmdprima = acker(Aamdprima, Bamd, Polos_Des_Boost_Mod_Disc);

% Closed-loop augmented state matrix
Afmd = Aamdprima-Bamd*Kmdprima;

% Verify that the resulting closed-loop poles coincide with the
% prescribed discrete-time pole locations
Polos_Boost_Lazo_Cerrado_Disc = eig(Aamd-Bamd*Kmd)

% Closed-loop boost-converter model with state feedback and integral action
SLC_Boost_m_SF_Disc = ss(Afmd,Eamd,Camd,0,Ts1);


%% ==================== SF CONTROLLER RESPONSES ============================
% Comparison of the open-loop and closed-loop discrete-time responses.

figure(1)

% Open-loop response: x1 with respect to uc
subplot(2,1,1)
hold on
step((Ucmax-UC) * Gz_uc_to_x1, 'b')
legend('Discrete')
title('Open-Loop Boost-Converter Model (x1/uc)')
ylabel('Current ipv [A]')
grid on

% Closed-loop response with state feedback and integral action
subplot(2,1,2)
hold on
step(Ref * SLC_Boost_m_SF_Disc, 'b')
legend('Discrete')
title('Closed-Loop Boost Converter with SF and Integral Action')
ylabel('Current ipv [A]')
xlabel('Time [s]')
grid on
hold off;


%% ========================= DC-AC INVERTER ================================
% Parameters of the inverter and grid-side stage used in the experimental
% model.

Fc = 2e3;               % Inverter carrier frequency
fs = 60;                % Grid fundamental frequency
ws = 2*pi*fs;           % Grid angular frequency

vdc = 12;               % Experimental dc-bus voltage

S = 10e3*3/2;           % Base apparent power
VL = (381.051/2);       % Base line-to-line rms voltage
Vf = VL/sqrt(3);        % Base phase rms voltage

vdc_ref = vdc;          % Desired dc-bus voltage

Lbase = vdc^2/(ws*S);   % Base filter inductance

L_pu = 0.3;
Linv = L_pu*Lbase;

% Experimentally selected inverter-filter inductance
Linv = 0.03;

Rinv = (Linv*377/fs/2); % Estimated internal resistance
Rinv = 0.0417;          % Selected resistance

Cinv = 470e-6;          % Inverter input capacitance

Ts4 = 1/(4*Fc);         % Controller sampling period
Td = Ts4/2;             % PWM update delay
Ts_inv = 1/(100*Fc);    % Sampling period of PWM and auxiliary blocks
Vp = 1;                 % PWM carrier peak voltage


%% ==================== DIGITAL INVERTER PI DESIGN =========================
% Digital PI controller design for the dc-ac inverter.

% dq-axis current PI controllers
Porcentaje1 = 1.0;                    % Desired maximum overshoot (%)
Mp1 = Porcentaje1/100;

Zeta1 = sqrt(log(Mp1)^2/(log(Mp1)^2+pi^2)); % Damping ratio

tset1 = 10e-3;                        % Desired settling time

Porcentaje = 2;                       % Allowed settling error (%)
E1 = Porcentaje/100;

wn1 = -log(E1)/(Zeta1*tset1);         % Natural frequency

Kp2 = 2*Linv*Vp*Zeta1*wn1-Rinv*Vp;   % Proportional gain
Ki2 = Linv*Vp*wn1^2;                  % Integral gain

% DC-bus voltage PI controller when a low-pass filter is used for id
Tlpf = 0.02;                          % Low-pass-filter response time
Alpha = 2;                            % Naslin design parameter

Kp3 = (Cinv)/(Alpha*Tlpf);            % Proportional gain
Ki3 = (Cinv)/(Alpha^3*Tlpf^2);        % Integral gain

% DC-bus voltage PI controller without low-pass filtering of id
Porcentaje = 1;                       % Desired maximum overshoot (%)
Mp2 = Porcentaje/100;

Zeta2 = sqrt(log(Mp2)^2/(log(Mp2)^2+pi^2));

n = 0.6;                              % Relative voltage-loop speed
tset2 = tset1*n;                      % Desired settling time

Porcentaje = 4;                       % Allowed settling error (%)
E2 = Porcentaje/100;

wn2 = -log(E2)/(Zeta2*tset2);         % Natural frequency

Kp4 = 2*Cinv*Zeta2*wn2;               % Proportional gain
Ki4 = Cinv*wn2^2;                     % Integral gain

% PLL PI controller
Zeta3 = 0.707;                         % Desired damping ratio
wn3 = (120*pi)/2;                      % PLL natural frequency

Kp5 = (2*Zeta3*wn3);                  % Proportional gain
Ki5 = wn3^2;                           % Integral gain

% Discrete-time controller gains
KI2 = Ki2*Ts4;
KP2 = Kp2-KI2/2;

KI3 = Ki3*Ts4;
KP3 = Kp3-KI3/2;

KI4 = Ki4*Ts4;
KP4 = Kp4-KI4/2;

KI5 = Ki5*Ts_inv;
KP5 = Kp5*Ts_inv-KI5/2;


%% ========================== TRANSFORMER ==================================
% Transformer parameters used in the grid-connected model.

Ptrafo   = 100e3;   % Rated transformer power [W]
Ftrafo   = fs;      % Operating frequency [Hz]
VLLsec   = 33000;   % Grid-side line-to-line rms voltage [V]
VLLprim  = VL;      % Inverter-side line-to-line rms voltage [V]


%% ============================ GRID =======================================
% Grid parameters.

VLngrid = 33000/sqrt(3);    % Line-to-neutral rms grid voltage
Fred    = fs;               % Grid frequency
In = 1e6/(1.73*(Vf));
Icc_calc = In/(5/100);


%% =========== DISCRETE FEEDBACK-LINEARIZING CONTROL DESIGN ===============
% Design of the proportional auxiliary-control law used by the FL-EHGO
% benchmark controller.
%
% After feedback linearization, the current dynamics are represented by
% the first-order transformed system:
%
%       z[k+1] = z[k] + Ts1*v[k]
%
% where z[k] corresponds to the controlled current state.

Atd=[1];
Btd=[Ts1];
Ctd=[1];               % Controlled variable: z[k] = ipv[k]
Dtd=[0];

states = {'z(K)'};
inputs = {'vt(k)'};
outputs = {'ipv(k)'};

Sys_d = ss(Atd,Btd,Ctd,Dtd,Ts1,...
    'statename',states,...
    'inputname',inputs,...
    'outputname',outputs);

tf(Sys_d)

Gz_vt_to_ipv=ans(1,1)

Ctd=[1];
Dtd=[0];

sys_boost_Gz_vt_to_ipv=ss(Atd,Btd,Ctd,Dtd,Ts1);

% Desired closed-loop settling time
Testcl = 0.01;
Testcl = 0.0105;

% Desired damping ratio
zeta = 1;

% Equivalent continuous-time decay rate
omega_n = 4 / (zeta*Testcl);

% Desired continuous-time pole
p1 = -omega_n;

% Exact mapping of the desired pole to the discrete-time domain
p_dis = exp(p1*Ts1);

% Proportional auxiliary-control gain
Kt_acker_d = acker(Atd, Btd, p_dis);
K1 = Kt_acker_d;

% Desired current reference for the transformed state
z1_ref_acker = Ref;

% Reference feedforward term
u_ref_acker_d = Kt_acker_d*[z1_ref_acker];

% Closed-loop transformed system
Aft_acker_d = Atd-Btd*Kt_acker_d;
Bft_acker_d = Btd*u_ref_acker_d;

% Closed-loop pole verification
Polos_boost_d_lazo_cerrado_acker=eig(Aft_acker_d)

% Closed-loop transformed models with and without reference tracking
slc_boost_d_SF_conREF_acker=...
    ss(Aft_acker_d,Bft_acker_d,Ctd,0,Ts1);

slc_boost_d_SF_sinREF_acker=...
    ss(Aft_acker_d,Btd,Ctd,0,Ts1);


%% ============== FEEDBACK-LINEARIZING CONTROL RESPONSES ==================
% Open- and closed-loop responses of the transformed boost-converter model.

figure(2)

subplot(311)
step(((Ucmax-UC))*Gz_uc_to_x1)
title('Discrete Open-Loop Boost Converter');

subplot(312)
step(1.0*slc_boost_d_SF_conREF_acker)
title('Discrete Closed-Loop Feedback-Linearized System with Reference Tracking');

subplot(313)
step(1.0*slc_boost_d_SF_sinREF_acker)
title('Discrete Closed-Loop Feedback-Linearized System without Reference Tracking');


%% ==================== HIGH-GAIN OBSERVER DESIGN ==========================
% High-gain observer parameters used in the FL-EHGO benchmark and proposed
% FL-EHGO-LFA controller.
%
% The observer dynamics are selected faster than the transformed
% closed-loop control dynamics so that the state and lumped-uncertainty
% estimates converge sufficiently fast relative to the controlled system.

ep=0.0001;

% Observer characteristic-polynomial parameters
Zeta_obs=1;
Test_obs=4;

wn_obs = 4 / (Zeta_obs*Test_obs);

Alpha1 = 2*Zeta_obs*wn_obs;
Alpha2 = wn_obs;

% Characteristic frequency of the continuous-time converter model
wp_sist_cont=sqrt((Kdpwm*UC-1)^2/(Cdc*L));

% Observer scaling parameter
Ep_cont=1/(48.6*wp_sist_cont)

% Normalized discrete-time observer parameter
Epsilon=Ep_cont/Ts1

% Equivalent discrete-time high-gain observer coefficients
K1disc=Alpha1/(Epsilon*Ts1);
K2disc=Alpha2^2/(Epsilon^2*Ts1^2);

% Numerical-range check for the F28069M implementation.
% An arbitrary threshold of 1e38 is used as a conservative reference.
umbral_f28069m = 1e38;

if abs(K1disc) > umbral_f28069m || abs(K2disc) > umbral_f28069m
    disp('Warning: At least one observer gain exceeds the selected numerical-range threshold for the F28069M.');
end

% Continuous- and discrete-time observer poles
polos_obs_cont = roots([1 Alpha1/Ep_cont Alpha2/Ep_cont^2])
polos_obs_dig = exp(polos_obs_cont*Ts1)


%% ============ FEEDBACK-LINEARIZING CONTROL OPERATING POINT ==============
% Parameters required by the practical feedback-linearizing controller.

X2;
L;
Ts=Ts1;


%% ====================== PI CONTROLLER DESIGN =============================
% Continuous- and discrete-time PI controller used as a benchmark.
% The continuous-time PI parameters were tuned using SISO Tool.
%
% Continuous-time controller:
%
%                 a(1 + b*s)
%       Cpi(s) = -------------
%                     s

% Candidate/tuning values retained for reproducibility
%a = 5.5e6;
%b = 0.0065;
%a = 80.0e6;
%b = 0.0010;

a = 4.3049e+06;
a = 3.8e+06;

b = 0.0018;
b = 0.0015;

% Continuous-time PI parameters
Kp1 = a*b
Ti1 = Kp1/a

% Discrete-time PI gains
KI1=Kp1/Ti1*Ts1;
KP1=Kp1-KI1/2;

% Continuous- and discrete-time PI transfer functions
Cs_PI_SISO=tf([Kp1 Kp1/Ti1],[1 0]);

Cz_PI=zpk(tf([(KP1+KI1) -KP1],[1 -1],Ts1))

% Closed-loop transfer function
TFclz=feedback(Cz_PI*Gz_uc_to_x1,1);

figure(3)

step(Ref * TFclz, 'b');

legend('Discrete (Digital)', 'Location', 'Best');
title('Closed-Loop Boost-Converter Response with PI Control');
xlabel('Time [s]');
ylabel('Current ipv [A]');
grid on;


%% ================== CONTROLLER RESPONSE COMPARISON =======================
% Comparison of the SF, PI, and feedback-linearizing benchmark controllers
% under the same low-power dc-side operating conditions.

figure(4)
hold on;

% State-feedback controller
step(Ref * SLC_Boost_m_SF_Disc, 'b');

% PI controller
step(Ref * TFclz, 'r');

% Feedback-linearizing controller
step(1.0 * slc_boost_d_SF_conREF_acker, 'g');

legend('State Feedback', ...
       'Digital PI', ...
       'Feedback Linearization', ...
       'Location', 'Best');

title('Closed-Loop Boost-Converter Response with Different Controllers');
xlabel('Time [s]');
ylabel('Current i_{pv} [A]');
grid on;
hold off;


%% ================= NOMINAL PARASITIC PARAMETERS ==========================
% Nominal parasitic parameters included in the low-power experimental
% converter model.

ESRCpv=0.2;       % ESR of the input-side capacitor
RL=494e-3;        % Inductor series resistance
Ronfet=0.05;      % MOSFET on-state resistance
Vfdiode=0.7;      % Diode forward-voltage drop
Rondiode=0.05;    % Diode on-state resistance
ESRCdc=0.2;       % ESR of the dc-bus capacitor

% Multiplicative factors retained for parameter-variation studies
Fact1 = 0.7;
Fact2 = 1.3;

% Nominal parameter assignment
Cpv=Cpv;
ESRCpv=ESRCpv;
L=L;
RL=RL;
Ronfet=Ronfet;
Vfdiode=Vfdiode;
Rondiode=Rondiode;
Cdc=Cdc;
ESRCdc=ESRCdc;


%% ========== LYAPUNOV-FUZZY AUXILIARY (LFA) CONTROL DESIGN ===============
% Parameters of the proposed Lyapunov-fuzzy auxiliary (LFA) control.
%
% The Tsukamoto fuzzy inference system adapts the desired decay rate
% alpha[k] within the Lyapunov-admissible interval
% [alpha_min, alpha_max]. The interval is established from prescribed
% settling-time limits.

a   = 9;          % Gaussian membership-function width
ts = 0.5;         % Nominal response-time parameter

% Admissible decay-rate interval
ts_min = 0.00030; % Minimum prescribed settling time
ts_max = 0.01544; % Maximum prescribed settling time

% Decay-rate bounds derived from the settling-time specifications
alpha_min = 4/ts_max;
alpha_max = 4/ts_min;

% Minimum adaptive gain associated with alpha_min
Rho = exp(-alpha_min*Ts);
Kmin = (1-Rho)/Ts

% Maximum adaptive gain associated with alpha_max
Rho = exp(-alpha_max*Ts);
Kmax = (1-Rho)/Ts