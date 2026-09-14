clc;
clear;
close all;

% =========================================================================
%   LFA CONTROL - TWO-RULE TSUKAMOTO FUZZY INFERENCE SYSTEM
%   Panels (b), (c), and (d) of the proposed FL-EHGO-LFA figure
%
%   Consistent with equations (37)-(42) and the Simulink implementation
% =========================================================================

%% ========================== DESIGN PARAMETERS ============================

a = 9;                          % Gaussian width of A1

ts_min = 0.00030;               % Minimum settling time
ts_max = ts_min*100;            % Maximum settling time

alpha_min = 4/ts_max;
alpha_max = 4/ts_min;

Delta_alpha = alpha_max - alpha_min;

% Centered Tsukamoto output universe
alpha_c_min = -Delta_alpha/2;
alpha_c_max =  Delta_alpha/2;

fprintf('alpha_min   = %.6f\n', alpha_min);
fprintf('alpha_max   = %.6f\n', alpha_max);
fprintf('Delta_alpha = %.6f\n', Delta_alpha);

%% =========================== INPUT UNIVERSE ==============================

% Visualization range only
e_max = 30;
step_e = 0.01;

e_abs = 0:step_e:e_max;

%% =================== (b) INPUT MEMBERSHIP FUNCTIONS ======================

% Equation (37)
% A1 -> Small tracking-error magnitude
% A2 -> Large tracking-error magnitude

w1 = exp(-(e_abs.^2)/(2*a^2));
w2 = 1 - w1;

mu_A1 = w1;
mu_A2 = w2;

%% ================= (c) CONSEQUENT MEMBERSHIP FUNCTIONS ===================

Nalpha = 2000;

alpha_c_axis = linspace(alpha_c_min, alpha_c_max, Nalpha);

% B1 -> High alpha_c, monotonically increasing
mu_B1 = (alpha_c_axis - alpha_c_min) / Delta_alpha;

% B2 -> Low alpha_c, monotonically decreasing
mu_B2 = (alpha_c_max - alpha_c_axis) / Delta_alpha;

%% ======================= TSUKAMOTO INFERENCE =============================

% Equation (40)
% R1: A1 -> B2
% R2: A2 -> B1

z1 = alpha_c_max - w1.*Delta_alpha;
z2 = alpha_c_min + w2.*Delta_alpha;

% Equation (41)
den = w1 + w2;
den(den < eps) = eps;

alpha_c_out = ...
    (w1.*z1 + w2.*z2) ./ den;

% Equation (42)
alpha_k = ...
    alpha_c_out + ...
    (alpha_min + alpha_max)/2;

% Numerical saturation exactly as in the implementation
alpha_k = min(max(alpha_k, alpha_min), alpha_max);

%% ==================== IMPLEMENTATION VERIFICATION ========================

% Direct implementation used in the MATLAB Function block

w1_impl = exp(-(e_abs.^2)/(2*a^2));
w2_impl = 1 - w1_impl;

z1_impl = alpha_c_max - w1_impl.*Delta_alpha;
z2_impl = alpha_c_min + w2_impl.*Delta_alpha;

den_impl = w1_impl + w2_impl;
den_impl(den_impl < eps) = eps;

alpha_c_impl = ...
    (w1_impl.*z1_impl + w2_impl.*z2_impl) ./ den_impl;

alpha_k_impl = ...
    alpha_c_impl + ...
    (alpha_min + alpha_max)/2;

alpha_k_impl = ...
    min(max(alpha_k_impl, alpha_min), alpha_max);

max_error = max(abs(alpha_k - alpha_k_impl));

fprintf('Maximum difference with implementation = %.3e\n', max_error);

%% ========================================================================
%                       PANELS (b), (c), AND (d)
% ========================================================================

figure('Position',[100 100 1500 430]);

%% ------------------------------------------------------------------------
% (b) Input membership functions
% -------------------------------------------------------------------------

subplot(1,3,1);

plot(e_abs, mu_A1, ...
    'LineWidth', 1.8, ...
    'Color', [0 0.4470 0.7410]);

hold on;

plot(e_abs, mu_A2, ...
    'LineWidth', 1.8, ...
    'Color', [0.8500 0.3250 0.0980]);

xlabel('|e[k]|');
ylabel('\mu_{A_i}(|e[k]|)');

legend('A_1: Small |e[k]|', ...
       'A_2: Large |e[k]|', ...
       'Location','northeast');

axis([0 e_max 0 1.05]);

set(gca, ...
    'FontName','Times New Roman', ...
    'FontSize',13);

title('(b)', ...
    'FontWeight','normal');

box on;
grid off;

%% ------------------------------------------------------------------------
% (c) Consequent membership functions
% -------------------------------------------------------------------------

subplot(1,3,2);

plot(alpha_c_axis, mu_B1, ...
    'LineWidth',1.8, ...
    'Color',[0 0.4470 0.7410]);

hold on;

plot(alpha_c_axis, mu_B2, ...
    'LineWidth',1.8, ...
    'Color',[0.8500 0.3250 0.0980]);

xlabel('\alpha_c');
ylabel('\mu_{B_i}(\alpha_c)');

legend('B_1: High \alpha_c', ...
       'B_2: Low \alpha_c', ...
       'Location','northeast');

axis([alpha_c_min alpha_c_max 0 1.05]);

set(gca, ...
    'FontName','Times New Roman', ...
    'FontSize',13);

title('(c)', ...
    'FontWeight','normal');

box on;
grid off;

%% ------------------------------------------------------------------------
% (d) Resulting fuzzy mapping alpha[k] = F(|e[k]|)
% -------------------------------------------------------------------------

subplot(1,3,3);

plot(e_abs, alpha_k, ...
    'LineWidth',2, ...
    'Color',[0 0.4470 0.7410]);

hold on;

% alpha_min
plot([0 e_max], ...
     [alpha_min alpha_min], ...
     '--', ...
     'LineWidth',1.2, ...
     'Color',[0.8500 0.3250 0.0980]);

% alpha_max
plot([0 e_max], ...
     [alpha_max alpha_max], ...
     '--', ...
     'LineWidth',1.2, ...
     'Color',[0.9290 0.6940 0.1250]);

xlabel('|e[k]|');
ylabel('\alpha[k]');

legend('\alpha[k] = F(|e[k]|)', ...
       '\alpha_{min}', ...
       '\alpha_{max}', ...
       'Location','southeast');

axis([0 e_max 0 alpha_max*1.05]);

set(gca, ...
    'FontName','Times New Roman', ...
    'FontSize',13);

title('(d)', ...
    'FontWeight','normal');

box on;
grid off;

%% ========================================================================
% FIGURE 2 - STEP-BY-STEP TSUKAMOTO INFERENCE FOR A SPECIFIC ERROR VALUE
% ========================================================================

% Error value to evaluate
e0 = 10;

% -------------------------------------------------------------------------
% 1) FUZZIFICATION
% -------------------------------------------------------------------------

e0_abs = abs(e0);

w1_0 = exp(-(e0_abs^2)/(2*a^2));
w2_0 = 1 - w1_0;

% -------------------------------------------------------------------------
% 2) CRISP CONSEQUENTS FROM TSUKAMOTO INVERSION
% -------------------------------------------------------------------------

z1_0 = alpha_c_max - w1_0*Delta_alpha;
z2_0 = alpha_c_min + w2_0*Delta_alpha;

% -------------------------------------------------------------------------
% 3) WEIGHTED AGGREGATION
% -------------------------------------------------------------------------

den_0 = w1_0 + w2_0;

if den_0 < eps
    den_0 = eps;
end

alpha_c_0 = ...
    (w1_0*z1_0 + w2_0*z2_0)/den_0;

% -------------------------------------------------------------------------
% 4) TRANSLATION TO PHYSICAL DECAY-RATE INTERVAL
% -------------------------------------------------------------------------

alpha_0 = ...
    alpha_c_0 + ...
    (alpha_min + alpha_max)/2;

% Numerical protection
alpha_0 = min(max(alpha_0,alpha_min),alpha_max);

%% ------------------------------------------------------------------------
% Command Window report
% -------------------------------------------------------------------------

fprintf('\n');
fprintf('=============================================================\n');
fprintf(' TSUKAMOTO INFERENCE FOR e0 = %.4f\n',e0);
fprintf('=============================================================\n');

fprintf('|e0|        = %.6f\n',e0_abs);

fprintf('\nFuzzification:\n');
fprintf('w1 = mu_A1  = %.6f\n',w1_0);
fprintf('w2 = mu_A2  = %.6f\n',w2_0);

fprintf('\nTsukamoto inversion:\n');
fprintf('z1           = %.6f\n',z1_0);
fprintf('z2           = %.6f\n',z2_0);

fprintf('\nAggregation:\n');
fprintf('alpha_c[k]   = %.6f\n',alpha_c_0);

fprintf('\nPhysical decay rate:\n');
fprintf('alpha[k]     = %.6f\n',alpha_0);

fprintf('=============================================================\n');


%% ========================================================================
%                           FIGURE 2
% ========================================================================

figure('Position',[100 100 1500 430]);

%% ------------------------------------------------------------------------
% (a) Fuzzification of e0
% -------------------------------------------------------------------------

subplot(1,3,1);

plot(e_abs,mu_A1,...
    'LineWidth',1.8,...
    'Color',[0 0.4470 0.7410]);

hold on;

plot(e_abs,mu_A2,...
    'LineWidth',1.8,...
    'Color',[0.8500 0.3250 0.0980]);

% Vertical line at |e0|
plot([e0_abs e0_abs],[0 1],...
    ':k',...
    'LineWidth',1.2);

% Horizontal firing-strength lines
plot([0 e0_abs],[w1_0 w1_0],...
    ':',...
    'LineWidth',1.2,...
    'Color',[0 0.4470 0.7410]);

plot([0 e0_abs],[w2_0 w2_0],...
    ':',...
    'LineWidth',1.2,...
    'Color',[0.8500 0.3250 0.0980]);

% Firing points
plot(e0_abs,w1_0,'o',...
    'MarkerSize',6,...
    'LineWidth',1.3,...
    'Color',[0 0.4470 0.7410]);

plot(e0_abs,w2_0,'o',...
    'MarkerSize',6,...
    'LineWidth',1.3,...
    'Color',[0.8500 0.3250 0.0980]);

text(e0_abs,w1_0,...
    sprintf('  w_1 = %.3f',w1_0),...
    'FontName','Times New Roman',...
    'FontSize',11);

text(e0_abs,w2_0,...
    sprintf('  w_2 = %.3f',w2_0),...
    'FontName','Times New Roman',...
    'FontSize',11);

xlabel('|e[k]|');
ylabel('\mu_{A_i}(|e[k]|)');

legend('A_1: Small |e[k]|',...
       'A_2: Large |e[k]|',...
       'Location','northeast');

axis([0 e_max 0 1.05]);

set(gca,...
    'FontName','Times New Roman',...
    'FontSize',13);

title('(a) Fuzzification',...
    'FontWeight','normal');

box on;
grid off;


%% ------------------------------------------------------------------------
% (b) Consequent inversion
% -------------------------------------------------------------------------

subplot(1,3,2);

plot(alpha_c_axis,mu_B1,...
    'LineWidth',1.8,...
    'Color',[0 0.4470 0.7410]);

hold on;

plot(alpha_c_axis,mu_B2,...
    'LineWidth',1.8,...
    'Color',[0.8500 0.3250 0.0980]);

% Crisp consequent z1 on B2
plot([z1_0 z1_0],[0 w1_0],...
    ':k',...
    'LineWidth',1.1);

plot([alpha_c_min z1_0],[w1_0 w1_0],...
    ':',...
    'LineWidth',1.1,...
    'Color',[0 0.4470 0.7410]);

plot(z1_0,w1_0,'o',...
    'MarkerSize',6,...
    'LineWidth',1.3,...
    'Color',[0 0.4470 0.7410]);

% Crisp consequent z2 on B1
plot([z2_0 z2_0],[0 w2_0],...
    ':k',...
    'LineWidth',1.1);

plot([alpha_c_min z2_0],[w2_0 w2_0],...
    ':',...
    'LineWidth',1.1,...
    'Color',[0.8500 0.3250 0.0980]);

plot(z2_0,w2_0,'o',...
    'MarkerSize',6,...
    'LineWidth',1.3,...
    'Color',[0.8500 0.3250 0.0980]);

text(z1_0,w1_0,...
    sprintf('  z_1 = %.1f',z1_0),...
    'FontName','Times New Roman',...
    'FontSize',11);

text(z2_0,w2_0,...
    sprintf('  z_2 = %.1f',z2_0),...
    'FontName','Times New Roman',...
    'FontSize',11);

xlabel('\alpha_c');
ylabel('\mu_{B_i}(\alpha_c)');

legend('B_1: High \alpha_c',...
       'B_2: Low \alpha_c',...
       'Location','northeast');

axis([alpha_c_min alpha_c_max 0 1.05]);

set(gca,...
    'FontName','Times New Roman',...
    'FontSize',13);

title('(b) Tsukamoto inversion',...
    'FontWeight','normal');

box on;
grid off;


%% ------------------------------------------------------------------------
% (c) Final mapping to alpha[k]
% -------------------------------------------------------------------------

subplot(1,3,3);

plot(e_abs,alpha_k,...
    'LineWidth',2,...
    'Color',[0 0.4470 0.7410]);

hold on;

% alpha_min
plot([0 e_max],...
    [alpha_min alpha_min],...
    '--',...
    'LineWidth',1.1,...
    'Color',[0.8500 0.3250 0.0980]);

% alpha_max
plot([0 e_max],...
    [alpha_max alpha_max],...
    '--',...
    'LineWidth',1.1,...
    'Color',[0.9290 0.6940 0.1250]);

% Selected input e0
plot([e0_abs e0_abs],...
    [0 alpha_0],...
    ':k',...
    'LineWidth',1.2);

% Final alpha0
plot([0 e0_abs],...
    [alpha_0 alpha_0],...
    ':k',...
    'LineWidth',1.2);

plot(e0_abs,alpha_0,...
    'ks',...
    'MarkerSize',6,...
    'LineWidth',1.3,...
    'MarkerFaceColor','w');

text(e0_abs,alpha_0,...
    sprintf('  \\alpha[k] = %.1f',alpha_0),...
    'FontName','Times New Roman',...
    'FontSize',11);

xlabel('|e[k]|');
ylabel('\alpha[k]');

legend('\alpha[k] = F(|e[k]|)',...
       '\alpha_{min}',...
       '\alpha_{max}',...
       'Location','southeast');

axis([0 e_max 0 alpha_max*1.05]);

set(gca,...
    'FontName','Times New Roman',...
    'FontSize',13);

title('(c) Final decay rate',...
    'FontWeight','normal');

box on;
grid off;