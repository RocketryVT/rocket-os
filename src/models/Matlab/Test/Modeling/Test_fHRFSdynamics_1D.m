%% Test_fHRFSdynamics_1D.m
% 
% This function tests the dynamics model function which computes the time
% derivative of the state of a Hybrid Rocket at an instantaneous point in
% time.
% 
% @author: Matt Marti
% @date: 2019-04-26

clear


%% Test 1: Input design parameters to generate a known thrust (no velocity)

% Test parameters
maxerr = 1e-12;

% Known design values
F = 13789454.7319554630666971;
rdot = 0.0035057280000891;
mfdot = 1548.2776011868756996;
mdot = 4644.8329671451692775;
P1 = 4826318.7978181280195713;
Isp = 302.7310536002646586;
CF = 1.7342192368821014;
c = 2968.7730063694425553;

% Input design parameters
rhof = 913.436110336788; % [kg/m^3] Fuel density (Thrust)
a = 3.045015375e-05; % Fuel grain regression coefficient (Thrust)
n = 0.680825577; % Fuel grain regression exponent (Thrust)
At = 1.64750631789099; % [m^2] Nozzle throat area (Thrust)
A2 = 12.7187487741184; % [m^2] Nozzle Exit area (Thrust)
N = 7; % Number of fuel grain ports (Thrust)
L = 30.2371738789983; % [m] Fuel grain length (Thrust)
Ri = 0.363557686863076; % [m] Initial Port radius (Thrust)
Af = 51.8868458595838; % [m^2] Rocket front area (Drag)
Cd = 0.25; % Random value for drag coefficient (Drag)
modot = @(tk) 3096.55536595829;

% Table 16-2
rsplinevec = (1:.2:3)'; % Mass Mixture Ratio spline data
cstarsplinedata = [
    4825;
    5180;
    5543;
    5767;
    5882;
    5912;
    5885;
    5831;
    5768;
    5703;
    5639] * 0.3048; % [m/s] Characteristic velocity spline data
ksplinedata = [
    1.308;
    1.282;
    1.239;
    1.201;
    1.171;
    1.152;
    1.143;
    1.138;
    1.135;
    1.133;
    1.132]; % [no units] Specific heat ratio spline data

% Atmosphere function
RE = 6.356766e6;
g0 = 9.807;
atm = @(h) stdAtmosphereCalc_hgp(h, RE, g0);

% Construct rocket state
tk = 0; % Time
posk = +1e20; % Altitude
velk = 0; % Velocity
dbi = 0.271442313136924; % [m] Fuel Grain Width
mok = 5; % Oxidizer mass
mfk = 5; % Fuel mass
mk = 100000; % Rocket Mass
xk = [ posk; velk; dbi; mok; mfk; mk ];

% Parameter values
regress_fun = @(Go) a*(Go^n);
Ap_fun = @(db) pi * N * (Ri-dbi+db)^2; % Port area function
Ab_fun = @(db) N * 2 * (Ri-dbi+db) * pi * L; % Burn area function
p = struct( ...
        'rhof', rhof, ... % Fuel density
        'regress_fun', regress_fun, ... % Fuel regression rate function
        'eta', 0.95, ... % Fuel/Oxidizer burn efficiency
        'At', At, ... % Throat Area
        'A2', A2, ... % Nozzle Exit Area
        'Ap_fun', Ap_fun, ... % Port area function
        'Ab_fun', Ab_fun, ... % Burn area function
        'Af', Af, ... % Frontal area
        'Cd', Cd); % Drag coefficient

% Compute truth for state
velk = 0;
acck = F / mk - g0;
rdotk = rdot;
modotk = modot(0);
mfdotk = mfdot;
mdotk = mdot;

% Compute truth for misc data
% P1
% F
D = 0;
q = 0;
% Isp

% Function call
[ xdotk, data ] = fHRFSdynamics_1D(tk, xk, p, modot, ...
    rsplinevec, cstarsplinedata, ksplinedata, atm);

% Parse output from xdotk
velk_test = xdotk(1);
acck_test = xdotk(2);
rdotk_test = xdotk(3);
modotk_test = xdotk(4);
mfdotk_test = xdotk(5);
mdotk_test = xdotk(6);

% Parse output from data
P1_test = data(1);
F_test = data(2);
D_test = data(3);
q_test = data(4);
Isp_test = data(5);

% Test cases for state
assert(abs((velk-velk_test)) < maxerr, 'Bad Velocity');
assert(abs((acck-acck_test)/acck) < maxerr, 'Bad Acceleration');
assert(abs((-rdotk-rdotk_test)/rdotk) < maxerr, 'Bad Fuel Regression');
assert(abs((-modotk-modotk_test)/modotk) < maxerr, 'Bad Oxidizer Flow Rate');
assert(abs((-mfdotk-mfdotk_test)/mfdotk) < maxerr, 'Bad Fuel Flow Rate');
assert(abs((-mdotk-mdotk_test)/mdotk) < maxerr, 'Bad Mass Flow Rate');

% Test cases for misc data
assert(abs((P1-P1_test)/P1) < maxerr, 'Bad Combustion Pressure');
assert(abs((F-F_test)/F) < maxerr, 'Bad Thrust');
assert(abs((D-D_test)) < maxerr, 'Bad Drag');
assert(abs((q-q_test)) < maxerr, 'Bad Dynamic Pressure');
assert(abs((Isp-Isp_test)/Isp) < maxerr / 1e-8, 'Bad ISP');



%% Test 2: Set Oxidizer Mass Flow Rate to 0

modot = @(t) 0;

% Function call
[ xdotk, data ] = fHRFSdynamics_1D(tk, xk, p, modot, ...
    rsplinevec, cstarsplinedata, ksplinedata, atm);

% Parse output from xdotk
velk_test = xdotk(1);
acck_test = xdotk(2);
rdotk_test = xdotk(3);
modotk_test = xdotk(4);
mfdotk_test = xdotk(5);
mdotk_test = xdotk(6);

% Parse output from data
P1_test = data(1);
F_test = data(2);
D_test = data(3);
q_test = data(4);
Isp_test = data(5);

% Test cases for state
assert(abs((velk-velk_test)) < maxerr, 'Bad Velocity');
assert(abs((-g0-acck_test)/g0) < maxerr, 'Bad Acceleration');
assert(abs(rdotk_test) < maxerr, 'Bad Fuel Regression');
assert(abs(modotk_test) < maxerr, 'Bad Oxidizer Flow Rate');
assert(abs(mfdotk_test) < maxerr, 'Bad Fuel Flow Rate');
assert(abs(mdotk_test) < maxerr, 'Bad Mass Flow Rate');

% Test cases for misc data
assert(abs(P1_test) < maxerr, 'Bad Combustion Pressure');
assert(abs(F_test) < maxerr, 'Bad Thrust');
assert(abs((D-D_test)) < maxerr, 'Bad Drag');
assert(abs((q-q_test)) < maxerr, 'Bad Dynamic Pressure');
assert(isnan(Isp_test), 'Bad ISP');


%% Test 2: Set Oxidizer Mass to 0

modot = @(tk) 3096.55536595829;
xk = [ posk; velk; dbi; 0; mfk; mk ];

% Function call
[ xdotk, data ] = fHRFSdynamics_1D(tk, xk, p, modot, ...
    rsplinevec, cstarsplinedata, ksplinedata, atm);

% Parse output from xdotk
velk_test = xdotk(1);
acck_test = xdotk(2);
rdotk_test = xdotk(3);
modotk_test = xdotk(4);
mfdotk_test = xdotk(5);
mdotk_test = xdotk(6);

% Parse output from data
P1_test = data(1);
F_test = data(2);
D_test = data(3);
q_test = data(4);
Isp_test = data(5);

% Test cases for state
assert(abs((velk-velk_test)) < maxerr, 'Bad Velocity');
assert(abs((-g0-acck_test)/g0) < maxerr, 'Bad Acceleration');
assert(abs(rdotk_test) < maxerr, 'Bad Fuel Regression');
assert(abs(modotk_test) < maxerr, 'Bad Oxidizer Flow Rate');
assert(abs(mfdotk_test) < maxerr, 'Bad Fuel Flow Rate');
assert(abs(mdotk_test) < maxerr, 'Bad Mass Flow Rate');

% Test cases for misc data
assert(abs(P1_test) < maxerr, 'Bad Combustion Pressure');
assert(abs(F_test) < maxerr, 'Bad Thrust');
assert(abs((D-D_test)) < maxerr, 'Bad Drag');
assert(abs((q-q_test)) < maxerr, 'Bad Dynamic Pressure');
assert(isnan(Isp_test), 'Bad ISP');


%% Test 2: Set Fuel Mass to 0

xk = [ posk; velk; dbi; mok; 0; mk ];

% Function call
[ xdotk, data ] = fHRFSdynamics_1D(tk, xk, p, modot, ...
    rsplinevec, cstarsplinedata, ksplinedata, atm);

% Parse output from xdotk
velk_test = xdotk(1);
acck_test = xdotk(2);
rdotk_test = xdotk(3);
modotk_test = xdotk(4);
mfdotk_test = xdotk(5);
mdotk_test = xdotk(6);

% Parse output from data
P1_test = data(1);
F_test = data(2);
D_test = data(3);
q_test = data(4);
Isp_test = data(5);

% Test cases for state
assert(abs((velk-velk_test)) < maxerr, 'Bad Velocity');
assert(abs((-g0-acck_test)/g0) < maxerr, 'Bad Acceleration');
assert(abs(rdotk_test) < maxerr, 'Bad Fuel Regression');
assert(abs((-modotk-modotk_test)/modotk) < maxerr, 'Bad Oxidizer Flow Rate');
assert(abs(mfdotk_test) < maxerr, 'Bad Fuel Flow Rate');
assert(abs((-modotk-mdotk_test)/modotk) < maxerr, 'Bad Mass Flow Rate');

% Test cases for misc data
assert(abs(P1_test) < maxerr, 'Bad Combustion Pressure');
assert(abs(F_test) < maxerr, 'Bad Thrust');
assert(abs((D-D_test)) < maxerr, 'Bad Drag');
assert(abs((q-q_test)) < maxerr, 'Bad Dynamic Pressure');
assert(isnan(Isp_test), 'Bad ISP');


%% Test 3: Give the rocket some velocity to make sure drag is correct
% Also models atmospheric flight

v = 100;
h = 100;
xk = [ h; v; dbi; mok; mfk; mk ];
[P, rho] = atm(h);
q = 1/2 * rho * v^2;
D = - Af * Cd * q;

% Function call
[ xdotk, data ] = fHRFSdynamics_1D(tk, xk, p, modot, ...
    rsplinevec, cstarsplinedata, ksplinedata, atm);

% Parse output from xdotk
velk_test = xdotk(1);
acck_test = xdotk(2);
rdotk_test = xdotk(3);
modotk_test = xdotk(4);
mfdotk_test = xdotk(5);
mdotk_test = xdotk(6);

% Parse output from data
P1_test = data(1);
F_test = data(2);
D_test = data(3);
q_test = data(4);
Isp_test = data(5);

% Compute truth acceleration here, because it changes with altitude
acck = (F_test + D)/mk - g0;

% Test cases for state
assert(abs((v-velk_test)/v) < maxerr, 'Bad Velocity');
assert(abs((acck-acck_test)/acck) < maxerr, 'Bad Acceleration');
assert(abs((-rdotk-rdotk_test)/rdotk) < maxerr, 'Bad Fuel Regression');
assert(abs((-modotk-modotk_test)/modotk) < maxerr, 'Bad Oxidizer Flow Rate');
assert(abs((-mfdotk-mfdotk_test)/mfdotk) < maxerr, 'Bad Fuel Flow Rate');
assert(abs((-mdotk-mdotk_test)/mdotk) < maxerr, 'Bad Mass Flow Rate');

% Test cases for misc data
assert(abs((P1-P1_test)/P1) < maxerr, 'Bad Combustion Pressure');
assert(F_test < F, 'Bad Thrust');
assert(abs((D-D_test)/D) < maxerr, 'Bad Drag');
assert(abs((q-q_test)/q) < maxerr, 'Bad Dynamic Pressure');
assert(Isp_test < Isp, 'Bad ISP');


%% Pass

fprintf('PASSED: Test_fHRFSdynamics_1D\n');