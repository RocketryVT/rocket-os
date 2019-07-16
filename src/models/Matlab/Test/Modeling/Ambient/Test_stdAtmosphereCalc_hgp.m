%% Test_stdAtmosphereCalc_hgp.m
% 
% Tests the "stdAtmosphereCalc_hgp.m" function, which computes the temperature,
% pressure, and density of the atmosphere at a given altitude (and initial
% temperature and pressure).
% 
% @author: Matt Marti
% @date: 2019-04-25

clear, clear global
constants_HRFS


%% Test 1: h = 0

clear

% Truth values
T_true = 288.16;
P_true = 1.01325e5;
rho_true = 1.2250;

% Input
h = 0;
RE = 6.356766e6;
g0 = 9.807;

% Function call
[P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0);

% Test Values
maxerr = 1e-6;
assert(abs(T - T_true)/T_true < maxerr, 'Bad Temperature');
assert(abs(P - P_true)/P_true < maxerr, 'Bad Pressure');
assert(abs(rho - rho_true)/rho_true < maxerr, 'Bad Density');


%% Test 2: h = -5000

clear

% Truth values
T_true = 320.69;
P_true = 1.7761e5;
rho_true = 1.9296;

% Input
h = -5000;
RE = 6.356766e6;
g0 = 9.807;

% Function call
[P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0);

% Test Values
maxerr = 1e-3;
assert(abs(T - T_true)/T_true < maxerr, 'Bad Temperature');
assert(abs(P - P_true)/P_true < maxerr, 'Bad Pressure');
assert(abs(rho - rho_true)/rho_true < maxerr, 'Bad Density');


%% Test 3: h = 5000

clear

% Truth values
T_true = 255.69;
P_true = 5.4048e4;
rho_true = 7.3643e-1;

% Input
h = 5000;
RE = 6.356766e6;
g0 = 9.807;

% Function call
[P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0);

% Test Values
maxerr = 1e-4;
assert(abs(T - T_true)/T_true < maxerr, 'Bad Temperature');
assert(abs(P - P_true)/P_true < maxerr, 'Bad Pressure');
assert(abs(rho - rho_true)/rho_true < maxerr, 'Bad Density');


%% Test 4: h = 10k

clear

% Truth values
T_true = 223.26;
P_true = 2.6500e4;
rho_true = 4.1351e-1;

% Input
h = 10e3;
RE = 6.356766e6;
g0 = 9.807;

% Function call
[P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0);

% Test Values
maxerr = 1e-4;
assert(abs(T - T_true)/T_true < maxerr, 'Bad Temperature');
assert(abs(P - P_true)/P_true < maxerr, 'Bad Pressure');
assert(abs(rho - rho_true)/rho_true < maxerr, 'Bad Density');


%% Test 5: h = 20k

clear

% Truth values
T_true = 216.66;
P_true = 5.5293e3;
rho_true = 8.8909e-2;

% Input
h = 20e3;
RE = 6.356766e6;
g0 = 9.807;

% Function call
[P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0);

% Test Values
maxerr = 1e-4;
assert(abs(T - T_true)/T_true < maxerr, 'Bad Temperature');
assert(abs(P - P_true)/P_true < maxerr, 'Bad Pressure');
assert(abs(rho - rho_true)/rho_true < maxerr, 'Bad Density');


%% Test 6: h = 30k

clear

% Truth values
T_true = 231.24;
P_true = 1.1855e3;
rho_true = 1.7861e-2;

% Input
h = 30e3;
RE = 6.356766e6;
g0 = 9.807;

% Function call
[P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0);

% Test Values
maxerr = 1e-3;
assert(abs(T - T_true)/T_true < maxerr, 'Bad Temperature');
assert(abs(P - P_true)/P_true < maxerr, 'Bad Pressure');
assert(abs(rho - rho_true)/rho_true < maxerr, 'Bad Density');


%% Test 7: h = 40k

clear

% Truth values
T_true = 260.91;
P_true = 2.9977e2;
rho_true = 4.0028e-3;

% Input
h = 40e3;
RE = 6.356766e6;
g0 = 9.807;

% Function call
[P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0);

% Test Values
maxerr = 1e-3;
assert(abs(T - T_true)/T_true < maxerr, 'Bad Temperature');
assert(abs(P - P_true)/P_true < maxerr, 'Bad Pressure');
assert(abs(rho - rho_true)/rho_true < maxerr, 'Bad Density');


%% Test 8: h = 50k

clear

% Truth values
T_true = 282.66;
P_true = 8.7858e1;
rho_true = 1.0829e-3;

% Input
h = 50e3;
RE = 6.356766e6;
g0 = 9.807;

% Function call
[P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0);

% Test Values
maxerr = 1e-3;
assert(abs(T - T_true)/T_true < maxerr, 'Bad Temperature');
assert(abs(P - P_true)/P_true < maxerr, 'Bad Pressure');
assert(abs(rho - rho_true)/rho_true < maxerr, 'Bad Density');


%% Test 9: h = 59k

clear

% Truth values
T_true = 258.10;
P_true = 2.9250e1;
rho_true = 3.9482e-4;

% Input
h = 59e3;
RE = 6.356766e6;
g0 = 9.807;

% Function call
[P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0);

% Test Values
maxerr = 1e-3;
assert(abs(T - T_true)/T_true < maxerr, 'Bad Temperature');
assert(abs(P - P_true)/P_true < maxerr, 'Bad Pressure');
assert(abs(rho - rho_true)/rho_true < maxerr, 'Bad Density');


%% Pass

fprintf('PASSED: Test_stdAtmosphereCalc_hgp\n');