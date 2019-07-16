%% Test_hybridRocketThrustCalc.m
% 
% Based on the driver script for AOE 4984: Booster Design, Assignment 4.
% That script is the implimentation of Example 16.4 from the textbook 
% "Rocket Propulsion Elements 8th Edition", by Oscar Biblarz George P. 
% Sutton. It models the performance of a hybrid rocket.
% 
% This script tests the hybridRocketThrustCalc.m function using data from
% Sutton. This is to ensure that Thrust and Specific Impulse are computed
% correctly. The values given and computed were compared and agree with the
% values in Sutton in the Booster Design homework as of 2019-04-22.
% 
% Example 16-1
% Suppose that the operating characteristics of a Space Shuttle-class
% hybrid rocket booster are to be determined, given the following initial
% design requirements.
% 
% Unlike Sutton, I use functions from numerical methods to compute the
% values instead of looking things up from tables. This allows me to have a
% much more flexible design.
% 
% @author: Matt Marti
% @date: 2019-04-25

clear


%% Given

% Constants
g0 = 32.174; % [ft/s/s] Gravity acceleration

% Numeric design choices
Fv = 3.1e6; % [lbf] Required Initial Thrust (vacuum)
tburn = 120; % [s] Burn Time
Douter = 150; % [in] Fuel Grain Outside Diameter
P1 = 700; % [psia] Initial Chamber Pressure
r = 2; % Initial Mixture Ratio
A2oAt = 7.72; % Initial Expansion Ratio

% Miscellaneous parameters
Nports_circ = 7; % Number of channels in fuel - circular array
cstar_efficiency = 0.95;

% Fuel: HTPB
rhof = 0.033; % [lbm/in^3] Fuel density
aHTPB = 0.104;
nHTPB = 0.681;
rHTPBdot_fun = @(G0) aHTPB*G0^nHTPB; % [in/s] HTPB regression rate
% Oxidizer: Liquid Oxygen
% Table 16-2
rvec = (1:.2:3)'; % Mass Mixture Ratio
cstarvec = [
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
    5639]; % [ft/s] Characteristic velocity
kvec = [
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
    1.132]; % [no units]


%% Determine Mass Flow Rates

% fprintf('--- Not dependent on fuel grain geometry ---\n');

% Determine characteristic velocity and specific heats using cubic spline
cstar_theory = cubicspline(rvec, cstarvec, r); % [ft/s]
k = cubicspline(rvec, kvec, r); % [ft/s]

% Determine pressure ratio from Expansion Ratio
A2oAt_fun = @(M,k) 1./M.*(2*(1+0.5*(k-1)*M.^2)/(k+1)).^(0.5*(k+1)/(k-1));
Mach_error_fun = @(M) log(A2oAt_fun(M,k)) - log(A2oAt); % Error function
M2 = secantrootsolve( Mach_error_fun, 1, 5); % Exit Mach No.
P0oP = @(M,k) (1+0.5*(k-1)*M.^2).^(k/(k-1)); % Stagnation Pressure P0 ~= P1
% Remember: People Order Our Patties!
% fprintf('Flow Exit Mach Number (M2):            %.2f\n', M2);

% Determine Thrust coefficient
P2 = P1 / P0oP(M2,k); % Exit pressure
P3 = 0; % Ambient pressure (vacuum)
P2oP1 = P2/P1;
Cf_P3 = sqrt(((2*k^2)/(k-1))*(2/(k+1))^((k+1)/(k-1))*(1-(P2oP1)^((k-1)/k))) ...
    + (P2-P3)/P1*A2oAt; % Thrust coefficient
% fprintf('Vacuum thrust coeff (Cfv):             %.3f\n', Cf_P3);

% Determine initial nozzle area
At = Fv / (Cf_P3*P1); % [in^2] Nozzle throat Area
Dt = 2*sqrt(At/pi); % [in] Nozzle throat Diameter
% fprintf('Throat Area (At):                      %.2f  [in^2]\n', At);
% fprintf('Throat Diameter (Dt):                  %.2f    [in]\n', Dt);

% Mass flow rate
cstar = cstar_efficiency * cstar_theory; % [ft/s] 
mdot = g0 * P1 * At / cstar;
% fprintf('Mass flow rate (mdot):                 %.2f [lbm/s]\n', mdot);

% Mass values
mfdot = mdot / (r+1); % [lbm/s] Fuel mass rate
modot = mdot - mfdot; % [lbm/s] Oxidizer mass rate
% fprintf('Fuel rate (mfdot):                     %.2f  [lbm/f]\n', mfdot)
% fprintf('Oxidizer rate (modot):                 %.2f  [lbm/f]\n', modot)


%% Determine fuel and oxidizer mass requirement for circular port array
% Note that the next section assumes (and is only valid for) a geometry in
% which all the fuel grains are circles centered around a center circle.

% fprintf('\n--- Cicular cross section parameters ---\n');
N = Nports_circ;
piN = pi*N;
a = aHTPB;
n = nHTPB;

% Determine Ri and db (initial port radius and fuel grain width)
Rtf_constraint = 0.5*Douter/3; % Constraint for fuel grain diameter
Rtf_fun = @(Ri, a, n, t) ...
    (a*(2*n+1)*((modot/piN)^n)*t + Ri^(2*n+1))^(1/(2*n+1)); % Eq 16-13: Solution to the fuel regression rate ODE
Ri_error_fun = @(Ri) Rtf_fun(Ri, aHTPB, nHTPB, tburn) - Rtf_constraint; % Error function for port radius
Ri = secantrootsolve( Ri_error_fun, 0, 0.5*Douter); % Combustion Port radius
db = 0.5*Douter/3 - Ri; % Fuel burn distance (grain width)
% fprintf('Port radius (Ri):                      %.2f    [in]\n', Ri);
% fprintf('Grain width (db):                      %.2f    [in]\n', db);

% Determine length of fuel
G0 = modot / (piN*Ri^2); % [lbm/in^2/s] Oxidizer Mass Velocity
rdot = rHTPBdot_fun(G0);
L = (mfdot/N) / (2*pi*Ri*rhof*rdot); % [in] fuel length
% fprintf('Oxidizer mass velocity (G0):           %.2f     [lbm/in^2/s]\n', G0);
% fprintf('Fuel regression rate (rdot):           %.3f    [in/s]\n', rdot);
% fprintf('Fuel grain length (L):                 %.1f   [in]\n', L);

% Determine required fuel and oxidizer mass
crossSectionArea_fuel = ( pi*(Ri+db)^2 - pi*Ri^2 );
mf = N * L * crossSectionArea_fuel * rhof;
mo = modot*tburn;
% fprintf('Fuel Mass (mf):                        %.0f   [lbm]\n', mf);
% fprintf('Oxidizer Mass (mo):                    %.0f   [lbm]\n', mo);
% fprintf('Total propellant mass (m):             %.0f  [lbm]\n', mo+mf);

% --- Determine end specific impulse for circular cross section ---

% Determine Specific impluse
Isp = Cf_P3*cstar / g0;
% fprintf('Specific Impulse (Isp):                %.2f   [s]\n', Isp);


%% Run function to test values obtained

% Area calculations
A2 = A2oAt * At; % [in^2] Nozzle Exit Area
Ap = N * pi * Ri^2; % [in^2] Port area
Ab = N*pi*2*Ri*L; % [in^2] Exposed fuel surface area

% Fuel reguression characteristics
regresscoeff = [aHTPB; nHTPB];

% Cstar efficiency
eta = cstar_efficiency;

% Unit Conversion parameters
lbm2kg = 0.453592; % Pounds to kilograms
in2m = 0.0254; % Inches to meters
ft2in = 12; % Feet to inches
lb2N = 4.44822; % Pounds to Newtons
psia2Pa = 6894.76; % Pounds per square inch to Pa
ft2m = 0.3048;

% Compute Unit conversions
g0_si = g0*ft2m;
rdot_si = rdot * in2m;
rhof_si = rhof * lbm2kg / in2m / in2m / in2m;
a_si = 3.045015375e-5; % a in SI units
n_si = 0.680825577; % n in SI units
% eta = eta
At_si = At*in2m^2;
A2_si = A2*in2m^2;
Ap_si = Ap*in2m^2;
Ab_si = Ab*in2m^2;
modot_si = modot * lbm2kg;
mfdot_si = mfdot * lbm2kg;
mdot_si = mdot * lbm2kg;
P3_si = P3 * psia2Pa;
Fv_si = Fv * lb2N;
P1_si = P1 * psia2Pa;
Go_si = modot_si / Ap_si;
cstarvec_si = cstarvec * ft2m;
c_si = Fv_si/mdot_si;

% Regression function
regress_fun = @(Go) a_si*(Go^n_si); % Sutton 16-15 and 16-5

% Function call
[ F_test, rdot_test, mfdot_test, mdot_test, P1_test, Isp_test, CF_test, c_test ] ...
        = hybridRocketThrustCalc( ...
            g0_si, rhof_si, regress_fun, rvec, cstarvec_si, kvec, eta, ...
            At_si, A2_si, Ap_si, Ab_si, modot_si, P3_si );

% Test values
merr = 1e-5; % Max error
assert( abs((Fv_si - F_test)/Fv_si) < merr, 'Bad thrust');
assert( abs(rdot_si - rdot_test)/rdot < merr, 'Bad fuel regression');
assert( abs(P1_si - P1_test)/P1_si < merr, 'Bad Combustion Pressure');
assert( abs(mfdot_si - mfdot_test)/mfdot_si < merr, 'Bad Fuel Mass rate');
assert( abs(mdot_si - mdot_test)/mdot_si < merr, 'Bad Mass flow rate');
assert( abs(Isp - Isp_test)/Isp < merr, 'Bad Specific Impulse');
assert( abs(Cf_P3 - CF_test)/Cf_P3 < merr, 'Bad Thrust Coefficient');
assert( abs(c_si - c_test)/c_si < merr, 'Bad Exhaust Velocity');


%% If oxidizer mass flow rate is 0, thrust should be 0

% Function call
[ F_test, rdot_test, mfdot_test, mdot_test, P1_test, Isp_test, CF_test, c_test ] ...
        = hybridRocketThrustCalc( ...
            g0_si, rhof_si, regress_fun, rvec, cstarvec_si, kvec, eta, ...
            At_si, A2_si, Ap_si, Ab_si, 0, P3_si );

% Test values
merr = 1e-5; % Max error
assert( abs(F_test) < merr, 'Bad thrust');
assert( abs(rdot_test) < merr, 'Bad fuel regression');
assert( abs(mfdot_test) < merr, 'Bad Fuel Mass rate');
assert( abs(mdot_test) < merr, 'Bad Mass flow rate');
assert( abs(c_test) < merr, 'Bad Exhaust Velocity');


%% Pass

fprintf('PASSED: Test_hybridRocketThrustCalc\n');

