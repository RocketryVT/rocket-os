%% Test_dragCalc.m
% 
% Test case for drag calculation
% 
% @author: Matt Marti
% @date: 2019-05-01

clear all

%% Test 1: A drag value with negative velocity direction

% Input
Cd = 0.25;
A = 4;
rho = 5;
vvec = -10;

% Truth values
qtrue = 0.5*5*100;
Dtrue = qtrue*A*Cd;

% Function call
[D, q] = dragCalc(Cd, A, rho, vvec);

% Compare truth values
assert(abs(D - Dtrue) < 1e-12, 'Bad Drag');
assert(sign(D) ~= sign(vvec), 'Bad Drag direction');
assert(abs(q - qtrue) < 1e-12, 'Bad Dynamic Pressure');
assert(sign(q) == 1, 'Bad Dynamic Pressure sign');


%% Test 2: No velocity

vvec = 0;
[D, q] = dragCalc(Cd, A, rho, vvec);
assert(D == 0, 'Bad Drag');
assert(q == 0, 'Bad Dynamic Pressure');


%% Pass
fprintf('PASSED: Test_dragCalc\n');