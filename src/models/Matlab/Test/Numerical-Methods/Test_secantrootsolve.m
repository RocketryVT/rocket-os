%% Test_secantrootsolve.m
% 
% Test case for the Secant Root Solver function. Based on the solution to
% Problem 2 of Homework 1 f AOE 4404 Numerical Methods
% 
% Use Graphical technique, bisection method, false-position, fixed-point
% iteration, Netwon method, and secant method to find the first root of
%     f(x) = x*exp(x) - cos(x)
% 
% @author: Matt Marti
% @date: 2019-04-26

clear

% Define function
f = @(x) cos(x) - x.*exp(x);


%% Part C: Secant Method

% Parameters
a = 0; % Lower bound
b = 1; % Upper bound
errstop = 1e-12; % Stopping criteria
maxiter = 1000;

% Function call
[x, niter, erra] = secantrootsolve(f, a, b, maxiter, errstop);

% Check results
assert(abs(f(x)) < errstop, 'Results error not less than specified error');
assert(abs(erra) < errstop, 'Results error not less than specified error');
assert(niter < maxiter, ...
    'Took too many iterations, function could be bugged');



%% Pass

fprintf('PASSED: Test_secantrootsolve\n');

