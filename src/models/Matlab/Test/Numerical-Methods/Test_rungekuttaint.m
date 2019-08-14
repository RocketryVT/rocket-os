%% Test_rungekuttaint.m
% 
% Test script for the Runge Kutta integration function developed as the
% solution to the AOE 4404 Homework 6 Problem 3.
% 
% @author: Matt Marti
% @date: 2019-04-29

clear


%% Test case 1: 4-th order Runge-Kutta

% Funnction input
ydot = @(t,y) [y(1,:); cos(t)];
y0 = [1; 5];
tlims = [0, 3];
h = 0.001;

% Truth values
yfun = @(t) [exp(t); sin(t) + 5];

% Compute test values
[yout, tout, ydotout] = rungekuttaint(ydot, y0, tlims, h);

% Compute error and test results
thist = tlims(1):h:tlims(2);
assert(max(abs(thist - tout)) < 1e-12, 'Bad Runge Kutta Time History');
errhist = max(max(yfun(thist) - yout));
assert(errhist < 1e-12, 'Bad Runge Kutta Time History');
errdothist = max(max(ydot(thist, yfun(thist)) - ydotout));
assert(errdothist < 1e-12, 'Bad Output time derivative history');
maxxerr4 = max(max(abs(errhist)));
assert(maxxerr4 < 1e-12, 'Bad Runge Kutta Solution History');


%% Test case 2: 3-rd order Runge-Kutta

% Compute test values
[yout, tout] = rungekuttaint(ydot, y0, tlims, h, 3);

% Compute error and test results
thist = tlims(1):h:tlims(2);
errhist = yfun(thist) - yout;
assert(max(abs(thist - tout)) < 1e-12, 'Bad Runge Kutta Time History');
maxxerr3 = max(max(abs(errhist)));
assert(maxxerr3 < 1e-5, 'Bad Runge Kutta Solution History');
assert(maxxerr4 < maxxerr3, 'Bad Runge Kutta Solution History');


%% Test case 3: 2-nd order Runge-Kutta

% Compute test values
[yout, tout] = rungekuttaint(ydot, y0, tlims, h, 2);

% Compute error and test results
thist = tlims(1):h:tlims(2);
errhist = yfun(thist) - yout;
assert(max(abs(thist - tout)) < 1e-12, 'Bad Runge Kutta Time History');
maxxerr2 = max(max(abs(errhist)));
assert(maxxerr2 < 1e-4, 'Bad Runge Kutta Solution History');
assert(maxxerr4 < maxxerr2, 'Bad Runge Kutta Solution History');
assert(maxxerr3 < maxxerr2, 'Bad Runge Kutta Solution History');


%% Test case 4: 1-st order Runge-Kutta

% Compute test values
[yout, tout] = rungekuttaint(ydot, y0, tlims, h, 1);

% Compute error and test results
thist = tlims(1):h:tlims(2);
errhist = yfun(thist) - yout;
assert(max(abs(thist - tout)) < 1e-12, 'Bad Runge Kutta Time History');
maxxerr1 = max(max(abs(errhist)));
assert(maxxerr1 < 1e-1, 'Bad Runge Kutta Solution History');
assert(maxxerr4 < maxxerr1, 'Bad Runge Kutta Solution History');
assert(maxxerr3 < maxxerr1, 'Bad Runge Kutta Solution History');
assert(maxxerr2 < maxxerr1, 'Bad Runge Kutta Solution History');


%% Test case 5: Custom 4-th order Runge-Kutta
% This example is the 3/8ths rule Runge-Kutta as described on Wikipedia

% Butcher Tableau
cvec = [ 0; 1/3; 2/3; 1 ];
amat = [ 
    0,    0,  0, 0;
    1/3,  0,  0, 0;
    -1/3, 1,  0, 0;
    1,    -1, 1, 0];
bvec = [ 1/8, 3/8, 3/8, 1/8 ];

% Compute test values
[yout, tout] = rungekuttaint(ydot, y0, tlims, h, ...
    4, cvec, amat, bvec);

% Compute error and test results
thist = tlims(1):h:tlims(2);
errhist = yfun(thist) - yout;
assert(max(abs(thist - tout)) < 1e-12, 'Bad Runge Kutta Time History');
maxxerr38 = max(max(abs(errhist)));
assert(maxxerr38 < 1e-12, 'Bad Runge Kutta Solution History');
assert(maxxerr38 <= maxxerr4, 'Bad Runge Kutta Solution History');


%% Test case 6: 5-th order, 6 stage Runge-Kutta

% Compute test values
[yout6, tout] = rungekuttaint(ydot, y0, tlims, h, 6);

% Compute error and test results
thist = tlims(1):h:tlims(2);

assert(max(abs(thist - tout)) < 1e-12, 'Bad Runge Kutta Time History');
errhist6 = yfun(thist) - yout6;
maxxerr6 = max(max(abs(errhist6)));
assert(maxxerr6 < maxxerr4, 'Bad Runge Kutta Solution History');


%% Pass

fprintf('PASSED: Test_rungekuttaint\n');