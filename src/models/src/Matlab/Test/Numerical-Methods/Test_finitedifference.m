%% Test_finitedifference.m
% 
% Test case for the finite difference function
% 
% @author: Matt Marti
% @date: 2019-05-06

clear


%% Test 1: Make sure all the orders work

% Function input
h = .1;
t = (0:h:10)';
yhist = -9.81/2*t.^2 + 50*t + 200;

% Truth value
ydottruthhist = -9.81*t + 50;

% Test Function call
errvec = [5e-1; 1e-11; 1e-11; 1e-11; 1e-11; 1e-11; 2e-11; 2.5e-11; 5e-11];
for n = 1:9
    [ydothist] = finitedifference(yhist, h, n);
    errhist = ydothist - ydottruthhist;
    maxerr = max(abs(errhist));
    assert(maxerr < errvec(n), ...
        'Error for low order polynomial is not zero');
end


%% Test 2: Derivative of nonlinear function is approximate and all orders work
% Also test that multiple data entries work

% Function input
h = .1;
t = (0:h:10)';
yhist = [sin(t), cos(t)];
n = 5;

% Truth value
ydottruthhist = [cos(t), -sin(t)];

% Function call
[ydothist] = finitedifference(yhist, h, n);

% Test
% Test Function call
errvec = [5e-2; 5e-3; 2.5e-4; 2e-5; 2e-6; 5e-7; 2e-8; 2e-9; 1e-10; 1e-11; 2e-12];
for n = 1:11
    [ydothist] = finitedifference(yhist, h, n);
    errhist = ydothist - ydottruthhist;
    maxerr = max(max(abs(errhist)));
    assert(maxerr < errvec(n), ...
        'Error for low order polynomial is not zero');
end


%% Pass

fprintf('PASSED: Test_finitedifference\n')

