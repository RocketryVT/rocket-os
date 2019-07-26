%% Test_gausselimination.m
% 
% Test case for the Gauss Elimination function. Based on the solution to
% AOE 4404 Numerical Methods Assignment 2 Problem 2. Uses Gauss Elimination
% to solve a set of linear equations.
% 
% @author: Matt Marti
% @date: 2019-04-26

clear


%% Run Test

% Given
A = [3,2,-3,1,6;6,2,4,0,5;-3,1,0,2,3;5,-8,1,2,6;5,-8,1,4,6];
B = [-24,5;-6,3;-9,8;24,2;36,12];

% True solution
xtru = A\B;

% Computed solution
[x_soln, A_aug] = gausselimination(A, B);

% Assertion
precision = 1e-12;
for i = 1:size(xtru,1)
    for j = 1:size(xtru,2)
        assert(abs(xtru(i,j) - x_soln(i,j)) < precision, 'Wrong solution');
    end
end
for i = 2:size(A,1)
    for j = 1:i-1
        assert(~A_aug(i,j), 'Non-zero element in lower triangular area');
    end
end


%% Test 2
% A matrix that actually needs partial pivoting

% Given
A = [1 1 1; 2 2 1; 3 4 2];
B = [1;2;2];

% True solution
xtru = A\B;

% Computed solution
[x_soln, A_aug] = gausselimination(A, B);

% Assertion
precision = 1e-12;
for i = 1:size(xtru,1)
    for j = 1:size(xtru,2)
        assert(abs(xtru(i,j) - x_soln(i,j)) < precision, 'Wrong solution');
    end
end
for i = 2:size(A,1)
    for j = 1:i-1
        assert(~A_aug(i,j), 'Non-zero element in lower triangular area');
    end
end


%% Results
fprintf('PASSED: Test_gausselimination\n');