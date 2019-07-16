%% finite_difference_calc.m
% 
% Computes coefficients for finite difference methods. Based off the
% solution to Problem 1 on Homework 6 of AOE 4404 Numerical Methods
% 
% @author: Matt Marti
% @date: 2019-05-06

clear, clc

% Order of accuracy
n = 3;

% Symbolic
syms fi h hmat
syms fip1 fip2 fip3 fip4 fip5 fip6 fip7 fip8 fip9 fip10 fip11 fip12 fip13 fip14 fip15
syms fim1 fim2 fim3 fim4 fim5 fim6 fim7 fim8 fim9 fim10 fim11 fim12 fim13 fim14 fim15

% forward difference h matrix
for i = 1:n
    d = 1;
    hi = h*i;
    for j = 1:n
        d = d*j;
        hmat(i,j) = (hi^j)/d;
    end
end

% Central difference
fpvec = [fip1 fip2 fip3 fip4 fip5 fip6 fip7 fip8 fip9 fip10 fip11 fip12 fip13 fip14 fip15] - fi;
fpvec = transpose(fpvec(1:n));
fmvec = fi - [fim1 fim2 fim3 fim4 fim5 fim6 fim7 fim8 fim9 fim10 fim11 fim12 fim13 fim14 fim15];
fmvec = transpose(fmvec(1:n));
dffvec = simplify(hmat \ fpvec);
fcvec = 0.5 * ( fpvec + fmvec );
dfcvec = simplify(hmat \ fcvec);
dfmvec = simplify(hmat \ fmvec);

% Output
fprintf('Forward Difference df_i/dx:\n');
dffvec(1)
fprintf('Central Difference df_i/dx:\n');
dfcvec(1)
fprintf('Backwards Difference df_i/dx:\n');
dfmvec(1)

