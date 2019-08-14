function [ydothist] = finitedifference(yhist, h, n)
% Foward, Central, Backwards finite difference calculation of derivative
% This function uses the Central Finite Difference method to compute the
% derivative for the given data vector. At the ends of the array, central
% difference doesn't work, so the forward difference method and backwards
% difference methods are used instead.
% 
% Note that this function decides for you to use forward and bakwards
% differencing functions at either end of the dataset. This cannot be
% turned off or changed.
% 
% @arg
% yhist    - N x M double matrix
%            Function value time history, where N is the length of the
%            dataset and M is the number different things to take the 
%            derivative of. M is usually 1.
% h        - double
%            Time step
% n        - double (optional)
%            Order of finite difference
% 
% @return
% ydothist - N x M double matrix
%            Finite difference derivative time history
% 
% @author: Matt Marti
% @date: 2019-05-06

% Input checking
N = size(yhist, 1);
assert(N >= n+1, 'Not enough data points for given order');
if nargin < 3
    n = 5;
end

% Preallocate output
ydothist = zeros(size(yhist));
hmat = zeros(n,n);

% Forward difference method
for i = 1:n    
    
    % Delta t matrix
    for j = 1:n
        d = 1;
        hj = h*j;
        for k = 1:n
            d = d*k;
            hmat(j,k) = (hj^k)/d;
        end
    end
    
    % Forward finite difference
    yfvec = yhist(i+1:i+n,:) - yhist(i,:);
    
    % Compute derivative
    ydoti = gausselimination(hmat, yfvec);
    
    % Assign output
    ydothist(i,:) = ydoti(1,:);
end

% Central difference method
for i = n+1:N-n
    
    % Delta t matrix
    for j = 1:n
        d = 1;
        hj = h*j;
        for k = 1:n
            d = d*k;
            hmat(j,k) = (hj^k)/d;
        end
    end
    
    % Forward finite difference
    yfvec = yhist(i+1:i+n,:) - yhist(i,:);
    
    % Backward finite difference
    ybvec = yhist(i,:) - yhist(i-1:-1:i-n,:);
    
    % Compute derivative
    ydoti_f = gausselimination(hmat, yfvec);
    ydoti_b = gausselimination(hmat, ybvec);
    ydoti = 0.5 * (ydoti_f + ydoti_b);
    
    % Assign output
    ydothist(i,:) = ydoti(1,:);
end

% Backwards difference method
for i = N-n+1:N
    
    % Delta t matrix
    for j = 1:n
        d = 1;
        hj = h*j;
        for k = 1:n
            d = d*k;
            hmat(j,k) = (hj^k)/d;
        end
    end
    
    % Backward finite difference
    ybvec = yhist(i,:) - yhist(i-1:-1:i-n,:);
    
    % Compute derivative
    ydoti = gausselimination(hmat, ybvec);
    
    % Assign output
    ydothist(i,:) = ydoti(1,:);
end

end

