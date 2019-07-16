function [x_soln, A_aug] = gausselimination(A, B)
% Use Gauss Elimination to solve a set of linear equations A*x = b
% Uses partial pivoting to avoid singularities
% 
% @arg
% A      - m x m double matrix
%          Matrix that represents the A matrix. Must be square
% B      - m x n double matrix
%          Matrix that represents the set of B vectors
% 
% @return
% x_soln - m x o double matrix
%          Solution matrix to the system of linear equations
% A_aug  - m x m double matrix
%          Upper Triangular Augmented form of input A as per Lecture 7,
%          page 13.
% 
% @author: Matt Marti
% @date: 2019-04-22

% Input Size
m = size(A,1);
o = size(B,2);

% Gauss Elimination Main Loop
for i = 1:(m-1) % Iterate by rows
    
    % Pivot zero-valued diagonal elements
    if ~A(i,i)
        k = i+1;
        successflag = 0;
        while k <= m
            if A(k,i)
                Aktemp = A(k,:);
                A(k,:) = A(i,:);
                A(i,:) = Aktemp;
                Bktemp = B(k,:);
                B(k,:) = B(i,:);
                B(i,:) = Bktemp;
                successflag = 1;
                break;
            end
            k = k + 1;
        end
        assert(logical(successflag), 'argument A is singular');
    end
    
    % Perform gauss elemination on this row
    for j = (i+1):m % Iterate by rows
        if A(j,i) % If there is a non-zero element
            ajioveraii = A(j,i)./A(i,i);
            A(j,:) = A(j,:) - ajioveraii.*A(i,:);
            B(j,:) = B(j,:) - ajioveraii.*B(i,:);
        end
    end
end
A_aug = A(1:m,1:m);

% Backsubstitution loop
x_soln = zeros(m,o);
for k = 1:o
    for i = m:-1:1
        sum = 0;
        for j = i+1:m
            sum = sum + A(i,j)*x_soln(j,k);
        end
        x_soln(i,k) = ( B(i,k) - sum ) ./ A(i,i);
    end
end