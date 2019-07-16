function [y2] = anonymousFuncSecondArg(func, x)
% Returns the second argument of an anonymous function
% 
% @arg
% func - Anonymous Function
%        Function handle to function which returns at least two outputs
% x    - double
% 
% @return
% y2   - double
%        Second return value of the function

[~, y2] = func(x);

end

