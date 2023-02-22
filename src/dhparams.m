function [outputArg1] = dhparams(d,a,alph,the)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

DH=[cos(the) -sin(the)*cos(alph) sin(the)*sin(alph) a*cos(the);
    sin(the) cos(the)*cos(alph) -cos(the)*sin(alph) a*sin(the);
    0 sin(alph) cos(alph) d;
    0 0 0 1]
outputArg1 = DH;
end

