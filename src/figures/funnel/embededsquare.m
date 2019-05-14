clear all;

sdpvar x y

% parametrize the circle.
r = 1.45; % approximately sqrt(2).
p = r^2 - x^2 - y^2;

% Parametrize the unit square.
g = [1-x; 1+x;1-y; 1+y];

% Create the LaGrangian multipliers.
[s1,c1] = polynomial([x y],2);
[s2,c2] = polynomial([x y],2);
[s3,c3] = polynomial([x y],2);
[s4,c4] = polynomial([x y],2);

% Apply the positivstellensatz (S-procedure) and try to
% find coefficients of the multiplier polynomials.
F = [sos(p - [s1 s2 s3 s4]*g), sos(s1), sos(s2), sos(s3), sos(s4)];

[sol,v,Q,res] = solvesos(F,[],[],[c1;c2;c3;c4])

% Check for successful decomposition
if sol.problem ~= 0
    disp('Sum-of-squares decomposition not possible!');
    return;
else
    disp('composition succesfull!');
    h = sosd(F);
    sdisplay(h)

    clean(p-h'*h,1e-6)
end
