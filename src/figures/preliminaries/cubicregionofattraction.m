clear all;

sdpvar x;

fdot = -x + x^3;

V = 1/2*x^2;

Vdot = x*(fdot);

[s1,c1] = polynomial([x],4);

C = [sos(-Vdot -s1*(1/2 - V)), sos(s1)]

[sol,v,Q,res] = solvesos(C,[],[],[c1])

if sol.problem ~= 0
    disp('Sum-of-squares decomposition not possible!');
    return;
else
    disp('composition succesfull!');
    h = sosd(F);
    sdisplay(h)

    % Check for successful decomposition
    clean(p-h'*h,1e-6)
end