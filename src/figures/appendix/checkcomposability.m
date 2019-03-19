close all;
clear all;
% load funnelLibrary.mat

load 'Funnels.mat'

x = msspoly('x',4);

inv_dim = [1,2,3,4];

for i = 1:length(Funnels)
    for j = 1:length(Funnels)
        % Check that the output of i fits in input of j

        % Output funnel
        tso = Funnels{i}.ts;
        x0o = Funnels{i}.trajectory.eval(tso(end));
        Vo = 1-Funnels{i}.lyapunov_function.getPoly(tso(end));
        Vo = clean(Vo,1e-6);

        % Input funnel
        tsi = Funnels{j}.ts;
        x0i = Funnels{j}.trajectory.eval(tsi(1));
        Vi = 1-Funnels{j}.lyapunov_function.getPoly(tsi(1));
        Vi = clean(Vi,1e-6);

        % Check condition
        prog = spotsosprog;
        x = decomp(Vo);
        prog = prog.withIndeterminate(x);
        [prog,L] = prog.newPos(1);
        coefs = L;

        prog = prog.withSOS(clean(subs(Vi,x(inv_dim),x0i(inv_dim)),1e-6) - L*Vo);

        options = spot_sdp_default_options();
        options.verbose = 1;
        sol = prog.minimize(0,@spot_mosek,options);

    if ~strcmp(sol.status,'STATUS_PRIMAL_AND_DUAL_FEASIBLE')
        disp('Funnels did not fit!')
        figure;
        hold on;
        Funnels{i}.plotFunnel;
        x = Funnels{i}.outletPosition;
        f2 = Funnels{j}.transformFunnelpostheta(x(1:2), x(3));
        f2.plotFunnel;
        keyboard;
        hold off;
    else
        disp('Funnels did fit!')
        figure;
        hold on;
        Funnels{i}.plotFunnel(1);
        x = Funnels{i}.outletPosition;
        f2 = Funnels{j}.transformFunnelpostheta(x(1:2), x(3));
        f2.plotFunnel(1);
        hold off;
    end
    end
end
