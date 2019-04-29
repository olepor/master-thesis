function [V,rho,Phi,usys] = optimizeTVControllerRobus_vol(polyOrig,Vtraj0,tv,ts,xtraj0,utraj,options,rho,Phi)
% % Implements time-varying reachability computation.
% The approach searches for a Lyapunov function
% and attempts to minimize the size of the funnel.
%
% Comments below are obsolete.
% @param sys Taylor expanded closed loop system (with tvlqr)
% @param polyOrig Taylor expanded original system (not closed loop)
% @param Vtraj0 Initial guess for Lyapunov function
% @param G Target set description
% @param tv Tvlqr controller
% @param ts Time samples
% @param xtraj0 Nominal trajectory
% @param utraj Nominal control input
% @param options Options structure
%
% @option controller_deg Degree of controller to search for
% @option max_iterations Maximum number of iterations (3 steps per
% iteration)
% @option converged_tol Tolerance for convergence
% @option saturations True if input saturations exist, False if not
% @option rho0 Initial guess for rho
% @option clean_tol Tolerance for cleaning small terms
% @option backoff_percent Percentage to back off on objective (helps with
% numerics)
% @option degL1 Multiplier degree
% @option Lu Multiplier degree
% @option Lu1 Multiplier degree
% @option Lu2 Multiplier degree
% @option Lup Multiplier degree
% @option Lum Multiplier degree


% Default options
if (nargin<7) options = struct(); end
if (~isfield(options,'controller_deg')) options.controller_deg = 1; end % Degree of polynomial controller to search for
if (~isfield(options,'max_iterations')) options.max_iterations = 10; end % Maximum number of iterations (3 steps per iteration)
if (~isfield(options,'converged_tol')) options.converged_tol = 1e-3; end % Tolerance for checking convergence
if (~isfield(options,'saturations')) options.saturations = false; end % Set whether or not there are any input saturations
if (~isfield(options,'rho0')) options.rho0 = 0.1*ones(length(ts),1); options.rho0(end) = 1; end % Initial "guessed" rho
if (~isfield(options,'clean_tol')) options.clean_tol = 1e-6; end % tolerance for cleaning small terms
if (~isfield(options,'backoff_percent')) options.backoff_percent = 5; end % 5 percent backing off
if (~isfield(options,'degL1')) options.degL1 = options.controller_deg + 1; end % Chosen to do degree matching
if (~isfield(options,'degLu')) options.degLu = options.controller_deg - 1; end
if (~isfield(options,'degLu1')) options.degLu1 = 2; end
if (~isfield(options,'degLu2')) options.degLu2 = 2; end
if (~isfield(options,'degLup')) options.degLup = 2; end
if (~isfield(options,'degLum')) options.degLum = 2; end
if (~isfield(options,'num_uncertain')) options.num_uncertain = 0; end

num_uncertain = options.num_uncertain;

% Get the necessary variables
t=msspoly('t',1);
x=Vtraj0.getFrame.getPoly;
ts=ts(:);

num_xd = polyOrig.getNumDiscStates();
num_xc = polyOrig.getNumContStates();
if (num_xd), error('Discrete state not handled'); end
num_u = polyOrig.getNumInputs() - num_uncertain;

if options.saturations && (num_u > 1)
    error('Sorry, I cannot handle actuator saturations for systems with more than one actuator.')
end

uw = polyOrig.getInputFrame.getPoly;
w = uw((end-num_uncertain+1):end); % Uncertainty/disturbance variables
u = uw(1:num_u);

typecheck(Vtraj0,'PolynomialLyapunovFunction');

N = length(ts);
Vmin = zeros(N-1,1);

xdottraj = fnder(xtraj0);

% sys = sys.inStateFrame(Vtraj0.getFrame); % convert system to Lyapunov function coordinates
origFrame = polyOrig.getStateFrame;
polyOrig = polyOrig.inStateFrame(Vtraj0.getFrame); % convert polyOrig to Lyapunov function coordinates


% evaluate dynamics and Vtraj at every ts once (for efficiency/clarity)
for i=1:N
    x0=xtraj0.eval(ts(i)); x0=x0(num_xd+(1:num_xc));

    %   fy{i} = sys.getPolyDynamics(ts(i));
    %   if (sys.getNumInputs>0)   % zero all inputs
    %     fy{i} = subs(fy{i},sys.getInputFrame.poly,zeros(sys.getNumInputs,1));
    %   end

    forig_u{i} = polyOrig.p_dynamics_traj.eval(ts(i));

    % if options.saturations
        forig_umax{i} = subss(polyOrig.p_dynamics_traj.eval(ts(i)),u,options.umax);
        forig_umin{i} = subss(polyOrig.p_dynamics_traj.eval(ts(i)),u,options.umin);
    % end

    K = tv.D.eval(ts(i));
    ui{i} = utraj.eval(ts(i)) + K*x;

    % Initialize Phi
    if nargin < 9
        Phi{i} = zeros(length(x),length(x));
    end

    Vy{i} = Vtraj0.getPoly(ts(i)) + x'*Phi{i}*x;

    S0s{i} = eval(Vtraj0.S,ts(i)) + Phi{i}; % 0.5*double(subs(diff(diff(Vy{i},x)',x),x,0*x));  % extract Hessian

    Vmin(i) =  minimumV(x,Vy{i});

end

dts = diff(ts);

% Initialize rho
if nargin < 8
    %  error('Have not implemented this yet. You must provide initial rho.');

    % rho = exp(options.rho0_tau*(ts-ts(1))/(ts(end)-ts(1)))-1+rho0; %+max(Vmin);
    % [Vy,rho,Phi] = rhoAndVLineSearch(Vtraj0,Vy,utraj,ts,forig_u,Phi,dts,options,u,ui,x,sys);
    rho = rhoLineSearch(Vtraj0,Vy,utraj,ts,forig_u,forig_umax,forig_umin,Phi,dts,options,u,ui,x,w);

end


rhodot = diff(rho)./dts;

N = length(ts);
num_xc = polyOrig.getNumContStates();

optimize_plot_fig = figure;

% perform alternation search here
last_rhointegral=Inf; % sum(rho);
for iter=1:options.max_iterations
    % last_rhointegral = rhointegral;

      % First step: Fix rho and V, search for L
      L1 = findL_nosat(Vy,rho,rhodot,ts,forig_u,Phi,options,u,ui,x,w,Vtraj0,utraj);
      subplot(2,2,1);
      hold on;
      plot(ts,rho);
      title(['iteration ' num2str(iter)])
      drawnow;


        % Second step: Fix V and L, search for u and rho
        % [ui,rho,cLvf,cLvpf,cLvmf,cLwf,cLwpf,cLwmf] = findURho(Vy,utraj,ts,forig_u,tv,Phi,options,forig_umax,forig_umin,u,xu,L1,Lu1,Lu2,Lp,Lup,Lm,Lum,w,Vtraj0,ui);
      [ui,rho,ucoeffs] = findurho_nosat(Vy,utraj,ts,forig_u,Phi,options,u,x,L1,w,Vtraj0,ui);
      subplot(2,2,1);
      plot(ts,rho);
      title(['iteration ' num2str(iter)])
      drawnow;

      % Second step: Fix u and L, search for V and rho
      Phiold = Phi;
      [Vy,rho,Phi] = findVRho_nosat(Vy,Vtraj0,ts,forig_u,Phi,S0s,options,x,u,ui,L1,w,rho);

    end % if

    rhodot = diff(rho)./dts;
    subplot(2,2,1);
    plot(ts,rho);
    % hold off;
    % title(['iteration ' num2str(iter)])
    drawnow;

    % rhointegral = sum(rho);

    rhointegral = 0;
    for k = 1:N
        % P0 = (S0s{k} + Phiold{k})/rho(k);
        % rhointegral = rhointegral - trace(inv(P0)*((S0s{k} + Phi{k})/rho(k)));

        Pk = (S0s{k} + Phi{k})/rho(k);
        rhointegral = rhointegral + 1/sqrt(det(Pk));
    end

    subplot(2,2,2);
    scatter(iter,rhointegral);
    hold on;
    title(['rhointegral'])
    drawnow;

    rhointegral = rhointegral^(1/num_xc);

    % check for convergence
    abs(-(rhointegral - last_rhointegral)/last_rhointegral)
    if abs((-(rhointegral - last_rhointegral)/last_rhointegral) < options.converged_tol)
        disp('converged')
        % keyboard;
        break;
    end


    last_rhointegral = rhointegral;



    % Plot funnel
    % Plot stuff
    for k = 1:length(ts)
        S0(:,:,k) = eval(Vtraj0.S,ts(k))/rho(k);
        s10(:,k) = eval(Vtraj0.s1,ts(k))/rho(k);
        s20(k) = eval(Vtraj0.s2,ts(k))/rho(k);
        Phik(:,:,k) = double(Phi{k})/rho(k);
        S(:,:,k) = S0(:,:,k) + Phik(:,:,k);
    end

    STraj = PPTrajectory(spline(ts(1:end),S));
    s1Traj = PPTrajectory(spline(ts(1:end),s10));
    s2Traj = PPTrajectory(spline(ts(1:end),s20));

    V = QuadraticLyapunovFunction(Vtraj0.getFrame,STraj,s1Traj,s2Traj);

    % Plot V in the Lyapunov function coordinates.
    subplot(2,2,3);
    options.plotdims = [1 2];
    options.inclusion = 'projection';
    plotFunnel(V,options);
    hold off;
    axis equal
    drawnow;

    % Convert V back to state-frame and plot it.
    Vxframe = V.inFrame(origFrame);
    subplot(2,2,4);
    options.plotdims = [1 2];
    options.x0 = xtraj0;
    options.inclusion = 'projection';
    plotFunnel(Vxframe,options);
    fnplt(xtraj0,[1 2]);
    axis equal
    drawnow;

end

if options.search_controller

    % Make controller object
    if options.controller_deg > 1 % If controller degree is greater than 1, make a PolynomialControlSystem
        uik = zeros(length(ucoeffs{1}),length(ts)-1);
        for k = 1:length(ui)
            uik(:,k) = ucoeffs{k};
        end
        uik(:,end+1) = uik(:,end); % Is there something better I can do here?
        coeffspline = spline(ts(1:end),uik);
        monoms = monomials(x,1:options.controller_deg);
        usys = PolynomialControlSystem(num_xc,num_u,xtraj0,utraj,coeffspline,monoms);
        % usys = setInputFrame(usys,polyOrig.getStateFrame); % These frames are not
        % right... set them when simulating
        % usys = setOutputFrame(usys,polyOrig.getInputFrame);

    else % If controller is linear, then just make a AffineSystem

        uik = zeros(length(ucoeffs{1}),length(ts)-1);
        for k = 1:length(ui)
            uik(:,k) = ucoeffs{k};
        end
        uik(:,end+1) = uik(:,end); % Is there something better I can do here?
        % coeffspline = spline(ts(1:end),uik);
        coeffspline = foh(ts(1:end),uik);
        usys = AffineSystem([],[],[],[],[],[],[],PPTrajectory(coeffspline)',utraj);
    end

else

    usys = tv;

end

for k = 1:length(ts)
    S0(:,:,k) = eval(Vtraj0.S,ts(k))/rho(k);
    s10(:,k) = eval(Vtraj0.s1,ts(k))/rho(k);
    s20(k) = eval(Vtraj0.s2,ts(k))/rho(k);
    Phik(:,:,k) = double(Phi{k})/rho(k);
    S(:,:,k) = S0(:,:,k) + Phik(:,:,k);
end

STraj = PPTrajectory(spline(ts(1:end),S));
s1Traj = PPTrajectory(spline(ts(1:end),s10));
s2Traj = PPTrajectory(spline(ts(1:end),s20));

V = QuadraticLyapunovFunction(Vtraj0.getFrame,STraj,s1Traj,s2Traj);

% keyboard;

end

function [L1f] = findL_nosat(Vy,rho,rhodot,ts,forig_u,Phi,options,u,ui,x,w,Vtraj0,utraj)

N = length(Vy)-1;
disp('Step 1: Searching for multipliers (nosat)...')

% Optimized multipliers
L1f = cell(1,N);

for k = 1:N

    % Initialize program
    prog = spotsosprog;
    prog = prog.withIndeterminate([x;w]);

    Phidotk = (Phi{k+1} - Phi{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));

    % Compute Vdot
    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phi{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;

    % Clean stuff
    V = clean(Vy{k},options.clean_tol);
    Vdoty = clean(Vdoty,options.clean_tol);

    % Create multipliers
    [prog,L1] = prog.newFreePoly(monomials([x;w],0:options.degL1));

    Lw = cell(options.num_uncertain,1);

    for i = 1:options.num_uncertain
        [prog,Lw{i}] = prog.newFreePoly(monomials([x;w(i)],0:options.degLw));
        prog = prog.withSOS(Lw{i});
    end

    [prog,slack] = prog.newPos(1);

    conDeriv = -slack*(x'*x)^(deg(Vdoty,x)/2) - Vdoty + rhodot(k) + L1*(V-rho(k));

    for i = 1:options.num_uncertain
        conDeriv = conDeriv - Lw{i}*(options.wmax(i)-w(i))*(w(i)-options.wmin(i));
    end

    prog = prog.withSOS(conDeriv);

    % Solve problem
    options_spot = spot_sdp_default_options();
    options_spot.verbose = 0;
    costfun = 0; % -slack;
    sol = prog.minimize(costfun,@spot_mosek,options_spot);

    if ~strcmp(sol.status,'STATUS_PRIMAL_AND_DUAL_FEASIBLE')
        disp('Primal infeasible')
        keyboard;
    end


    % Backoff
    % options.backoff_percent = 5;
    % costfun_opt = double(sol.eval(costfun));
    % prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);
    % sol = prog.minimize(0,@spot_sedumi,options_spot);
    % Backoff
    % options_spot.backoff_percent = 10;
    % costfun_opt = double(sol.eval(costfun));
    % prog = prog.withPos(costfun_opt + (options_spot.backoff_percent/100)*abs(costfun_opt) - costfun);
    % sol = prog.minimize(0,@spot_mosek,options_spot);

    % if ~strcmp(sol.status,'STATUS_PRIMAL_AND_DUAL_FEASIBLE')
    %     disp('Primal infeasible')
    %     keyboard;
    % end

    % Optimized multipliers
    L1f{k} = sol.eval(L1);

end
end

function [uf,rho,ucoeffs] = findurho_nosat(vy,utraj,ts,forig_u,phi,options,u,x,l1,w,vtraj0,ui)

n = length(vy)-1;
disp('step 2: searching for controller and rho noSat...')

uorig = ui;

prog = spotsosprog;
prog = prog.withIndeterminate([x;w]);

% initialize rho
[prog,rho] = prog.newPos(n+1);
prog = prog.withPos(rho(1) - options.rho0);

rhodot = msspoly(zeros(n,1));

% initialize multipliers
lw = cell(options.num_uncertain,n);

ucoeffs = cell(n,1);

% now setup vdot constraints for each time step
for k = n:-1:1

    phidotk = (phi{k+1} - phi{k})/(ts(k+1) - ts(k));
    v0k = vtraj0.getPoly(ts(k));
    v0dotk = vtraj0.getPolyTimeDeriv(ts(k));

    % create multipliers for uncertainty variables
    for i = 1:options.num_uncertain
        [prog,lw{i}{k}] = prog.newFreePoly(monomials(w(i),0:options.degLw));
        prog = prog.withSOS(lw{i}{k});
    end

    % create u
    if options.search_controller
        if options.controller_deg == 1
            umonoms = [x]; % making this a special case since for a linear controller we will make an affinesystem controller.
            % for this, i need the monomials to be [x1,x2,...,xn] and
            % not ordered in some weird way (which is what
            % we'll get if we call monomials(x,1).
        else
            umonoms = monomials(x,1:options.controller_deg);
        end
        [prog,ucoeffs{k}] = prog.newFree(length(umonoms));
        ubar{k} = ucoeffs{k}'*umonoms;

        ui{k} = utraj.eval(ts(k)) + ubar{k};
        vdoty = diff(v0k,x)*subss(forig_u{k},u,ui{k}) + v0dotk + 2*x'*phi{k}*subss(forig_u{k},u,ui{k}) + x'*phidotk*x;


        % [prog,ubar{k}] = prog.newFreePoly(monomials(x,0:options.controller_deg));
        % ui{k} = utraj.eval(ts(k)) - ubar{k};
        % vdoty = diff(v0k,x)*subss(forig_u{k},u,ui{k}) + v0dotk + 2*x'*phi{k}*subss(forig_u{k},u,ui{k}) + x'*phidotk*x;
    else
        % just use the ui from before
        vdoty = diff(v0k,x)*subss(forig_u{k},u,ui{k}) + v0dotk + 2*x'*phi{k}*subss(forig_u{k},u,ui{k}) + x'*phidotk*x;
    end

    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));

    % clean stuff
    vdoty = clean(vdoty,options.clean_tol);

    % declare sos conditions
    conderiv = - vdoty + rhodot(k) + l1{k}*(vy{k}-rho(k));

    for i = 1:options.num_uncertain
        conderiv = conderiv - lw{i}{k}*(options.wmax(i)-w(i))*(w(i)-options.wmin(i));
    end

    prog = prog.withSOS(conderiv);

end


% solve sos program
options_spot = spot_sdp_default_options();
options_spot.verbose = 0;
options_spot.scale_monomials = true;
costfun = sum(rho);
costfun = costfun/1000;
sol = prog.minimize(costfun,@spot_mosek,options_spot);


if ~strcmp(sol.status,'STATUS_PRIMAL_AND_DUAL_FEASIBLE')
    disp('Primal infeasible')
    keyboard;
end

% Volume cost functional

% C = [eye(2), zeros(2,2)]; % Projection matrix
% costfun = 0;
% for k = 2:(N+1)
%     P0k = (S0s{k} + Phiold{k})/double(rho(k)); % Nominal value for linearization.
%     Pk = (S0s{k} + PhidM{k})/double(rho(k)); % The linearization origo.
%     weight = 1;
%     % Linearized projection volume
%     costfun = costfun - weight*trace(((C'*((C*(P0k\(C')))\C))/P0k)*(Pk*inv(P0k)));
%     % prog = prog.withPos(costfun + (trace(((C'*((C*(P0k\(C')))\C))/P0k)*(Pk*inv(P0k)))));
% end

% if sol.info.primalinfeasible ~= 0 || sol.info.dualinfeasible ~= 0
%     keyboard;
% end

disp('backing off now...')

% backoff
options.backoff_percent = 1;
costfun_opt = double(sol.eval(costfun));
prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);
sol = prog.minimize(0,@spot_mosek,options_spot);

% if sol.info.primalinfeasible ~= 0 || sol.info.dualinfeasible ~= 0
%     keyboard;
% end

rho = double(sol.eval(rho));

if options.search_controller
    for k = 1:n
        uf{k} = sol.eval(ui{k});
        ucoeffs{k} = sol.eval(ucoeffs{k});
    end
else
    uf = uorig;
end

% for k = 1:n
%     for i = 1:options.num_uncertain
%         lwf{i}{k} = sol.eval(lw{i}{k});
%     end
% end

end

function [V,rho,Phi] = findVRho_nosat(Vy,Vtraj0,ts,forig_u,Phiold,S0s,options,x,u,ui,L1,w,rho)

N = length(Vy)-1;

disp('Step 2: Searching for V and rho(nosat)...')

% Declare rho
prog = spotsosprog;
prog = prog.withIndeterminate([x;w]);
rhodot = msspoly(zeros(N,1));

PhidM = cell(N+1,1);
[prog,PhidM{1}] = prog.newSym(length(x));

[prog,tau] = prog.newPos(1);
V{1} = Vtraj0.getPoly(ts(1)) + x'*PhidM{1}*x;
rho_fac = 1.0;
[prog,rho0] = prog.newPos(1);
prog = prog.withPos(rho0 - options.rho0);
rho = msspoly(rho);
rho(1) = rho0;

% Make sure initial condition set contains B0 = {x | x'*G0*x <= 1}
% Make sure that V is <= 1 on X0
prog = prog.withSOS(rho(1) - V{1} - tau*(rho_fac - x'*options.G0*x));

% Constrain V so that it is <= 1 on the set X
% [prog, L0] = prog.newPos(1);
% prog = prog.withSOS(Vtraj0.getPoly(ts(end)) - 1 + *L0*(x'*options.GX*x - 1));

% prog = prog.withSOS(rho_fac - x'*options.G0*x - ...
%                                                x'*options.G1 - options.G2 - ...
%                     tau*(1 - V{1}));

for k = 2:N+1
      [prog,PhidM{k}] = prog.newSym(length(x));
      % Make V positive definite
      prog = prog.withPSD(S0s{k} + PhidM{k});

      % Normalization constraint
        % prog = prog.withEqs(trace(PhidM{k}) - trace(Phiold{k}));
        % prog = prog.withEqs(PhidM{k} - zeros(length(x),length(x)));
end

% Constrain at the end
% V{N+1} = Vtraj0.getPoly(ts(end)) + x'*PhidM{N+1}*x;
% [prog, L0] = prog.newPos(1);
% prog = prog.withSOS(rho(end) - V{N+1} -L0*(1 - x'*options.G0*x));


% prog = prog.withEqs(trace(PhidM{N+1}) - trace(Phiold{N+1}));
% prog = prog.withEqs(PhidM{N+1} - zeros(length(x),length(x)));

for k = 1:N

    % Create multipliers for uncertainty variables
    for i = 1:options.num_uncertain
        [prog,Lw{i}{k}] = prog.newFreePoly(monomials([x;w(i)],0: ...
                                                     options.degLw));
        prog = prog.withSOS(Lw{i}{k});
    end

    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));

    % Compute Vdot
    Phidotk = (PhidM{k+1} - PhidM{k})/(ts(k+1) - ts(k));

    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*PhidM{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;

    Vy{k} = V0k + x'*PhidM{k}*x;

    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));

    % Clean stuff
    L1{k} = clean(L1{k},options.clean_tol);
    Vy{k} = clean(Vy{k},options.clean_tol);
    Vdoty = clean(Vdoty,options.clean_tol);

    % Declare SOS conditions
    % [prog,slack] = prog.newPos(1);
    conDeriv = rhodot(k) - Vdoty + L1{k}*(Vy{k}-rho(k));

    for i = 1:options.num_uncertain
        conDeriv = conDeriv - Lw{i}{k}*(options.wmax(i)-w(i))*(w(i)-options.wmin(i));
    end

    prog = prog.withSOS(conDeriv);

    % Make V positive definite
    % [prog,Phislack] = prog.newPSD(length(x));
    % prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(k)) + PhidM{k}));

end


% Solve SOS program
options_spot = struct();
options_spot = spot_sdp_default_options();
options_spot.verbose = 0;
options_spot.scale_monomials = true;

% Solve SOS program
 % costfun = 0;
 %  for k = 2:(N+1) % (N+1)
 %      % P0k = (S0s{k} + Phiold{k})/rho(k);
 %      Pk = (S0s{k} + PhidM{k})/double(rho(k));
 %      Wk = diag([10 10 1]);
 %      Pk = Wk*Pk*Wk';
 %      [prog,tauk] = maxdet(prog,Pk); % tauk lower bounds det of Pk
 %      costfun = costfun - tauk;
 %  end

% costfun = costfun/1000;

C = [eye(2), zeros(2,2)]; % Projection matrix
C3 = [eye(2), zeros(2,1)]; % Projection matrix
C = C3;
costfun = 0;
for k = 2:(N+1)
    P0k = (S0s{k} + Phiold{k})/double(rho(k)); % Nominal value for linearization.
    Pk = (S0s{k} + PhidM{k})/double(rho(k)); % The linearization origo.
    weight = 1;
    % Linearized projection volume
    costfun = costfun - weight*trace(((C'*((C*(P0k\(C')))\C))/P0k)*(Pk*inv(P0k)));
    % prog = prog.withPos(costfun + (trace(((C'*((C*(P0k\(C')))\C))/P0k)*(Pk*inv(P0k)))));
end



sol = prog.minimize(costfun,@spot_mosek,options_spot);

if ~strcmp(sol.status,'STATUS_PRIMAL_AND_DUAL_FEASIBLE')
    disp('Primal infeasible')
    keyboard;
end

disp('Backing off now...')

% Backoff
options.backoff_percent = 5;
costfun_opt = double(sol.eval(costfun));
prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);
sol = prog.minimize(0,@spot_sedumi,options_spot);



% if ~strcmp(sol.status,'STATUS_PRIMAL_AND_DUAL_FEASIBLE') % Don't care if
% the backoff is unsuccessful.
%     disp('Primal infeasible')
%     keyboard;
% end

rho = double(sol.eval(rho));

for k = 1:N+1
    V{k} = Vtraj0.getPoly(ts(k)) + x'*double(sol.eval(PhidM{k}))*x;
    Phi{k} = double(sol.eval(PhidM{k}));
end
% Phi{1} = zeros(length(x),length(x));

end

function rho = rhoLineSearch(Vtraj0,Vy,utraj,ts,forig_u,forig_umax,forig_umin,Phi,dts,options,u,ui,x,w)

N = length(ts)-1;
disp('Step 0: Initializing rho with bisection search...')

% if (matlabpool('size')==0) matlabpool 4; end

rho = zeros(N+1,1);
rho(1) = options.rho0;

for k = 1:N
    rhonow = rho(k);
    rhomin = 0.7*rhonow;
    rhomax = 2.0*rhonow; % 1.5
    rho(k+1) = fzero(@(rhonext) checkRho(Vtraj0,Vy,rhonext,utraj,ts,forig_u,forig_umax,forig_umin,Phi,dts,options,u,ui,x,w,k,rhonow),[rhomin rhomax],optimset('TolX',1e-5));
    rho(k+1) = 1.001*rho(k+1) % 1.001 To ensure feasibility

end

end

function gamma = checkRho(Vtraj0,Vy,rhonext,utraj,ts,forig_u,forig_umax,forig_umin,Phi,dts,options,u,ui,x,w,k,rho)

Phidotk = (Phi{k+1} - Phi{k})/(ts(k+1) - ts(k));
V0k = Vtraj0.getPoly(ts(k));
V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));

% Compute Vdot
Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phi{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;

Vdotyp = diff(V0k,x)*forig_umax{k} + V0dotk + 2*x'*Phi{k}*forig_umax{k} + x'*Phidotk*x;
Vdotym = diff(V0k,x)*forig_umin{k} + V0dotk + 2*x'*Phi{k}*forig_umin{k} + x'*Phidotk*x;

% Clean stuff
V = clean(Vy{k},options.clean_tol);
Vdoty = clean(Vdoty,options.clean_tol);
Vdotyp = clean(Vdotyp,options.clean_tol);
Vdotym = clean(Vdotym,options.clean_tol);

% Initialize program
prog = spotsosprog;
prog = prog.withIndeterminate([x;w]);

% Create multipliers
[prog,L1] = prog.newFreePoly(monomials(x,0:options.degL1));
[prog,Lu1] = prog.newFreePoly(monomials(x,0:options.degLu1)); prog = prog.withSOS(Lu1);
[prog,Lu2] = prog.newFreePoly(monomials(x,0:options.degLu2)); prog = prog.withSOS(Lu2);
[prog,Lp] = prog.newFreePoly(monomials(x,0:options.degL1));
[prog,Lup] = prog.newFreePoly(monomials(x,0:options.degLup)); prog = prog.withSOS(Lup);
[prog,Lm] = prog.newFreePoly(monomials(x,0:options.degL1));
[prog,Lum] = prog.newFreePoly(monomials(x,0:options.degLum)); prog = prog.withSOS(Lum);

Lw = cell(options.num_uncertain,1);
Lwp = cell(options.num_uncertain,1);
Lwm = cell(options.num_uncertain,1);

for i = 1:options.num_uncertain
    [prog,Lw{i}] = prog.newFreePoly(monomials(w(i),0:options.degLw)); prog = prog.withSOS(Lw{i});
    [prog,Lwp{i}] = prog.newFreePoly(monomials(w(i),0:options.degLw)); prog = prog.withSOS(Lwp{i});
    [prog,Lwm{i}] = prog.newFreePoly(monomials(w(i),0:options.degLw)); prog = prog.withSOS(Lwm{i});
end

[prog,slack] = prog.newPos(1);
% [prog,slackp] = prog.newPos(1);
% [prog,slackm] = prog.newPos(1);

rhodot = (rhonext - rho)/dts(k);

conDeriv = -slack*(x'*x)^(deg(Vdoty)/2) - Vdoty + rhodot + L1*(V-rho) + Lu1*(ui{k} - options.umax) + Lu2*(options.umin - ui{k});
conDerivp = -slack*(x'*x)^(deg(Vdotyp)/2) - Vdotyp + rhodot + Lp*(V-rho) + Lup*(options.umax - ui{k});
conDerivm = -slack*(x'*x)^(deg(Vdotym)/2) - Vdotym + rhodot + Lm*(V-rho) + Lum*(ui{k} - options.umin);

for i = 1:options.num_uncertain
    conDeriv = conDeriv - Lw{i}*(options.wmax(i)-w(i))*(w(i)-options.wmin(i));
    conDerivp = conDerivp- Lwp{i}*(options.wmax(i)-w(i))*(w(i)-options.wmin(i));
    conDerivm = conDerivm - Lwm{i}*(options.wmax(i)-w(i))*(w(i)-options.wmin(i));
end

prog = prog.withSOS(conDeriv);
prog = prog.withSOS(conDerivp);
prog = prog.withSOS(conDerivm);

% Solve problem
options_spot = spot_sdp_default_options();
options_spot.verbose = 0;
% prog = prog.withPos(slack - 0.01);
sol = prog.minimize(-slack/10,@spot_mosek,options_spot);



if strcmp(sol.info.solverinfo.itr.prosta,'primal_infeasible') || strcmp(sol.info.solverinfo.itr.prosta,'dual_infeasible')
    gamma = -1.0;
else

    gamma = double(sol.eval(slack));

    if strcmp(sol.info.solverInfo.itr.prosta,'UNKNOWN')
        gamma = -1.0;
    end

end


end



%% Helper/Debugging functions after this%%




function [mi,ma]=plotPoly(x,P,rho)
if(nargin<3) rho=0; end
[X1,X2]=ndgrid(-2:.1:2,-2:.1:2);
Ps=reshape(doubleSafe(msubs(P,x,[X1(:)';X2(:)'])),size(X1));
mi=min(min(Ps));
ma=max(max(Ps));
surf(X1,X2,Ps); colorbar;
view(0,90);
hold on;
[c,h]=contour3(X1,X2,Ps,[rho,rho]);
set(h,'EdgeColor',[1 1 1],'LineWidth',4);
end

function [Vmin,b] = minimumV(x,V)
if (deg(V,x)>2)
    prog = mssprog;
    [prog,slack] = new(prog,1,'free');
    prog.sos = slack + V;
    [prog,info] = sedumi(prog,slack,0);
    Vmin = -doubleSafe(prog(slack));
else
    H = doubleSafe(0.5*diff(diff(V,x)',x));
    b = -0.5*(H\doubleSafe(subs(diff(V,x),x,0*x)'));
    Vmin = subs(V,x,b);
end
end

function m=sampleCheck(x,V,Vdot,rho,rhodot)
if (deg(V,x)>2) error('only checks quadratics'); end

n=length(x);
K=100;
X = randn(n,K);
X = X./repmat(sqrt(sum(X.^2,1)),n,1);

H = doubleSafe(0.5*diff(diff(V,x)',x));
b = -0.5*(H\doubleSafe(subs(diff(V,x),x,0*x)'));

try
    X = repmat(b,1,K) + (H/(doubleSafe(rho-subs(V,x,b))+eps))^(-1/2)*X;
catch
    keyboard;
end
m=max(doubleSafe(msubs(Vdot,x,X))) - rhodot;
if (m>0)
    warning('found a positive Vdot');
end
end


function y=doubleSafe(x)
y=double(x);
if (~isa(y,'double')) error('double failed'); end
end

function tf=equalpoly(A,B)

x=decomp(A);
sizecheck(A,1); sizecheck(B,1);
if (deg(A,x)>2 || deg(B,x)>2) error('not supported yet'); end  % but not very hard!

C=A-B;
if (any(abs(doubleSafe(subs(C,x,0*x)))>1e-4))
    tf=false; return;
end

if (any(abs(doubleSafe(subs(diff(C,x),x,0*x)))>1e-4))
    tf=false; return;
end

if (any(abs(doubleSafe(subs(diff(diff(C,x)',x),x,0*x)))>1e-4))
    tf=false; return;
end

tf=true;
end
