---
layout: single
title: "Decentralized control of nonlinear quadruple-tank network"
showTitle: true
excerpt: "Application of decentralized control to the quadruple-tank network, using the one-step method."
permalink: /examples/QuadrupleTankNetworkControlOneStep/
classes: wide
sidebar:
  title: "Examples"
  nav: sidebar-examples
header:
  teaser: "/assets/img/NTanksNetworkControlOneStepScheme4.pdf"
tags:
  - example
  - mpc
  - ltv
  - nonlinear
  - one-step
  - control
date: "2021-02-24"
last_modified_at: "2021-12-27"
---
{{page.excerpt}}

***
To open this example execute the following command in the MATLAB command window
~~~m
open NTanksNetworkControlOneStep
~~~

Consider four interconnected tanks. The water levels of tank 1 to tank 4 are denoted by $h_1$, $h_2$, $h_3$, and $h_4$. The network is controlled by two pumps, whose inputs are denoted by $u_1$ and $u_2$, which are controlled by tank 1 and tank 2, in accordance with the schematic below. Each pump is connected to a three-way valve that regulates the fraction of the flow, held constant, that goes to each of the tanks supplied by the pump. Each tank has a sensor which measures its water level. Making use of mass balances and Bernoulli's law, the system dynamics, in the absence of noise, are given by
{: .text-justify}

$$
	\begin{cases}
	\dot{h}_1(t) = -\frac{a_1}{A_1}\sqrt{2gh_1(t)}+\frac{a_3}{A_1}\sqrt{2gh_3(t)}+\frac{\gamma_1k_1}{A_1}u_1(t)\\
	\dot{h}_2(t) = -\frac{a_2}{A_2}\sqrt{2gh_2(t)}+\frac{a_4}{A_2}\sqrt{2gh_4(t)}+\frac{\gamma_2k_2}{A_2}u_2(t)\\
	\dot{h}_3(t) = -\frac{a_3}{A_3}\sqrt{2gh_3(t)}+\frac{(1-\gamma_2)k_2}{A_3}u_2(t)\\
	\dot{h}_4(t)= -\frac{a_4}{A_4}\sqrt{2gh_4(t)}+\frac{(1-\gamma_1)k_1}{A_4}u_1(t)
	\end{cases}\:,
$$

where $A_i$ and $a_i$ are the cross sections of tank $i$ and of its outlet hole, respectively; constants $\gamma_1$ and $\gamma_2$ represent the fraction of the flow that passes through the valves to the lower tanks; $k_1$ and $k_2$ are the constants of proportionality between the mass flow and the input for each pump; and $g$ denotes the acceleration of gravity. Furthermore, the input of each pump is subject to a hard constraint $u_{1,2}\in [0,u^{\mathrm{sat}}]$, where $u^{\mathrm{sat}} \in \mathbb{R}^+$.
{: .text-justify}

{:refdef: style="display: block; margin-left: auto; margin-right: auto; width: 50%;"}
![image-title-here](/assets/img/NTanksNetworkControlOneStepScheme4.pdf){:class="img-responsive"}
{: refdef}


In [[1]](#references), a controller design approach is proposed to be implemented to this nonlinear network, making use of the functions of DECENTER toolbox. In short, it is based on an MPC-like scheme computed using an iterative LQR (iLQR) and a novel tracking approach. The full MATLAB code for the implementation of the decentralized controller and its simulation is included in the toolbox source code as an example. To open it execute the following command in the MATLAB command window
{: .text-justify}

~~~m
open NTanksNetworkControlOneStep
~~~

Below the prototypes of every function are displayed, as well as the main function and the implementation of the iLQR scheme.
Initialize workspace, parameters that caracterize the network, and the desired reference signal
{: .text-justify}

~~~m
%% Description
% This script simulates the extension of the original non-linear quadruple
% tank network to a network of N tanks. Although the system is nonlinear,
% it can be linearized about equilibrium points. The known LTV
% decentralized control techniques can be applied to the linearized system.
% The control vector is computed making use of an MPC-like scheme, using
% the one-step method and comparing its performance with the centralized
% solution. In addition, to take into consideration the nonlinearity of the
% dynamics of this network, an iLQR scheme is used.

%% Initilalize workspace
clear;
clear LQROneStepLTV; % clear permanent variables in LQROneStepLTV

%% Set constants and parameters that describe the network and simulation
% --- Network parameters ---
cte = getConstantsNTankNetwork(1);
% --- Simulation options ---
% Select initial state vector in cm
x0 = 20*ones(cte.n,1);
% Select the time window for the simulation in seconds
tspan = [0 600*cte.dT]; % (s)
% Compute number of discrete time-steps
simIt = tspan(2)/cte.dT+1;

%% Generate reference to lower tanks
% Initialize the reference vector for the lower tanks as a cell array,
% i.e., ref{1,k} : [h_ref1; ...; h_ref_N/2] for time instant k
ref = cell(1,simIt);
% Initialize the equilibrum levels and inputs corresponding to the
% reference reference chosen for the lower tanks for each time instant
% i.e., refEq{1,k} : x_bar and refEq{1,k} : u_bar  for time instant k,
% according to the control scheme proposed in [1].
% Note that intergral states are included in x_bar.
% Also note that a reference is needed for the instant after
refEq = cell(2,simIt);
% Gererate random step and sine references with the defined periodicity,
% which repeat every four lower tanks
for k = 1:simIt
    % initalize vector of reference water levels for the lower tanks at
    % time instant k
    ref{1,k} = zeros(cte.n/2,1);
    for j = 1:cte.n/2 % select one ammong 4 different reference waves
       switch rem(j,4)
           % sine wave with:
           %    - period: 2*pi*50s
           %    - amplitude: 10cm
           %    - mean level: 30cm
           case 0   
               ref{1,k}(j) = 30+10*cos((k-1)/50);
           % square wave with:
           %    - period: 200s
           %    - amplitude: 5cm
           %    - mean level: 25cm
           case 1  
               if rem(floor((k-1)/100),int32(2)) == 0
                    ref{1,k}(j) = 30;
               else
                    ref{1,k}(j) = 20;
               end
           % sine wave with:
           %    - period: 2*pi*35s
           %    - amplitude: 10cm
           %    - mean level: 30cm
           case 2
               ref{1,k}(j) = 30+10*cos((k-1)/35);
           % square wave with:
           %    - period: 400s
           %    - amplitude: 5cm
           %    - mean level: 25cm
           case 3
               if rem(floor((k-1)/200),int32(2)) == 0
                    ref{1,k}(j) = 30;
               else
                    ref{1,k}(j) = 20;
               end
       end
    end
    [xEq,uEq] = computeEquilibriumLevels(ref{1,k},cte);    
    refEq{1,k} = [xEq;zeros(cte.n/2,1)];
    refEq{2,k} = uEq;
end
% Do not allow square wave switching in the last time instant
ref{1,simIt}(1:2:cte.n/2,1) = ref{1,simIt-1}(1:2:cte.n/2,1);
[xEq,uEq] = computeEquilibriumLevels(ref{1,simIt},cte);    
refEq{1,simIt} = [xEq;zeros(cte.n/2,1)];
refEq{2,simIt} = uEq;
~~~

The discrete simulation and control are carried out running

~~~m
% Discrete time vector
t_disc = 0:cte.dT:tspan(2);
% Initialize temporary cell to hold the gain matrices for each of the four
% methods used
K = cell(4,simIt-1);
% Initialize discrete control vector
uControlDisc = cell(4,simIt-1);
% Initialize cell array to hold the component of the command action u_a
u_a = cell(4,simIt-1);
% Initialize measurement and state vectors
xNonLin = cell(4,simIt);
x = cell(4,simIt);
% Initialize nonlinear solver for each control action
nonLinSol = cell(4,1);
% --- Brief details of the control laws simulated (see [1] for more) --
% An MPC sheme is used where, for each time-instant, a window of gains into
% the future is computed. Then, only a fraction of those gains are actually
% used in the pumps. The tracker design is put forward in [1]. As the
% conditions of [Remark 5, 1] are satisfied, that particular case of
% [Theorem 2, 1].
% Four distinct control laws are compared:
% 1. Centralized LQR gains (d=1 gains are used out of each MPC window)
% 2. Centralized LQR gains (d>1 gains are used out of each MPC window)
% d is defined in private function getConstantsNTankNetwork
% 3. One-step LQR gains (d=1 gains are used out of each MPC window)
% 4. One-step LQR gains (d>1 gains are used out of each MPC window)
% d is defined in private function getConstantsNTankNetwork
% ---
% Iterate through every discrete time-instant
for k = 1:simIt
    if k == 1   % For the first instant initilaization is necessary
        for m = 1:4 % Iterate through every method
            % --- Measure output ---
            % water level obtained in each water level sensor (wo/ noise)
            x{m,k}(1:cte.n,1) = x0;
            % Initialize intergrator state
            x{m,k}(cte.n+1:3*cte.n/2) = x0(1:cte.n/2,1)-ref{1,k}(1:cte.n/2,1);
            % Initialize nonliner solver
            xNonLin{m,k} = x0;
            % Compute the first MPC window
            [K(m,k:min(k+cte.d(m)-1,simIt-1)),u_a(m,k:min(k+cte.d(m)-1,simIt-1))]...
                = iLQR(x{m,k},refEq(:,k:end),cte,min(cte.d(m),simIt-k),m);
            % Control law [1]: u(k) = -K(k)(x(k)-x_bar(k))+u_bar(k)+u_a(k)
            uControlDisc{m,k} = -K{m,k}*(x{m,k}-refEq{1,k})+u_a{m,k}+refEq{2,k};
            % Saturate commands to the pumps
            uControlDisc{m,k}(uControlDisc{m,k}< cte.uMin) = cte.uMin;
            uControlDisc{m,k}(uControlDisc{m,k}>cte.uMax) = cte.uMax;
            % Simulate nonlinear dynamics
            nonLinSol{m,1} = ode45(@(t,x) xdotContinuous(x,uControlDisc{m,min(floor(t/cte.dT)+1,...
                round(t_disc(k+1)/cte.dT))},cte),[t_disc(k) t_disc(k+1)],x0);
        end
    elseif k ~= simIt
         for m = 1:4 % Iterate through every method
             % --- Measure output ---
             x{m,k}(1:cte.n,1) = deval(nonLinSol{m,1}, t_disc(k));
             % --- Integrate tracking error ---
             x{m,k}(cte.n+1:3*cte.n/2) = x{m,k-1}(cte.n+1:3*cte.n/2)+x{m,k}(1:cte.n/2,1)...
                 -ref{1,k}(1:cte.n/2,1);
             % Anti windup for integral action according to [1]
             for j = 1:cte.n/2
                if abs(x{m,k}(cte.n+j)) > cte.AntiWU(m)
                    x{m,k}(cte.n+j) = cte.AntiWU(m)*abs(x{m,k}(cte.n+j))/x{m,k}(cte.n+j);
                end
             end
             xNonLin{m,k} = deval(nonLinSol{m,1}, t_disc(k));
             % If a new MPC window needs to be computed this time-instant
             if rem(k-1,cte.d(m)) == 0
                [K(m,k:min(k+cte.d(m)-1,simIt-1)),u_a(m,k:min(k+cte.d(m)-1,simIt-1))]...
                    = iLQR(x{m,k},refEq(:,k:end),cte,min(cte.d(m),simIt-k),m);
             end

             uControlDisc{m,k} = -K{m,k}*(x{m,k}-refEq{1,k})+u_a{m,k}+refEq{2,k};
             uControlDisc{m,k}(uControlDisc{m,k}<0) = 0;
             uControlDisc{m,k}(uControlDisc{m,k}>cte.uMax) = cte.uMax;
             % Simulate nonlinear dynamics
             nonLinSol{m,1} = ode45(@(t,x) xdotContinuous(x,...
                 uControlDisc{m,min(floor(t/cte.dT)+1,round(t_disc(k+1)/cte.dT))},cte),...
                 [t_disc(k) t_disc(k+1)],deval(nonLinSol{m,1}, t_disc(k)));
         end
    else % for the last time-instant
        % compute only the state at the last time-instant
        for m = 1:4
            xNonLin{m,k} = deval(nonLinSol{m,1}, t_disc(k));
        end
    end
    % Show status
    if k == 1
        fprintf('Running simulation of N tank network: ');
        delstatus = '';
    end
    status = strcat(sprintf('%3.1f', 100*k/simIt),'%%');
    fprintf([delstatus, status]);
    delstatus= repmat(sprintf('\b'),1,length(status)-1);
    if k == simIt fprintf('\n'); end
end
~~~

Notice that the computation of the gains for each time-instant is given by an iLQR scheme, which is described thoroughly in [[1]](#references) and implemented by the following function
{: .text-justify}

~~~m
%% iLQR - Description
% This function computes the iLQR (iterative LQR gains). It is necessary
% because of the nonlinearity of the N tank network. The computation of the
% gains for a finite window using either a centralized or decentralized
% method require that the future dynamics of the system are known. However,
% for this system, the system dynamics in the future depend on the future
% state. iLQR iterative procedure where the future dynamics are being
% updated every time a new MPC window is computed, until convergence. For
% more details see [1].
% Input:    - x0: state at the beginning of the new window
%           - ref: cell array containing x_bar and u_bar
%           - d: number of gains to output out of those computed for the
%           whole window
%           - cte: struct with the necessary constants and parameters
%           - m: number of the method used for the computation of LQR gains
% Output:   - K: struct of LQR gains
%           - u_a: struct of addictional command actions
function [K,u_a] = iLQR(x0,ref,cte,d,m)
    % Initialize cell array for the future dynamics of the system
    system = cell(cte.T(m)+1,7);
    % --- Forward pass variables ---
    % sequence of states throughout the window
    x = cell(1,cte.T(m)+1);
    % sequence of additional command action throughout the window
    u_a = cell(1,cte.T(m)+1);
    % sequence of command action throughout the window
    uControlDisc = cell(1,cte.T(m));
    % sequence of command action throughout the window (previous iteration)
    % to check when convergence is reached
    prevuControlDisc = cell(1,cte.T(m));
    % Time of discrete-time intants
    t_disc = 0:cte.dT:cte.T(m);
    % Perform the iLQR iterations up to a maximum of cte.iLQRIt iterations
    for k = 1:cte.iLQRIt
       % Forward pass
       if k == 1
           % Initial propagated system assumes level is mantained constant
           system(1,:) = getDiscreteDynamics(x0,cte);
           for i = 2:cte.T(m)+1
               system(i,:) = system(1,:);
           end
       else
           for i = 1:cte.T(m) % Simulate forward pass
                if i == 1
                    % Compute the linearized dynamics for the first instant
                    system(i,:) = getDiscreteDynamics(x0,cte);
                    % (assumes level is mantained constant)
                    for l = i+1:i+cte.dTlin-1
                        system(l,:) = system(i,:);
                    end
                    % Measure simulated output
                    x{1,i} = x0;
                    % Compute aditional command action
                    u_a{1,i} = (cte.h*system{i,2}(1:cte.n,1:cte.m))\cte.h*...
                       (ref{1,min(i+1,size(ref,2))}(1:cte.n,1)-ref{1,min(i,size(ref,2))}(1:cte.n,1));
                    % Control law [1]: u(k) = -K(k)(x(k)-x_bar(k))+u_bar(k)+u_a(k)
                    uControlDisc{1,i} = -K{i,1}*(x{1,i}-ref{1,min(i,size(ref,2))})...
                        +u_a{1,i}+ref{2,min(i,size(ref,2))};
                    % Saturate commands to the pumps
                    uControlDisc{1,i}(uControlDisc{1,i}<cte.uMin) = cte.uMin;
                    uControlDisc{1,i}(uControlDisc{1,i}>cte.uMax) = cte.uMax;
                    % Simulate nonlinear dynamics
                    nonLinSol = ode45(@(t,x) xdotContinuous(x,uControlDisc{1,min(floor(t/cte.dT)+1,round(t_disc(i+1)/cte.dT))},cte),[t_disc(i) t_disc(i+1)],x0(1:cte.n,1));
                    x{1,i+1}(1:cte.n,1) = deval(nonLinSol, t_disc(i+1));
                    % Integrate tracking error
                    x{1,i+1}(cte.n+1:3*cte.n/2) = x{1,i}(cte.n+1:3*cte.n/2)+x{1,i+1}(1:cte.n/2,1)-ref{1,i+1}(1:cte.n/2,1);
                else
                    % if a new linearization is necessary
                    if rem(i-1,cte.dTlin) == 0
                       system(i,:) = getDiscreteDynamics(x{1,i},cte);
                       for l = i+1:cte.T(m)+1
                           system(l,:) = system(i,:);
                       end
                    end
                    % Compute aditional command action
                    u_a{1,i} = (cte.h*system{i,2}(1:cte.n,1:cte.m))\cte.h*(ref{1,min(i+1,size(ref,2))}(1:cte.n,1)-ref{1,min(i,size(ref,2))}(1:cte.n,1));
                    % Control law [1]: u(k) = -K(k)(x(k)-x_bar(k))+u_bar(k)+u_a(k)
                    uControlDisc{1,i} = -K{i,1}*(x{1,i}-ref{1,min(i,size(ref,2))})+u_a{1,i}++ref{2,min(i,size(ref,2))};
                    uControlDisc{1,i}(uControlDisc{1,i}<cte.uMin) = cte.uMin;
                    uControlDisc{1,i}(uControlDisc{1,i}>cte.uMax) = cte.uMax;
                    % Simulate nonlinear dynamics
                    nonLinSol = ode45(@(t,x) xdotContinuous(x,uControlDisc{1,min(floor(t/cte.dT)+1,round(t_disc(i+1)/cte.dT))},cte),[t_disc(i) t_disc(i+1)],x{1,i}(1:cte.n,1));
                    x{1,i+1}(1:cte.n,1) = deval(nonLinSol, t_disc(i+1));
                    % Integrate tracking error
                    x{1,i+1}(cte.n+1:3*cte.n/2) = x{1,i}(cte.n+1:3*cte.n/2)+x{1,i+1}(1:cte.n/2,1)-ref{1,min(i+1,size(ref,2))}(1:cte.n/2,1);
                    % Anti windup for integral action according to [1]
                    for j = 1:cte.n/2
                       if abs(x{1,i}(cte.n+j)) > cte.AntiWU(m)
                           x{1,i}(cte.n+j) = cte.AntiWU(m)*abs(x{1,i}(cte.n+j))/x{1,i}(cte.n+j);
                       end
                    end
                end
           end
       end
       % --- stopping criterion ---
       % stop the iteartions if the maximum difference
       % in relation to the actuation computed in the previous iteration
       % falls under cte.iLQReps
       if k > 2
           dif = zeros(1,cte.T(m));
           for i = 1:cte.T(m)
                dif(1,i) = norm(prevuControlDisc{1,i}-uControlDisc{1,i})/...
                    norm(uControlDisc{1,i});
           end
           if max(dif) < cte.iLQReps
               break;
           end
       end
       prevuControlDisc = uControlDisc;
       % --- Compute LQR gains ---
       if m<= 2 % Centralized
            [K,~] = LQRCentralizedLTV(system(:,1:4),cte.T(m));
       else % One-step
            [K,~] = LQROneStepLTV(system(:,1:4),cte.E,cte.T(m));
       end
       % check if maximum number of iterations was reached and issue
       % warning
       if k == cte.iLQRIt
           fprintf(sprintf('The maximum number of iLQR iterations was reached before convergence for method number %d.\n',m));
       end
    end
    % Output only gains and additional command action that are used
    K = transpose(K(1:d,1));
    u_a = u_a(1,1:d);
end
~~~

In addition, the prototypes of several other functions used, which are included in the full MATLAB code, are presented below

~~~m
%% getConstantsNTankNetwork - Description
% This function outputs a struct of constants of the model dynamics.
% Input:    -flagEqMatrices: only if true compute equilibrium matrices
% Output:   -Cte: struct with the necessary constants and parameters
function cte = getConstantsNTankNetwork(flagEqMatrices)

%% getConstantsNTankNetwork - Description
% This function computes matrices alpha and beta according to [1] for
% equilibrium level computation
% Output:  -alpha, beta: matrices to compute water level
% WARNING: Uses symbolic toolbox
function [alpha,beta] = getEquilibriumMatrices()

%% computeEquilibriumLevels - Description
% This function computes the equilibrium levels corresponding to a given
% reference to the lower tanks, according to [1]
% Input :   - ref: reference vector to the lower tanks
%           - cte: parameters of the network
% Output:   - xEq: vector of equilibrium water levels
%           - uEq: vector of equilibrium pump actuations
function [xEq,uEq] = computeEquilibriumLevels(ref,cte)

%% getDiscreteDynamics - Description
% This function computes the linearized discrete model for a given state.
% The equilibrium state is computed around the level in the lower tanks.
% Input:    - x: state vector
%           - cte: struct of constants of the model dynamics
% Output:   - linDynamics: 1x7 cell with matrices A,C,Q,R,B, equilibrium
% state vector, and equilibrium control vector, uEq (in this order).
% Note: The state vector in this function includes integral states
function linDynamics = getDiscreteDynamics(x,cte)

%% xdotContinuous - Description
% This function computes the derivative of the state vector using the non
% linear dynamics of the model, for a given state and control vector.
% Input:    - x: state vector
%           - u: actuation vector
% Output:   - xdot: derivative of the state vector
function xdot = xdotContinuous(x,u,cte)

%% Dxdot - Description
% This function computes matrices A and B of the continuous linearized
% model given an equilibrium state.
% Input:    - x_eq: equilibrium state vector
%           - cte: struct with constants of the model dynamics
function [A,B] = Dxdot(x_eq,cte)
~~~

The evolution of the water level in each of the tanks is shown below
<div class="row">
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneSteph1.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneSteph2.svg" style="width:100%">
  </div>
</div>
<div class="row">
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneSteph3.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneSteph4.svg" style="width:100%">
  </div>
</div>

The evolution of inputs to the pumps is shown below
<div class="row">
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepu1.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepu2.svg" style="width:100%">
  </div>
</div>

{: .text-justify}


# References
[1] {% for paper in site.data.references%}{% if paper.key == "Pedroso2021LQROSLTV" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}
