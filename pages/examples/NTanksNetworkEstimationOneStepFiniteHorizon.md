---
layout: single
title: "Decentralized estimation of nonlinear N-tank network"
showTitle: true
excerpt: "Application of decentralized estimation to the N-tank network, using the one-step and finite-horizon methods."
permalink: /examples/NTanksNetworkEstimationOneStepFiniteHorizon/
classes: wide
sidebar:
  title: "Examples"
  nav: sidebar-examples
header:
  teaser: "/assets/img/NTanksNetworkControlOneStepSchemeN.pdf"
tags:
  - example
  - ltv
  - nonlinear
  - one-step
  - finite-horizon
  - estimation
date: "2021-07-10"
last_modified_at: "2021-12-27"
---
{{page.excerpt}}

***

The quadruple-tank network presented in [[1]](#references), whose MATLAB code is presented in [this example](/examples/QuadrupleTankNetworkControlOneStep/), can be generalized to N tanks. Consider $N$ interconnected tanks, where $N$ is an even integer. The water level of tank $i$ is denoted by $h_i$. The network is actuated by $N/2$ pumps, which are controlled by the lower tanks, whose inputs are denoted by $u_i$ for $i=1,...,N/2$, in accordance with the schematic. Each pump is connected to a three-way valve that regulates the fraction of the flow, held constant, that goes to each of the tanks supplied by the pump. Each tank has a sensor, which measures its water level. Making use of mass balances and Bernoulli's law, the system dynamics, in the absence of noise, are given by
{: .text-justify}

$$
	\begin{cases}
	A_i\dot{h}_i(t) = -a_i\sqrt{2gh_i(t)}+a_{\frac{N}{2}+i}\sqrt{2gh_{\frac{N}{2}+i}(t)}+\gamma_ik_iu_i(t), \:\: i = 1,...,N/2\\
	A_i\dot{h}_i(t) = -a_i\sqrt{2gh_i(t)}+(1-\gamma_{i-\frac{N}{2}-1})k_{i-\frac{N}{2}-1}u_{i-\frac{N}{2}-1}(t), \:\: i = \frac{N}{2}+2,...,N\\
	A_{\frac{N}{2}+1}\dot{h}_{\frac{N}{2}+1}(t) = -a_{\frac{N}{2}+1}\sqrt{2gh_{\frac{N}{2}+1}(t)}+(1-\gamma_{\frac{N}{2}})k_{\frac{N}{2}}u_{\frac{N}{2}}(t)
	\end{cases}\:,
$$

where $A_i$ and $a_i$ are the cross sections of tank $i$ and of its outlet hole, respectively; the constant $\gamma_i$ represents the fraction of the flow that passes through the valve $i$ to the lower tanks; $k_i$ is the constant of proportionality between the mass flow and the input of pump $i$; and $g$ denotes the acceleration of gravity. Furthermore, the input of each pump is subject to a hard constraint $u_i\in [0,u^{\mathrm{sat}}]$, where $u^{\mathrm{sat}} \in \mathbb{R}^+$.
{: .text-justify}

{:refdef: style="display: block; margin-left: auto; margin-right: auto; width: 70%;"}
![image-title-here](/assets/img/NTanksNetworkControlOneStepSchemeN.pdf){:class="img-responsive"}
{: refdef}

The application of the algorithms to this network shows their scalability.
In [[1]](#references), a filter design approach is proposed to be implemented to this nonlinear network, making use of the functions of DECENTER toolbox. In short, each tank has i) only access to the measurement of its water level, and ii) may receive state estimates of other tanks through communication. The approach followed consists in the implementation of a local filter in each of the tanks, which estimates exclusively its own water level, relying on a communication link with some of the other tanks. The full MATLAB code for the implementation of the decentralized filter and its simulation is included in the toolbox source code as an example. To open it execute the following command in the MATLAB command window
{: .text-justify}

{: .text-justify}
~~~m
open NTanksNetworkEstimationOSFH
~~~

Below the prototypes of every function are displayed, as well as the main function.

First, initialize workspace, parameters that caracterize the network, and the desired reference signal
{: .text-justify}
~~~m
%% Description
% This script simulates the extention of the original non-linear quadruple.
% The control vector is computed using predefined law (since we only want
% to study the estimation performance). In this script observers of the LTV
% system are also implemented. It includes a centralized observer and two
% distributed observers, whose gains are computed using the One Step and
% Finite Horizon Algorithms.

%% Descentralization scheme
% The distributed scheme presented assumes the control vector is determined
% in a centralized manner and  is available to the local observers via
% communication. It consists of 4 local observers in each tank,
% estimating their own position

%% Set constants and parameters that describe the network and simulation
clear;
% --- Network parameters ---
cte = getConstantsNTankNetwork(1);
% --- Simulation options ---
% Select initial state vector in cm
x0 = 20*ones(cte.n,1);
% Select the time window for the simulation in seconds
tspan = [0 800*cte.dT]; % (s)
% Compute number of discrete time-steps
simIt = tspan(2)/cte.dT+1;
LinIt = tspan(2)/cte.dTlin+1;
% Select default seed for random number generation, for debbuging purposes
rng(7);

%% Initialize observe
% Define initial error covariance matrix
cte.P0 = 10*eye(cte.n);
xhat0 = x0+transpose(mvnrnd(zeros(cte.n,1),cte.P0));
% Initialize vector of observations
xhatC = zeros(cte.n,simIt-1);
xhatOS = zeros(cte.n,simIt-1);
% Initialize temporary cell to hold the covariance matrices
PC = cell(simIt,1);
POS = cell(simIt,1);
% Variable to hold the previous predicted estimation error covariance
PpredC = zeros(cte.n);
PpredOS = zeros(cte.n);
~~~

The simulation of the centralized and one-step solutions is carried out running
~~~m
%% Discrete simulation for centralized and OS
% Discrete time vector
t_disc = 0:cte.dT:tspan(2);
% Initialize discrete control vector
uControlDiscrete = zeros(cte.m,simIt);
% Initialize measurement vector
y = cell(1,simIt-1);
x_disc = zeros(cte.n,simIt-1);
% Initialize the discrete time varying system that corresponds to the
% linearized non-linear model
system = cell(simIt-1,7);
systemC = cell(LinIt-1,7);
systemOS = cell(LinIt-1,7);

for i = 1:simIt-1
    % Define indices for the linearized system model to take into account
    j_ = floor((i-1)/cte.dTlin)+1;   
    if i == 1     
         % Compute the linearized dynamics for the first instant
         systemC(i,:) = getDiscreteDynamics(xhat0,cte);
         systemOS(i,:) = getDiscreteDynamics(xhat0,cte);
         % Get control vector with centralized LQR
         uControlDiscrete(:,floor(t_disc(i)/cte.dT)+1) = getControl((i-1)*cte.dT,cte.m);
         % Simulate nonlinear dynamics
         system(i,:) = getDiscreteDynamics(x0,cte);
         nonLinSol = ode45(@(t,x) xdotContinuous(x,uControlDiscrete(:,min(floor(t/cte.dT)+1,floor(t_disc(i+1)/cte.dT))),cte),[0 t_disc(i+1)],x0);
         x_disc(:,i) = deval(nonLinSol, t_disc(i+1)) + transpose(mvnrnd(zeros(cte.n,1),system{i,3}));

         % Observer dynamics
         % Measure output (systemC is used beacuse C and R are constant hence equal for systemC and systemOS )
         y{i,1} = systemC{j_,2}*x_disc(:,i) + transpose(mvnrnd(zeros(cte.o,1),systemC{j_,4}));

         % ----- Centralized -----
         % Compute observer gain
         PpredC = systemC{j_,1}*cte.P0*systemC{j_,1}'+systemC{j_,3};
         [KC,PpredC,PC{i,1}] = kalmanCentralizedLTV(systemC(j_,:),PpredC);
         % Predict
         xhatC_ = systemC{j_,6}+systemC{j_,1}*(xhat0-systemC{j_,6})+...
              systemC{j_,5}*(uControlDiscrete(:,floor(t_disc(i)/cte.dT)+1)-systemC{j_,7});
         % Update
         xhatC(:,i) = xhatC_ + KC*(y{i,1}-systemC{j_,2}*xhatC_);

         % ----- One Step -----
         % Compute one step observer gain
         PpredOS = systemOS{j_,1}*cte.P0*systemOS{j_,1}'+systemOS{j_,3};
         [KOS,PpredOS,POS{i,1}] = kalmanOneStepLTV(systemOS(j_,1:4),cte.E,PpredOS);
         % Predict
         xhatOS_ = systemOS{j_,6}+systemOS{j_,1}*(xhat0-systemOS{j_,6})+...
              systemOS{j_,5}*(uControlDiscrete(:,floor(t_disc(i)/cte.dT)+1)-systemOS{j_,7});
         % Update
         xhatOS(:,i) = xhatOS_ + KOS*(y{i,1}-systemOS{j_,2}*xhatOS_);

    else
         % Recompute the linearized dynamics of the system with a given
         % periodicity
         if rem(i-1,cte.dTlin) == 0
            systemC(j_,:) = getDiscreteDynamics(xhatC(:,i-1),cte);
            systemOS(j_,:) = getDiscreteDynamics(xhatOS(:,i-1),cte);
         end
         % Get control vector with predefined control law
         uControlDiscrete(:,floor(t_disc(i)/cte.dT)+1) = getControl((i-1)*cte.dT,cte.m);
         % Simulate nonlinear dynamics
         [system(i,:)] = getDiscreteDynamics(x_disc(:,i-1),cte);
         nonLinSol = ode45(@(t,x) xdotContinuous(x,uControlDiscrete(:,min(floor(t/cte.dT)+1,floor(t_disc(i+1)/cte.dT))),cte),[t_disc(i) t_disc(i+1)],x_disc(:,i-1));
         x_disc(:,i) = deval(nonLinSol, t_disc(i+1)) + transpose(mvnrnd(zeros(cte.o,1),system{i,3}));
         % Observer dynamics
         % Measure output (systemC is used beacuse C and R are constant hence equal for systemC and systemOS )
         y{i,1} = systemC{j_,2}*x_disc(:,i) + transpose(mvnrnd(zeros(cte.n,1),systemC{j_,4}));
         % ----- Centralized -----
         % Compute observer gain
         [KC,PpredC,PC{i,1}] = kalmanCentralizedLTV(systemC(j_,1:4),PpredC);
         xhatC_ = systemC{j_,6}+systemC{j_,1}*(xhatC(:,i-1)-systemC{j_,6})+...
              systemC{j_,5}*(uControlDiscrete(:,floor(t_disc(i)/cte.dT)+1)-systemC{j_,7});
         xhatC(:,i) = xhatC_ + KC*(y{i,1}-systemC{j_,2}*xhatC_);

         % ----- One Step -----
         % Compute one step observer gain
         [KOS,PpredOS,POS{i,1}] = kalmanOneStepLTV(systemOS(j_,1:4),cte.E,PpredOS);
         % Predict
         xhatOS_ = systemOS{j_,6}+systemOS{j_,1}*(xhatOS(:,i-1)-systemOS{j_,6})+...
              systemOS{j_,5}*(uControlDiscrete(:,floor(t_disc(i)/cte.dT)+1)-systemOS{j_,7});
         % Update
         xhatOS(:,i) = xhatOS_ + KOS*(y{i,1}-systemOS{j_,2}*xhatOS_);
    end
end
~~~

The simulation of the finite-horizon solution is carried out running
~~~m
%% Simulate FH propagated system
% Initialize the discrete time varying system that corresponds to the
% linearized non-linear model
systemFH = cell(simIt-1,7);
% Initialize vector of predictions
x_predictFH = zeros(cte.n,simIt-1);
% Initialize vector of observations
xhatFH = zeros(cte.n,simIt-1);
% Define algorithm parameters
FH_w = 40;
FH_d = 0;
opts.verbose = false;
opts.epsl = 1e-6;
opts.maxOLIt = 100;
% Compute number of FH windows
nW = floor((simIt-1)/(FH_w-FH_d));
% Number of remaining instants that do not fill a window
endW = rem(simIt-1,FH_w-FH_d);
% Initialize temporary cell to hold the covariance matrices and gains
KFH = cell(simIt-1,1);
PFH = cell(simIt-1,1);

for i = 1:floor(nW)
    if i == 1
        %% First window
        % Compute system dynamics for the window
        systemFH(i,:) = getDiscreteDynamics(xhat0,cte);
        x_predictFH(:,1) = systemFH{i,1}*(xhat0-systemFH{i,6})+...
            systemFH{i,5}*(getControl((i-1)*cte.dT,cte.m)-systemFH{i,7})+systemFH{i,6};
        for j = 2:FH_w
            if rem(j-1,cte.dTlin) == 0
                systemFH(j,:) = getDiscreteDynamics(x_predictFH(:,j-1),cte);
            else
                systemFH(j,:) = systemFH(j-1,:);
            end
            x_predictFH(:,j) = systemFH{j,1}*(x_predictFH(:,j-1)-systemFH{j,6})+...
                systemFH{j,5}*(getControl((j-1)*cte.dT,cte.m)-systemFH{j,7})+systemFH{j,6};
        end
        % Compute gains
        P0_pred = systemFH{i,1}*cte.P0*systemFH{i,1}'+systemFH{i,3};
        [auxK, auxP] = kalmanFiniteHorizonLTV(systemFH(1:end,1:4),cte.E,FH_w,P0_pred,opts);
        KFH((i-1)*(FH_w-FH_d)+1:i*(FH_w-FH_d),1) = auxK(1:(FH_w-FH_d));
        PFH((i-1)*(FH_w-FH_d)+1:i*(FH_w-FH_d),1) = auxP(1:(FH_w-FH_d));
        % Simulate dynamics
        for j = 1:FH_w-FH_d
             if j == 1
                 % Predict
                 xhatFH_ = systemFH{j,6}+systemFH{j,1}*(xhat0-systemFH{j,6})+...
                      systemFH{j,5}*(uControlDiscrete(:,floor(t_disc(j)/cte.dT)+1)-systemFH{j,7});
                 % Update
                 xhatFH(:,j) = xhatFH_ + KFH{1,1}*(y{j,1}-systemFH{j,2}*xhatFH_);
             else
                 % Predict
                 xhatFH_ = systemFH{j,6}+systemFH{j,1}*(xhatFH(:,j-1)-systemFH{j,6})+...
                      systemFH{j,5}*(uControlDiscrete(:,floor(t_disc(j)/cte.dT)+1)-systemFH{j,7});
                 % Update
                 xhatFH(:,j) = xhatFH_ + KFH{j,1}*(y{j,1}-systemFH{j,2}*xhatFH_);
             end
        end
    else
        %% Remaining windows window
        % Compute system dynamics for the window
        j_ = (i-1)*(FH_w-FH_d)+1;
        systemFH(j_,:) = getDiscreteDynamics(xhatFH(:,j_-1),cte);
        x_predictFH(:,j_) = systemFH{j_,1}*(xhatFH(:,j_-1)-systemFH{j_,6})+...
            systemFH{j_,5}*(getControl((j_-1)*cte.dT,cte.m)-systemFH{j_,7})+systemFH{j_,6};
        for j = j_+1:j_+FH_w-1
            if rem(j-j_,cte.dTlin) == 0
                systemFH(j,:) = getDiscreteDynamics(x_predictFH(:,j-1),cte);
            else
                systemFH(j,:) = systemFH(j-1,:);
            end
            x_predictFH(:,j) = systemFH{j,1}*(x_predictFH(:,j-1)-systemFH{j,6})+...
                systemFH{j,5}*(getControl((j-1)*cte.dT,cte.m)-systemFH{j,7})+systemFH{j,6};
        end
        % Compute gains
        P0_pred = systemFH{(i-1)*(FH_w-FH_d),1}*PFH{(i-1)*(FH_w-FH_d),1}*systemFH{(i-1)*(FH_w-FH_d),1}'+systemFH{(i-1)*(FH_w-FH_d),3};
        [auxK, auxP] = kalmanFiniteHorizonLTV(systemFH((i-1)*(FH_w-FH_d)+1:end,1:4),cte.E,min(FH_w,size(systemFH((i-1)*(FH_w-FH_d)+1:end,1),1)),P0_pred,opts);
        KFH((i-1)*(FH_w-FH_d)+1:i*(FH_w-FH_d),1) = auxK(1:(FH_w-FH_d));
        PFH((i-1)*(FH_w-FH_d)+1:i*(FH_w-FH_d),1) = auxP(1:(FH_w-FH_d));
        % Simulate dynamics
        for j = j_:j_+FH_w-FH_d-1
             % Predict
             xhatFH_ = systemFH{j,6}+systemFH{j,1}*(xhatFH(:,j-1)-systemFH{j,6})+...
                  systemFH{j,5}*(uControlDiscrete(:,floor(t_disc(j)/cte.dT)+1)-systemFH{j,7});
             % Update
             xhatFH(:,j) = xhatFH_ + KFH{j,1}*(y{j,1}-systemFH{j,2}*xhatFH_);
        end

    end
end
%% Remaining partial window of the simulation
if endW > 0
  % Compute system dynamics for the window
    j_ = i*(FH_w-FH_d)+1;
    systemFH(j_,:) = getDiscreteDynamics(xhatFH(:,j_-1),cte);
    x_predictFH(:,j_) = systemFH{j_,1}*(xhatFH(:,j_-1)-systemFH{j_,6})+...
            systemFH{j_,5}*(getControl((j_-1)*cte.dT,cte.m)-systemFH{j_,7})+systemFH{j_,6};
    for j = j_+1:j_-1+endW
        if rem(j-j_,cte.dTlin) == 0
            systemFH(j,:) = getDiscreteDynamics(x_predictFH(:,j-1),cte);
        else
            systemFH(j,:) = systemFH(j-1,:);
        end
        x_predictFH(:,j) = systemFH{j,1}*(x_predictFH(:,j-1)-systemFH{j,6})+...
                systemFH{j,5}*(getControl((j-1)*cte.dT,cte.m)-systemFH{j,7})+systemFH{j,6};
    end
    % Compute gains
    P0_pred = systemFH{(floor(nW))*(FH_w-FH_d),1}*PFH{(floor(nW))*(FH_w-FH_d),1}*systemFH{(floor(nW))*(FH_w-FH_d),1}'+systemFH{(floor(nW))*(FH_w-FH_d),3};
    [auxK, auxP] = kalmanFiniteHorizonLTV(systemFH((i)*(FH_w-FH_d)+1:end,1:4),cte.E,endW,P0_pred,opts);  
    KFH(floor(nW)*(FH_w-FH_d)+1:end,1) = auxK(1:endW);
    PFH(floor(nW)*(FH_w-FH_d)+1:end,1) = auxP(1:endW);
    % Simulate dynamics
    for j = j_:j_-1+endW
         % Predict
         xhatFH_ = systemFH{j,6}+systemFH{j,1}*(xhatFH(:,j-1)-systemFH{j,6})+...
              systemFH{j,5}*(uControlDiscrete(:,floor(t_disc(j)/cte.dT)+1)-systemFH{j,7});
         % Update
         xhatFH(:,j) = xhatFH_ + KFH{j,1}*(y{j,1}-systemFH{j,2}*xhatFH_);
    end
end
~~~

In addition, the prototypes of several other functions used, which are included in the full MATLAB code, are presented below

~~~m
%% getControl - Description
% This function outputs the known control law
% Inputs:   - t: time instant
%           - m: number of pumps
% Outputs:  - du: control action

%% getConstantsNTankNetwork - Description
% This function outputs a struct of constants of the model dynamics.
% Input:    -flagEqMatrices: only if true compute equilibrium matrices
% Output:   -Cte: struct with the necessary constants and parameters

%% getEquilibriumMatrices - Description
% This function computes matrices alpha and beta according to [1] for
% equilibrium level computation
% Output:  -alpha, beta: matrices to compute water level
% WARNING: Uses symbolic toolbox

%% getDiscreteDynamics - Description
% This function computes the linearized discrete model for a given state.
% The equilibrium state is computed around the level in the lower tanks.
% Input:    - x: state vector
%           - cte: struct of constants of the model dynamics
% Output:   - linDynamics: 1x7 cell with matrices A,C,Q,R,B, equilibrium
% state vector, and equilibrium control vector, uEq (in this order).
% Note: The state vector in this function includes integral states

%% computeEquilibriumLevels - Description
% This function computes the equilibrium levels corresponding to a given
% reference to the lower tanks, according to [1]
% Input :   - ref: reference vector to the lower tanks
%           - cte: parameters of the network
% Output:   - xEq: vector of equilibrium water levels
%           - uEq: vector of equilibrium pump actuations

%% xdotContinuous - Description
% This function computes the derivative of the state vector using the non
% linear dynamics of the model, for a given state and control vector.
% Input:    - x: state vector
%           - u: actuation vector
% Output:   - xdot: derivative of the state vector

%% Dxdot - Description
% This function computes matrices A and B of the continuous linearized
% model given an equilibrium state.
% Input:    - x_eq: equilibrium state vector
%           - cte: struct with constants of the model dynamics
~~~

The evolution of the water level in two of the tanks and each of the estimates using the different methods is shown below

  {:refdef: style="display: block; margin-left: auto; margin-right: auto; width: 80%;"}
  ![image-title-here](/assets/img/NTanksNetworkEstimationOSFH_h13.svg){:class="img-responsive"}
  {: refdef}

  {:refdef: style="display: block; margin-left: auto; margin-right: auto; width: 80%;"}
  ![image-title-here](/assets/img/NTanksNetworkEstimationOSFH_h31.svg){:class="img-responsive"}
  {: refdef}



# References
[1] {% for paper in site.data.references%}{% if paper.key == "Pedroso2021KalmanOSFHLTV" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}
