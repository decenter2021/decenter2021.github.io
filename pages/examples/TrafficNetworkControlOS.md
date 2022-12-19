---
layout: single
title: "Decentralized control strategy for congested urban road networks"
showTitle: true
excerpt: "Application of decentralized control to a large-scale congested urban road network using the one-step method."
permalink: /examples/TrafficNetworkControlOS/
classes: wide
sidebar:
  title: "Examples"
  nav: sidebar-examples
header:
  teaser: "/assets/img/roundabout_traff_net-eps-converted-to.svg"
tags:
  - example
  - mpc
  - lti
  - nonlinear
  - one-step
  - control
date: "2021-12-28"
last_modified_at: "2021-12-28"
---
{{page.excerpt}}

***
To open this example execute the following command in the MATLAB command window
{: .text-justify}
~~~m
open TrafficNetworkControlOS
~~~

Consider a traffic network composed of **links** and **signalized junctions**. For each junction, there is a set of **stages**. Each stage has an associated **green time**, during which a set of links that are directed towards that junction are given right of way. In this example the traffic network topology is represented by a **directed graph**. In this framework, each junction is represented by a vertex and each link by a directed edge. For example, the following illustrative traffic network (Fig. from [[1]](#references))
{: .text-justify}
{:refdef: style="display: block; margin-left: auto; margin-right: auto; width: 50%;"}
![image-title-here](/assets/img/roundabout_traff_net-eps-converted-to.svg){:class="img-responsive"}
{: refdef}
can be graphically represented as a directed graph as (Fig. from [[1]](#references))
{:refdef: style="display: block; margin-left: auto; margin-right: auto; width: 50%;"}
![image-title-here](/assets/img/ill_traff_net.svg){:class="img-responsive"}
{: refdef}

The convention followed to model the topology of the traffic network according to this framework is detailed in [[Section 2.1, 1]](#references). In this example, the network dynamics are modelled making use of the **store-and-forward model**, which is a linear macroscopic vehicle dynamics model. This model is thoroughly detailed in [[Section 2.1, 1]](#references) and, in particular for the illustrative network above, in [[Section 2.3, 1]](#references).
{: .text-justify}


Illustrative parameters that characterize the network above are provided in the directory of the MATLAB script. The topology and parameters of a generic traffic network are read in MATLAB from a series of plain text files, whose templates, as well as a support spreadsheet are provided. The model is, then, available in MATLAB as a structure obtained with
{: .text-justify}
~~~m
%% Import traffic network model
model = modelSynthesis("./data/");
~~~

In [[1]](#references), two control strategies are proposed to control the green times of each stage, making use of the functions of DECENTER toolbox. In short, they are based on an MPC-like scheme applied to two formulations of the store-and-forward model. The full MATLAB code for the implementation of the decentralized controller and its simulation is included in the toolbox source code as an example. To open it execute the following command in the MATLAB command window
{: .text-justify}
~~~m
open TrafficNetworkControlOS
~~~

Below the prototypes of every function are displayed, as well as the main steps of the control strategy. After loading the model, a **modal decomposition** is performed and a **historic actuation** is generated, as described in [[1]](#references)
{: .text-justify}
~~~m
%% Modal decomposition as detailed in [1]
% Separation of controllable and uncontrolable modes
ctrbM = ctrb(model.A,model.Bg); % controlability matrix
r = rank(ctrbM);
Z = size(model.A,1);
H = orth(ctrbM);
V = null(H');
W = [H V];
A_hat = W\model.A*W;
Bg_hat = W\model.Bg;
Bg1_hat = [eye(r) zeros(r,Z-r)]*Bg_hat;
A1_hat = [eye(r) zeros(r,Z-r)]*A_hat*[eye(r);zeros(Z-r,r)];

%% Generate historic actuation as detailed in [1]
gN_DTUC = -(1/model.C)*(Bg1_hat'*Bg1_hat)\Bg1_hat'*[eye(r) zeros(r,Z-r)]/W*model.d;
gN_D2TUC = -(1/model.C)*(model.BG'*model.BG)\model.BG'*model.d;
~~~

Then, the linear feedback gain matrices are computed for 4 decentralized control strategies:
{: .text-justify}
- DTUC with decentralized configuration $\Psi$
- DTUC with decentralized configuration $\Phi$
- D2TUC with decentralized configuration $\Psi$
- D2TUC with decentralized configuration $\Phi$

as put forward in [[1]](#references), and also for two centralized strategies as a baseline:
{: .text-justify}
- TUC, as detailed in [[2]](#references)
- QPC, which is a centralized version of D2TUC, inspired in [[2]](#references)

~~~m
%% Controller gain synthesis
% Compute LQR weight matrices as detailled in [1] for DTUC
Q_DTUC = [eye(r) zeros(r,Z-r)]*W'*diag(1./model.capacity)*...
    W*[eye(r) ;zeros(Z-r,r)];
R_DTUC = 0.0001*eye(model.stg);

% Compute LQR weight matrices as detailled in [1] for D2TUC
Q_D2TUC = diag(1./model.capacity);
R_D2TUC = 0.0001*eye(model.L);

% Centralized gain computation for DTUC
[K_TUC,P_TUC] = LQROneStepLTI_augmented(A1_hat,Bg1_hat,Q_DTUC,R_DTUC,ones(size(model.E_DTUC_phi)),1e3,1e-5,model.A,model.Bg,r,Z,W);    
% One-step gain computation for DTUC with configuration psi
[K_DTUC_psi,P_DTUC_psi] = LQROneStepLTI_augmented(A1_hat,Bg1_hat,Q_DTUC,R_DTUC,model.E_DTUC_psi,1e3,1e-5,model.A,model.Bg,r,Z,W);
% One-step gain computation for DTUC with configuration phi
[K_DTUC_phi,P_DTUC_phi] = LQROneStepLTI_augmented(A1_hat,Bg1_hat,Q_DTUC,R_DTUC,model.E_DTUC_phi,1e3,1e-5,model.A,model.Bg,r,Z,W);
% Centralized gain computation for D2TUC with configuration phi
[K_D2TUC_C,P_D2TUC_C] = LQRCentralizedLTI(model.A,model.BG,Q_D2TUC,R_D2TUC);
% One-step gain computation for D2TUC with configuration psi
[K_D2TUC_psi,P_D2TUC_psi] = LQROneStepLTI(model.A,model.BG,Q_D2TUC,R_D2TUC,model.E_D2TUC_psi);
% One-step gain computation for D2TUC with configuration phi
[K_D2TUC_phi,P_D2TUC_phi] = LQROneStepLTI(model.A,model.BG,Q_D2TUC,R_D2TUC,model.E_D2TUC_phi);
~~~

The simulation of the network is carried out with a **nonlinear** mascroscopic model with **upstream gating**
{: .text-justify}
~~~m
%% Nonlinear simulation
controlStrat = 6; % Number of control strategies to simulate
nDisc = 10; % Number of discrete time steps to simulate
tspan = (0:model.T:nDisc*model.C); % (s)
xNL = cell(controlStrat,1);
xDisc = cell(controlStrat,1);
dNL = cell(controlStrat,1);
gNL = cell(controlStrat,1);
uNL = cell(controlStrat,1);

% Variable initialization
for m = 1:controlStrat
    xNL{m,1} = zeros(model.L,length(tspan));
    dNL{m,1} = zeros(model.L,length(tspan)-1);
    gNL{m,1} = zeros(model.stg,length(0:model.C:tspan(end))-1);
    uNL{m,1} = zeros(model.stg,length(tspan(end))-1);
end

% Simulate each control strategy
for  m = 1:controlStrat
    xNL{m,1}(:,1) = model.x0;
    for k = 1:length(tspan)-1
        % Control update frequency is T/C times slower if
        if ~rem(int16(k-1),int16(model.C/model.T))
            if k ~= 1
               xDisc{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1) = xNL{m,1}(:,k);
            else
                xDisc{m,1}(:,1) = model.x0;
            end
            xD = xDisc{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1);
            switch m
            case 1 % TUC
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1) =...
                    LQcontrolAction(xD,K_TUC,model,gN_DTUC);
            case 2 % DTUC with configuration psi     
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1) =...
                    LQcontrolAction(xD,K_DTUC_psi,model,gN_DTUC);
            case 3 % DTUC with configuration phi                 
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1) =...
                    LQcontrolAction(xD,K_DTUC_phi,model,gN_DTUC);
            case 4 % TUC - QPC (centralized version of D2TUC) inspired in [2]
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1) =...
                    QPCcontrolAction(xD,K_D2TUC_C,model,model.stageMatrix,gN_D2TUC);
            case 5 % DTUC with configuration phi  
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1) =...
                    QPCcontrolAction(xD,K_D2TUC_psi,model,model.stageMatrix,gN_D2TUC);
            case 6 % DTUC with configuration phi
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1) =...
                    QPCcontrolAction(xD,K_D2TUC_phi,model,model.stageMatrix,gN_D2TUC);
            end
        end

        % Compute nonlinear control action with upstream gating
        for l = 1:model.L % Equation (14) of [2]
            if sum(xNL{m,1}(model.turningRatesTable(l,1:end-1)~=0,k) >=...
                   model.jam*model.capacity(model.turningRatesTable(l,1:end-1)~=0))
               uNL{m,1}(l,k) = 0;
            else
               uNL{m,1}(l,k) = min(xNL{m,1}(l,k)/model.T,model.stageMatrix(l,:)*...
                   gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1)*...
                model.saturation(l)/model.C);
            end
        end
        dNL{m,1}(:,k) = min((model.capacity-...
            xNL{m,1}(:,k)-model.Bu_sim*uNL{m,1}(:,k))/model.T,model.d);
        xNL{m,1}(:,k+1) = xNL{m,1}(:,k)+model.Bu_sim*uNL{m,1}(:,k)+model.T*model.d;

        % Catch overspill
        if sum(xNL{m,1}(:,k+1)./model.capacity>1) ~=0
           fprintf("Overspill: method %d | instant k=%d | link %d \n", m,k+1,find(xNL{m,1}(:,k+1)./model.capacity>1));
           break;
        end

    end
end
~~~

Two objective functions, proposed in [[2]](#references) are used to evaluate the performance of the proposed decentralized approaches: **the total time spent (TTS)** and the **relative queue balance (RQB)**
{: .text-justify}
~~~m
%% Compute performance indices defined in [1,2]
TTS = zeros(controlStrat,1);
RBQ = zeros(controlStrat,1);
xNL_crit = cell(controlStrat,1);
for  m = 1:controlStrat
    xNL_crit{m,1}(:,1) = model.x0;
    for k = 2:length(tspan)-1
        % Control update frequency is T/C times slower if
        if ~rem(int16(k-1),int16(model.C/model.T))
            xNL_crit{m,1}(:,idivide(int16(k-1),int16(model.C/model.T))+1) = ...
                 mean(xNL{m,1}(:,k-(model.C/model.T-1):k),2);
        end
    end
end
for m = 1:controlStrat % variable initialization
    TTS(m) = model.C*(1/3600)*sum(sum(xNL_crit{m,1}(:,2:end)));
    RBQ(m) = sum(sum(xNL_crit{m,1}(:,2:end).^2,2)./model.capacity);
end
~~~

In addition, the prototypes of several other functions used, which are included in the full MATLAB code, are presented below
{: .text-justify}
~~~m
%% modelSynthesis - Description
% This function synthesizes the model of a traffic network from the raw
% data provide in text files.
% Input:    - folder: path to directory of text files
%               - 'general.txt': general parameters
%               - 'junctions_table.txt': caracteristics of each junction
%               - 'links_table.txt': characteristics of each link
%               - 'stage_matrix.txt': stage matrix
%               - 'turning_rates_table.txt': turning rates matrix
% (For templates of the text files check "template_*.txt")
% Output:   - model: struct of variables that characterize the network
function model =  modelSynthesis(folder)

%% LQcontrolAction - Description
% This function implements a standard linear quadratic feedback control
% action for the raffic network model
% Input:    - x: state
%           - L_LQ: LQ gain matrix
%           - model: struct of variables that characterize the network
%           - gN: historic green times
% Output:   - g: green times for each stage
function g = LQcontrolAction(x,L_LQ,model,gN)

%% QPCcontrolAction - Description
% This function implements a linear quadratic rogramming feedback control
% action for the raffic network model
% Inspired in the nonlinear QPC control action proposed in [2]
% Input:    - x: state
%           - L_QPC: QPC linear gain matrix
%           - model: struct of variables that characterize the network
%           - ROW: stage matrix S
%           - gNQPC: historic green times
% Output:   - g: green times for each stage
function g = QPCcontrolAction(x,L_QPC,model,ROW,gNQPC)

%% knapsack - Description
% This function solves the knapsack problem
% Implementation of algorithm in [3]
% Input:    - a,b,c,d as defined in [1]
% Output:   - x: knapsack solution
% [3] Helgason, R., Kennington, J., Lall, H., 1980. A polynomially bounded
% algorithm for a singly constrained quadratic program. Math. Program.
% 18 (1), 338-343.
function x = knapsack(a,b,c,d)

%% LQROneStepLTI_augmented - Description
% This function computes the steady-state augmented one-step LQR regulator
% gain for a window w. Method derived in [1].
% Input:    - A_hat, B_hat
%           - Q, R
%           - E: sparsity pattern
%           - itMax: maximum number of iterations until convergence
%           - epslInf: minimum relative improvement
%           - A, B
%           - r,Z,W: as defined in [1]
% Output:   - K: nxo steady-state gain matrix
%           - P: nxn steady-state estimation error covariance matrix
% Important notes:
%           - output gain corresponds to the control law: u(k)=-K(k)*x(k)
% WARNING: Returns Kinf = NaN and Pinf = NaN if convergence could not be reached
function [K,P] = LQROneStepLTI_augmented(A_hat,B_hat,Q,R,E,itMax,epslInf,A,B,r,Z,W)
~~~

The evolution of the controllable (left) and uncontrollable (right) components are shown below
{: .text-justify}
<div class="row">
  <div class="column">
    <img src="/assets/img/TrafficNetworkOS_controllable.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/TrafficNetworkOS_uncontrollable.svg" style="width:100%">
  </div>
</div>

The evolution of relative occupancy of the links and of the green times of the stages for DTUC with decentralized configuration $\Phi$ are shown below
{: .text-justify}
<div class="row">
  <div class="column">
    <img src="/assets/img/TrafficNetworkOS_occ.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/TrafficNetworkOS_stage_time.svg" style="width:100%">
  </div>
</div>

{: .text-justify}


# References
[1] {% for paper in site.data.references%}{% if paper.key == "Pedroso2021Traffic" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}

[2] <a href="http://refhub.elsevier.com/S0968-090X(21)00406-X/sb1" target="_blank">Aboudolas, K., Papageorgiou, M., Kosmatopoulos, E., 2009. Store-and-forward based methods for the signal control problem in large-scale congested urban road networks. Transp. Res. C 17 (2), 163–174.</a>
{: .text-justify}
