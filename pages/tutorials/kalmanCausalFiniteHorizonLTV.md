---
layout: single
title: Causal finite-horizon Kalman filter tutorial for LTV systems
showTitle: true
excerpt: "Tutorial on decentralized Kalman filter synthesis using the causal finite-horizon method."
permalink: /tutorials/kalmanCausalFiniteHorizonLTV/
classes: wide
sidebar:
  title: "Tutorials"
  nav: sidebar-tutorials
header:
  teaser: "/assets/img/kalmanCausalFiniteHorizonTutorial_net_teaser.svg"
tags:
  - tutorial
  - ltv
  - causal-finite-horizon
  - finite-horizon
  - estimation
date: "2021-12-28"
last_modified_at: "2021-12-28"
---
{{page.excerpt}}\\
See [documentation for kalmanCausalFiniteHorizonLTV](/documentation/kalmanCausalFiniteHorizonLTV/) for more information.
{: .text-justify}

***

To open this tutorial execute the following command in the MATLAB command window
{: .text-justify}
~~~m
open kalmanCausalFiniteHorizonLTVTutorial
~~~

Generate random synthetic system matrices $\mathbf{A}(k)$, $\mathbf{C}(k)$, $\mathbf{Q}(k)$, $\mathbf{R}(k)$, and $\mathbf{E}$ with
{: .text-justify}
~~~m
%% Synthetic random system
n = 5;
o = 3;
rng(1); % Pseudo-random seed for consistency
% Alternatively comment out rng() to generate a random system
% Do not forget to readjust the synthesys parameters of the methods
system = cell(T,4);
% Initial matrices (just to compute the predicted covariance at k = 1)
A0 = rand(n,n)-0.5;
Q0 = rand(n,n)-0.5;
Q0 = Q0*Q0';
for i = 1:T
    if i == 1
        system{i,1} = A0+(1/10)*(rand(n,n)-0.5);
        system{i,2} = rand(o,n)-0.5;
    else % Generate time-varying dynamics preventing erratic behaviour
        system{i,1} = system{i-1,1}+(1/10)*(rand(n,n)-0.5);
        system{i,2} = system{i-1,2}+(1/10)*(rand(o,n)-0.5);
    end
    system{i,3} = rand(n,n)-0.5;
    system{i,3} = system{i,3}*system{i,3}';
    system{i,4} = rand(o,o)-0.5;
    system{i,4} = system{i,4}*system{i,4}';
end
E = round(rand(n,o));
~~~
or, alternatively, generate a network of randomly generated agents with measurement couplings
{: .text-justify}
~~~m
%% Network of randomly generated agents
% Graph topology generation parameters
seed = 3; % random seed
rng(seed);
N = 10; % number of agents
max_neighborhood = 2;
proximity_neighborhood = 5;
edges = [];
% Define interconnetions marix
couplings = zeros(N,N);
% Generate interconnections for each agent
for i = 1:N        
    lcur = [];
    lmax =  round(exp(rand()*(log(max_neighborhood))));
    l = 0;
    couplings(i,i) = 1;
    while true
        if l == lmax
            break;
        end
        l = l+1;
        j = round(rand()*(N-1))+1;
        if j ~= i
            if ~sum(lcur == j) && abs(j-i) <= proximity_neighborhood
                couplings(i,j) = 1;
                edges = [edges [j;i]];
                lcur = [lcur j];
            else
                l =l-1;
                continue;
            end
        else
            l = l-1;
            continue;
        end
    end
end
% Plot diagraph
G = digraph();
for e = 1:size(edges,2)
    G = addedge(G,edges(1,e),edges(2,e));
end
figure;
hold on;
p = plot(G,'Layout','force','UseGravity',true);
p.Marker = 's';
p.NodeColor = 'r';
p.MarkerSize = 7;
axis off
hold off;

% Generate global dynamics
n = 4;
o = 2;
q_rel = 0.1;
r_rel = 0.1;
A = cell(T,1);
C = cell(T,1);
Q = cell(T,1);
R = cell(T,1);
A0_aux = eye(n)+0.1*(rand(n,n)-0.5);
Q0_aux = 0.01*eye(n);%+0.1*(rand(n,n)-0.5);
A0 = zeros(N*n);
Q0 = zeros(N*n);
for k = 1:T
    A{k,1} = zeros(N*n);
    C{k,1} = zeros(o*N,n*N);
    Q{k,1} = zeros(N*n);
    R{k,1} = zeros(o*N,o*N);
    for i = 1:N    
        % Principal diagonal A,Q
        A0((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n) = A0_aux;
        Q0((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n) = Q0_aux;
        if k == 1   
            A{k,1}((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n) = A0_aux+0.05*(rand(n,n)-0.5);
            Q{k,1}((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n) = real(sqrtm(Q0_aux+0.01*(rand(n,n)-0.5)));  
            R{k,1}((i-1)*o+1:(i-1)*o+o,(i-1)*o+1:(i-1)*o+o) = 0.1*eye(o);%0.1*(rand(o,o)-0.5);     
        else
            A{k,1}((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n) = A{k-1,1}((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n)+0.05*(rand(n,n)-0.5);
            Q{k,1}((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n) = real(sqrtm(Q{k-1,1}((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n)+0.01*(rand(n,n)-0.5)));
            R{k,1}((i-1)*o+1:(i-1)*o+o,(i-1)*o+1:(i-1)*o+o) = real(sqrtm(R{k-1,1}((i-1)*o+1:(i-1)*o+o,(i-1)*o+1:(i-1)*o+o)+0.01*(rand(o,o)-0.5)));
        end
        R{k,1}((i-1)*o+1:(i-1)*o+o,(i-1)*o+1:(i-1)*o+o) = R{k,1}((i-1)*o+1:(i-1)*o+o,(i-1)*o+1:(i-1)*o+o)*R{k,1}((i-1)*o+1:(i-1)*o+o,(i-1)*o+1:(i-1)*o+o)';
        Q{k,1}((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n) = Q{k,1}((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n)*Q{k,1}((i-1)*n+1:(i-1)*n+n,(i-1)*n+1:(i-1)*n+n)';        
        for j = 1:N
            if couplings(i,j) == 0
                continue;
            end
            if k == 1
                C{k,1}((i-1)*o+1:(i-1)*o+o,(j-1)*n+1:(j-1)*n+n) = 2*(rand(o,n)-0.5);            
            else
                C{k,1}((i-1)*o+1:(i-1)*o+o,(j-1)*n+1:(j-1)*n+n) = C{k-1,1}((i-1)*o+1:(i-1)*o+o,(j-1)*n+1:(j-1)*n+n)+0.1*(rand(o,n)-0.5);      
            end
        end
    end
end

% Build global system matrices
system = cell(T,4);
for i = 1:T
   system{i,1} = A{i,1};
   system{i,2} = C{i,1};
   system{i,3} = Q{i,1};
   system{i,4} = R{i,1};
end
% Sparsity pattern for fully distributed configuration
E = zeros(n*N,o*N);
for i = 1:N
    E((i-1)*n+1:(i-1)*n+n,(i-1)*o+1:(i-1)*o+o) = ones(n,o);
end
% Dimensions of the global system
n = n*N;
o = o*N;
~~~
which can be represented a directed graph, whose edges represent measurement couplings
{: .text-justify}
{:refdef: style="text-align: center;max-width: 100%; height: auto;"}
![image-title-here](/assets/img/kalmanCausalFiniteHorizonTutorial_net.svg){:class="img-responsive"}
{: refdef}

Then, synthesize the Kalman filter causal finite-horizon gains online and simulate the estimation error dynamics with
{: .text-justify}
~~~m
%% Simulate error dynamics
% Initial estimation erro covariance matrix
P0 = 100*eye(n);
% Parameters of CFH method
W = 4;
opts.verbose = true;
opts.epsl = 1e-4;
opts.alpha = 0.1;
opts.maxOLIt = 1e4;
% Initialise error cell
x = cell(T,1);
P = cell(T,1);
% Generate random initial error
x0 = transpose(mvnrnd(zeros(n,1),P0));
% Initialize vectors to hold the process and sensor noise
% (for the simulation of estimation error dynamics)
w0_noise = mvnrnd(zeros(n,1),Q0)';
w_noise = cell(T,1); % process noise (the T-th entry is unused)
v_noise = cell(T,1); % sensor noise

for k = 1:T
    %%%%% Gain computation
    CFH_tau = max([1 k-W+1]); % tau in CFH algorithm [1]
    CFH_W = min([k,W]); % CFH window length algorithm [1]

    if CFH_tau == 1 % if the initial instant of the current window is the first
        CFH_Ppred0 = A0*P0*A0'+Q0; % Covariance prediction
        x_aux = x0; % Get estimate at the beggining of the window
    else
        CFH_Ppred0 = system{CFH_tau-1,1}*P{CFH_tau-1,1}*system{CFH_tau-1,1}'+...
                        system{CFH_tau-1,3}; % Covariance prediction
        x_aux = x{CFH_tau-1,1}; % Get estimate at the beggining of the window
    end
    % Compute the Finite Horizon gains for the window of past instants
    [CFH_K,CFH_P] = kalmanCausalFiniteHorizonLTV(system(CFH_tau:k,:),E,CFH_W,CFH_Ppred0,opts);
    % Covariance at the end of the CFH window is the  becomes the final
    % covariance for instant k
    P{k,1} = CFH_P{end,1};

    %%%%% Estimation error dynamics
    % Random process and sensor Gaussian noise
    w_noise{k,1} = mvnrnd(zeros(n,1),system{k,3})';
    v_noise{k,1} = mvnrnd(zeros(o,1),system{k,4})';
    % Simulate the error dynamics for the window
    for j = CFH_tau:k
        if CFH_tau == 1 && j == 1
           x_aux = (eye(n)-CFH_K{j-CFH_tau+1,1}*system{j,2})*(A0*x_aux+...
                    w0_noise)-CFH_K{j-CFH_tau+1,1}*v_noise{j,1};
        else
           x_aux = (eye(n)-CFH_K{j-CFH_tau+1,1}*system{j,2})*(system{j-1,1}*x_aux+...
                    w_noise{j-1,1})-CFH_K{j-CFH_tau+1,1}*v_noise{j,1};
        end
    end
    % Estimate at the end of the CFH window becomes the final estimate for
    % instant k
    x{k,1} = x_aux;
end
~~~

Plot the trace of the estimation error covariance matrix
{:refdef: style="text-align: center;width: 75%;"}
![image-title-here](/assets/img/kalmanCausalFiniteHorizonTutorial_cov.svg){:class="img-responsive"}
{: refdef}
and the norm of the estimation error
{:refdef: style="text-align: center;width: 75%;"}
![image-title-here](/assets/img/kalmanCausalFiniteHorizonTutorial_error.svg){:class="img-responsive"}
{: refdef}

# References
[1] *Not published yet*

[2] {% for paper in site.data.references%}{% if paper.key == "Pedroso2021KalmanOSFHLTV" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}
