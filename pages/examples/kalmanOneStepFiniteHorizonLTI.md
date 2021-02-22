---
layout: single
title: "One-step and finite-horizon: comparison for LTI estimation"
showTitle: true
excerpt: "Performance comparison between OS and FH Kalman filters using Monte-Carlo simulations."  
permalink: /examples/kalmanOneStepFiniteHorizonLTI/
classes: wide
sidebar:
  title: "Examples"
  nav: sidebar-examples
header:
  teaser: "/assets/img/kalmanOneStepFiniteHorizonLTIComparison.svg"
tags:
  - example
  - LTI
  - one-step
  - finite-horizon
  - estimation
date: "2021-02-22"
last_modified_at: "2021-02-22"
---
{{page.excerpt}}\\
See [documentation for kalmanOneStepLTI](/documentation/kalmanOneStepLTI/) and [documentation for kalmanFiniteHorizonLTI](/documentation/kalmanFiniteHorizonLTI/) for more information.

***
To open this tutorial execute the following command in the MATLAB command window
~~~m
open kalmanOneStepFiniteHorizonLTIComparison
~~~


Use synthetic system matrices $\mathbf{A}$, $\mathbf{B}$, $\mathbf{Q}$, $\mathbf{R}$, and $\mathbf{E}$ generated randomly
~~~m
n = 10;
o = 6;
rng(1); % Pseudo-random seed for consistency
% Alternatively comment out rng() to generate a random system
% Do not forget to readjust the synthesys parameters of the methods
A = rand(n,n)
C = rand(o,n)
Q = rand(n,n);
Q = Q*Q'
R = rand(o,o);
R = R*R'
E = round(rand(n,o))
~~~

~~~text
A =
    0.4170    0.4192    0.8007    0.0983    0.9889    0.0194    0.1023    0.9034    0.8833    0.1147
    0.7203    0.6852    0.9683    0.4211    0.7482    0.6788    0.4141    0.1375    0.6237    0.9495
    0.0001    0.2045    0.3134    0.9579    0.2804    0.2116    0.6944    0.1393    0.7509    0.4499
    0.3023    0.8781    0.6923    0.5332    0.7893    0.2655    0.4142    0.8074    0.3489    0.5784
    0.1468    0.0274    0.8764    0.6919    0.1032    0.4916    0.0500    0.3977    0.2699    0.4081
    0.0923    0.6705    0.8946    0.3155    0.4479    0.0534    0.5359    0.1654    0.8959    0.2370
    0.1863    0.4173    0.0850    0.6865    0.9086    0.5741    0.6638    0.9275    0.4281    0.9034
    0.3456    0.5587    0.0391    0.8346    0.2936    0.1467    0.5149    0.3478    0.9648    0.5737
    0.3968    0.1404    0.1698    0.0183    0.2878    0.5893    0.9446    0.7508    0.6634    0.0029
    0.5388    0.1981    0.8781    0.7501    0.1300    0.6998    0.5866    0.7260    0.6217    0.6171
C =
    0.3266    0.0158    0.9326    0.7115    0.8600    0.5858    0.8071    0.0599    0.5597    0.2523
    0.5271    0.9294    0.6968    0.1243    0.5388    0.9696    0.3879    0.1213    0.0126    0.7438
    0.8859    0.6909    0.0660    0.0199    0.5528    0.5610    0.8635    0.0446    0.0720    0.1954
    0.3573    0.9973    0.7555    0.0262    0.8420    0.0186    0.7471    0.1075    0.9673    0.5814
    0.9085    0.1723    0.7539    0.0283    0.1242    0.8006    0.5562    0.2257    0.5681    0.9700
    0.6234    0.1371    0.9230    0.2462    0.2792    0.2330    0.1365    0.7130    0.2033    0.8468
Q =
    2.0248    2.0828    2.5116    1.5566    1.5742    2.2890    1.0643    2.1221    2.4284    2.7857
    2.0828    4.6675    3.1268    2.8692    2.7604    3.7298    1.9061    3.4757    3.2309    3.5648
    2.5116    3.1268    3.8267    2.7687    2.7802    3.5348    1.4805    2.8452    3.2115    4.3199
    1.5566    2.8692    2.7687    3.5790    2.4780    3.2995    1.7717    2.7696    2.9485    3.7020
    1.5742    2.7604    2.7802    2.4780    2.7361    3.3282    1.2796    2.2486    2.5126    3.5423
    2.2890    3.7298    3.5348    3.2995    3.3282    4.8683    2.0391    3.6455    3.6324    4.7527
    1.0643    1.9061    1.4805    1.7717    1.2796    2.0391    1.5282    1.5951    1.4736    1.9614
    2.1221    3.4757    2.8452    2.7696    2.2486    3.6455    1.5951    3.6668    3.5500    3.7370
    2.4284    3.2309    3.2115    2.9485    2.5126    3.6324    1.4736    3.5500    4.0140    4.2097
    2.7857    3.5648    4.3199    3.7020    3.5423    4.7527    1.9614    3.7370    4.2097    5.7205
R =
    1.8502    2.1035    2.1588    1.9751    1.5911    1.0335
    2.1035    3.0864    2.8309    2.8417    2.2906    1.4985
    2.1588    2.8309    3.0409    2.8277    2.0798    1.4439
    1.9751    2.8417    2.8277    2.7927    2.1208    1.5089
    1.5911    2.2906    2.0798    2.1208    1.8323    1.1133
    1.0335    1.4985    1.4439    1.5089    1.1133    1.0021
E =
     0     0     1     0     0     1
     0     0     0     1     1     1
     1     1     0     1     1     1
     0     0     1     1     1     0
     1     1     0     0     1     1
     1     1     1     1     1     1
     1     0     0     0     1     0
     1     0     0     1     1     0
     0     0     1     1     0     1
     0     0     0     0     1     0
~~~

Syntesize centralized, one-step, and finite-horizon filter gains
~~~m
[KC,PC] = kalmanCentralizedLTI(A,C,Q,R);
[KOS,POS] = kalmanOneStepLTI(A,C,Q,R,E);
opts.W = 50;
opts.maxOLIt = 10;
[KFH,PFH] = kalmanFiniteHorizonLTI(A,C,Q,R,E,opts);
~~~

Run Monte Carlo simulations and compute covariance matrices
~~~m
% Generate initial estimation error covariance
P0 = rand(n,n);
P0 = P0*P0';
% Set number of Monte Carlo simulations
NMCSim = 1e3;
% Set simulation span of each Monte Carlo simulation
SimIt = 100;
% Initialise error cell, where the first dimension indicates the number of
% the Monte Carlo simulation; the second the instant of the simulation; and
% the third the method used
error = cell(NMCSim,SimIt,3);
% This loops can, alternatively, be performed in parallel using parfor
for method = 1:3    % Iterate through the methods
    switch method   % Select gain
        case 1
            K = KC;
        case 2
            K = KOS;
        case 3
            K = KFH;
    end
    for i = 1:NMCSim    % Iterate through the Monte Carlo simulations
        error{i,1,method} = transpose(mvnrnd(zeros(n,1),P0));
        for k = 2:SimIt
            error{i,k,method} = (eye(n)-K*C)*(A*error{i,k-1,method}+...
                        transpose(mvnrnd(zeros(n,1),Q)))-...
                        K*transpose(mvnrnd(zeros(o,1),R));
        end
    end
end
% Initialise Monte Carlo covariance cell, where the rows contain the data
% of each method and the columns indicate the instant in the simulation
P_MC = cell(3,SimIt);
% Iterate though the methods
for method = 1:3
    for k = 1:SimIt
        % Compute covariance matrix
        P_MC{method,k} = cov(cell2mat(error(:,k,method)')');
    end
end
% Compute the trace of each covariance matrix
traceEvolution = zeros(SimIt,3);
for method = 1:3
    for k = 1:SimIt
        traceEvolution(method,k) = trace(P_MC{method,k});
    end
end
~~~
Plot results and compare them with the projected steady-state performance (dashed lines)
~~~m
% Plot the Trace vs instant of the simulation for the Monte Carlo
% simulation results
figure;
hold on;
set(gca,'FontSize',35);
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
p = zeros(3,1);
for method = 1:3
    p(method) = plot(0:SimIt-1,traceEvolution(method,:),'LineWidth',3);
end
% Plot dashed projected performance
plot(0:SimIt-1,trace(PC)*ones(1,SimIt),'--','LineWidth',2,'Color',get(p(1),'Color'));
plot(0:SimIt-1,trace(POS)*ones(1,SimIt),'--','LineWidth',2,'Color',get(p(2),'Color'));
plot(0:SimIt-1,trace(PFH)*ones(1,SimIt),'--','LineWidth',2,'Color',get(p(3),'Color'));
set(gcf, 'Position', [100 100 900 550]);
ylabel('$\mathrm{tr}(\mathbf{P}_{MC}(k))$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');
legend('Cent.', 'OS', 'FH','Location','Best');
hold off;
~~~
{:refdef: style="text-align: center;"}
![image-title-here](/assets/img/kalmanOneStepFiniteHorizonLTIComparison.svg){:class="img-responsive"}
{: refdef}
Note that the results obtained with Monte-Carlo simulations are very close to the projected results. It is also important to remark the significant improve in performance obtained with the finite-horizon method in relation to the one-step method.
{: .text-justify}
