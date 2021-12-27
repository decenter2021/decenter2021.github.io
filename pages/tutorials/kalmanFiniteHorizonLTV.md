---
layout: single
title: Finite-horizon Kalman filter tutorial for LTV systems
showTitle: true
excerpt: "Tutorial on decentralized Kalman filter synthesis using the finite-horizon method."
permalink: /tutorials/kalmanFiniteHorizonLTV/
classes: wide
sidebar:
  title: "Tutorials"
  nav: sidebar-tutorials
header:
  teaser: "/assets/img/kalmanFiniteHorizonLTVTutorial.svg"
tags:
  - tutorial
  - ltv
  - finite-horizon
  - estimation
date: "2021-07-09"
last_modified_at: "2021-12-27"
---
{{page.excerpt}}\\
See [documentation for kalmanFiniteHorizonLTV](/documentation/kalmanFiniteHorizonLTV/) for more information.
{: .text-justify}

***

To open this tutorial execute the following command in the MATLAB command window
~~~m
open kalmanFiniteHorizonLTVTutorial
~~~

Use randomly generated synthetic system matrices $\mathbf{A}(k)$, $\mathbf{C}(k)$, $\mathbf{Q}(k)$, $\mathbf{R}(k)$, and $\mathbf{E}$
~~~m
T = 50;
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

Synthesize Kalman filter gain using the finite-horizon algorithm for the whole window
~~~m
% Generate random initial predicted covariance for the initial time instant
P0 = rand(n,n);
P0 = 100*(P0*P0');
% Algorithm paramenters (optional)
opts.verbose = true;
opts.epsl = 1e-5;
opts.maxOLIt = 100;
% Synthesize regulator gain using the finite-horizon method
[K,P] = kalmanFiniteHorizonLTV(system,E,T,P0,opts);
~~~
~~~text
----------------------------------------------------------------------------------
Running finite-horizon algorithm with:
epsl = 1e-05 | maxOLIt = 100.
Convergence reached with: epsl = 1e-05
A total of 10 outer loop iterations were run.
----------------------------------------------------------------------------------
~~~
Simulate the error dynamics for the synthetic system
~~~m
% Initialise error cell
x = cell(T,1);
% Generate random initial error
x0 = transpose(mvnrnd(zeros(n,1),P0));
% Init predicted covariance matrix
Ppred = A0*P0*A0'+Q0;
for j = 1:T
    % Error dynamics
    if j == 1
        x{j,1} = (eye(n)-K{j,1}*system{j,2})*(A0*x0+...
                mvnrnd(zeros(n,1),Q0)')-K{j,1}*mvnrnd(zeros(o,1),system{j,4})';
    else
        x{j,1} = (eye(n)-K{j,1}*system{j,2})*(system{j-1,1}*x{j-1,1}+...
                mvnrnd(zeros(n,1),system{j-1,3})')-K{j,1}*mvnrnd(zeros(o,1),system{j,4})';
    end
end
~~~
Plot the norm of the estimation error
~~~m
% Plot the ||x||_2 vs instant of the simulation
figure;
hold on;
set(gca,'FontSize',35);
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
xPlot = zeros(T,1);
for j = 1:T
   xPlot(j,1) =norm(x{j,1}(:,1));
end
plot(1:T, xPlot(:,1),'LineWidth',3);
set(gcf, 'Position', [100 100 900 550]);
ylabel('$\|\mathbf{x}_{FH}(k)\|_2$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;
~~~
{:refdef: style="text-align: center;"}
![image-title-here](/assets/img/kalmanFiniteHorizonLTVTutorial.svg){:class="img-responsive"}
{: refdef}
