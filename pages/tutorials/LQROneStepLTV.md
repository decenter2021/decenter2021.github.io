---
layout: single
title: One-step regulator synthesis tutorial for LTV systems
showTitle: true
excerpt: "Tutorial on decentralized regulator synthesis using the one-step method."
permalink: /tutorials/LQROneStepLTV/
classes: wide
sidebar:
  title: "Tutorials"
  nav: sidebar-tutorials
header:
  teaser: "/assets/img/LQROneStepLTVTutorial.svg"
tags:
  - control
  - ltv
  - one-step
  - tutorial
date: "2021-02-23"
last_modified_at: "2021-02-23"
---
{{page.excerpt}}\\
See [documentation for LQROneStepLTV](/documentation/LQROneStepLTV/) for more information.

***

To open this tutorial execute the following command in the MATLAB command window
~~~m
open LQROneStepLTVTutorial
~~~

Generate synthetic time-varying system matrices $\mathbf{A}(k)$, $\mathbf{B}(k)$, $\mathbf{Q}(k)$, $\mathbf{R}(k)$, and time-invariant sparsity pattern $\mathbf{E}$
~~~m
T = 50;
n = 5;
m = 3;
rng(1); % Pseudo-random seed for consistency
% Alternatively comment out rng() to generate a random system
% Do not forget to readjust the synthesys parameters of the methods
system = cell(T+1,4);
for i = 1:T+1
    if i == 1
        system{i,1} = rand(n,n)-0.5;
        system{i,2} = rand(n,m)-0.5;
    elseif i == T+1
        system{i,1} = nan;
        system{i,2} = nan;
        system{i,3} = rand(n,n)-0.5;
        system{i,3} = system{i,3}*system{i,3}';
        system{i,4} = nan;
        continue;
    else % Generate time-varying dynamics preventing erratic behaviour
        system{i,1} = system{i-1,1}+(1/4)*(rand(n,n)-0.5);
        system{i,2} = system{i-1,2}+(1/4)*(rand(n,m)-0.5);
    end
    system{i,3} = rand(n,n)-0.5;
    system{i,3} = system{i,3}*system{i,3}';
    system{i,4} = rand(m,m)-0.5;
    system{i,4} = system{i,4}*system{i,4}';
end
E = round(rand(m,n));
~~~

Synthesize regulator gain using the one-step method (with some optional parameters)
~~~m
opts.verbose = true;
[K,P] = LQROneStepLTV(system,E,T,opts);
~~~
~~~text
----------------------------------------------------------------------------------
Running one-step algorithm with T = 50.
----------------------------------------------------------------------------------
~~~
Simulate the dynamics of the synthetic system
~~~m
% Generate random initial covariance for the initial time instant
% (LQROneStepLTV does not make assumptions on P0)
P0 = rand(n,n);
P0 = P0*P0';
% Initialise error cell
x = cell(T+1,1);
% Generate random initial error
x{1,1} = transpose(mvnrnd(zeros(n,1),P0));
for j = 2:T+1
    x{j,1} = (system{j-1,1}-system{j-1,2}*K{j-1,1})*x{j-1,1};
end
~~~
Plot the norm of the estimation error
~~~m
figure;
hold on;
set(gca,'FontSize',35);
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
xPlot = zeros(T+1,1);
for j = 1:T+1
   xPlot(j,1) =norm(x{j,1}(:,1));
end
plot(0:T, xPlot(:,1),'LineWidth',3);
set(gcf, 'Position', [100 100 900 550]);
ylabel('$\|\mathbf{x}_{OS}(k)\|_2$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;
~~~
{:refdef: style="text-align: center;"}
![image-title-here](/assets/img/LQROneStepLTVTutorial.svg){:class="img-responsive"}
{: refdef}
