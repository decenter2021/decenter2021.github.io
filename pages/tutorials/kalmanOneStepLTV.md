---
layout: single
title: One-step Kalman filter tutorial for LTV systems
showTitle: true
excerpt: "Tutorial on decentralized Kalman filter synthesis using the one-step method."
permalink: /tutorials/kalmanOneStepLTV/
classes: wide
sidebar:
  title: "Tutorials"
  nav: sidebar-tutorials
header:
  teaser: "/assets/img/kalmanOneStepLTVTutorial.svg"
tags:
  - tutorial
  - ltv
  - one-step
  - estimation
date: "2021-07-10"
last_modified_at: "2021-07-10"
---
{{page.excerpt}}\\
See [documentation for kalmanOneStepLTV](/documentation/kalmanOneStepLTV/) for more information.

***

To open this tutorial execute the following command in the MATLAB command window
~~~m
open kalmanOneStepLTVTutorial
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

Simulate error dynamics ans synthesize distributed Kalman filter gain using the one-step method each time-step
~~~m
% Generate random initial predicted covariance for the initial time instant
P0 = rand(n,n);
P0 = P0*P0';
% Initialise error cell
x = cell(T,1);
% Generate random initial error
x0 = transpose(mvnrnd(zeros(n,1),P0));
% Init predicted covariance matrix
Ppred = A0*P0*A0'+Q0;
for j = 1:T
    % Synthesize regulator gain using the one-step method
    [K,Ppred,~] = kalmanOneStepLTV(system(j,:),E,Ppred);
    % Error dynamics
    if j == 1
        x{j,1} = (system{j,1}-K*system{j,2})*x0;
    else
        x{j,1} = (system{j,1}-K*system{j,2})*x{j-1,1};
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
ylabel('$\|\mathbf{x}_{OS}(k)\|_2$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;
~~~
{:refdef: style="text-align: center;"}
![image-title-here](/assets/img/kalmanOneStepLTVTutorial.svg){:class="img-responsive"}
{: refdef}
