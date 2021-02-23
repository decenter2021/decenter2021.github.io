---
layout: single
title: One-step regulator synthesis tutorial for LTI systems
showTitle: true
excerpt: "Tutorial on decentralized regulator synthesis using the one-step method."
permalink: /tutorials/LQROneStepLTI/
classes: wide
sidebar:
  title: "Tutorials"
  nav: sidebar-tutorials
header:
  teaser: "/assets/img/LQROneStepLTITutorial.svg"
tags:
  - control
  - lti
  - one-step
  - tutorial
date: "2021-02-14"
last_modified_at: "2021-02-22"
---
{{page.excerpt}}\\
See [documentation for LQROneStepLTI](/documentation/LQROneStepLTI/) for more information.

***

To open this tutorial execute the following command in the MATLAB command window
~~~m
open LQROneStepLTITutorial
~~~

Use synthetic system matrices $\mathbf{A}$, $\mathbf{B}$, $\mathbf{Q}$, $\mathbf{R}$, and $\mathbf{E}$
~~~m
n = 5;
m = 3;
A = [0.6085    0.0188    0.9615    0.6161    0.0494;
     0.3959    0.6575    0.7305    0.5338    0.4611;
     0.1743    0.9322    0.3419    0.2229    0.1466;
     0.5204    0.7850    0.2257    0.4315    0.8538;
     0.8603    0.8842    0.1132    0.7550    0.3850];
B = [0.3843    0.7494    0.4509;
     0.1446    0.5369    0.0092;
     0.6133    0.6413    0.6322;
     0.3401    0.4020    0.3725;
     0.7084    0.4744    0.0031];
Q = [0.8057    0.9316    0.9227    0.7569    0.6049;
     0.9316    2.6200    1.2024    1.4863    1.6318;
     0.9227    1.2024    1.1229    0.9413    0.6800;
     0.7569    1.4863    0.9413    1.3068    1.2506;
     0.6049    1.6318    0.6800    1.2506    1.5626];
R = [1.0134    0.5867    0.9654;
     0.5867    0.4666    0.4427;
     0.9654    0.4427    1.5383];
E = [1     1     1     0     1;
     0     0     1     0     0;
     0     0     1     1     1];
~~~

Synthesize regulator gain using the one-step method (with some optional parameters)
~~~m
opts.verbose = true;
[Kinf,Pinf] = LQROneStepLTI(A,C,Q,R,E,opts);
Kinf
trace(Pinf)
~~~
~~~text
----------------------------------------------------------------------------------
Running one-step algorithm with: epsl = 1e-05 | maxIt = 1000.
Convergence reached with: epsl = 1e-05 | maxIt = 1000.
A total of 16 iterations were run.
----------------------------------------------------------------------------------
Kinf =
    1.2672    1.4884   -0.6235         0    0.9940
         0         0    1.3406         0         0
         0         0   -0.0036    1.4180    0.0848
ans =
   24.8584
~~~
Notice that the filter gain has the desired sparsity pattern.\\
Simulate the dynamics of the synthetic system
~~~m
% Generate random initial covariance
P0 = rand()*eye(n);
% Simulation time
SimIt = 50;
% Initialise error cell
x = cell(1,SimIt);
% Generate random initial error
x0 = transpose(mvnrnd(zeros(n,1),P0));
for j = 1:SimIt
    if j == 1
        x{1,j} = (A-B*Kinf)*x0;
    else
        x{1,j} = (A-B*Kinf)*x{1,j-1};
    end
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
xPlot = zeros(SimIt,1);
for j = 1:SimIt
   xPlot(j,1) =norm(x{1,j}(:,1));
end
plot(0:SimIt, [norm(x0(:)); xPlot(:,1)],'LineWidth',3);
set(gcf, 'Position', [100 100 900 550]);
ylabel('$\|\mathbf{x}_{OS}(k)\|_2$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;
~~~
{:refdef: style="text-align: center;"}
![image-title-here](/assets/img/LQROneStepLTITutorial.svg){:class="img-responsive"}
{: refdef}
