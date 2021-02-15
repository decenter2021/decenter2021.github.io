---
layout: single
title: One-step Kalman filter tutorial for LTI systems
titleShort: One-step   
permalink: /tutorials/kalmanOneStepLTI/
layout: single
classes: wide
sidebar:
  title: "Tutorials"
  nav: sidebar-tutorials
showTitle: " "
category: "estimationLTITutorial"

date: "2021-02-14"
last_modified_at: "2021-02-14"

pageTag: "tutorials-estOSLTI"
excerpt: "Tutorial on decentralized Kalman filter synthesis using the one-step method."
excerptShort: "Decentralized Kalman filter synthesis using the one-step method."

frontPage: "true"
header:
  teaser: "/assets/img/kalmanOneStepLTITutorial.svg"
---
{{page.excerpt}}\\
See [documentation for kalmanOneStepLTI](/documentation/kalmanOneStepLTI/) for more information.

***

To open this tutorial execute the following command in the MATLAB command window
~~~m
open kalmanOneStepLTITutorial
~~~


Use synthetic system matrices $\mathbf{A}$, $\mathbf{C}$, $\mathbf{Q}$, $\mathbf{R}$, and $\mathbf{E}$ from [[Section 4.1, 1]](#references)
~~~m
n = 5;
o = 4;
A = [0.152  0.092   0.235   0.642   0.506;
     0.397  0.615   0.448   0.221   0.279;
     0.375  0.011   0.569   0.837   0.747;
     0.131  0.573   0.061   0.971   0.237;
     0.435  0.790   0.496   0.846   0.957];
C = [0.620  0.255   0.725   0.404   0.511;
     0.600  0.859   0.230   1.988   0.061;
     0.173  0.911   0.576   0.090   0.726;
     0.090  0.700   0.811   0.321   0.557];
Q = [3.318  4.662   1.598   -1.542  -1.999;
     4.662  11.520  2.608   -2.093  -5.442;
     1.598  2.608   4.691   0.647   -0.410;
     -1.542 -2.093  0.647   2.968   0.803;
     -1.999 -5.442  -0.410  0.803   2.851];
R = [3.624  2.601   -0.042  -0.944;
     2.601  7.343   -0.729  -2.786;
     -0.042 -0.729  0.745   -0.242;
     -0.944 -2.786  -0.242  1.612];
E = [1   0   1   1;
     0   1   0   1;
     0   0   1   0;
     1   1   1   0;
     1   1   0   1];
~~~

Synthesize Kalman filter gain using the one-step method (with some optional parameters)
~~~m
opts.verbose = true;
[Kinf,Pinf] = kalmanOneStepLTI(A,C,Q,R,E,opts);
Kinf
trace(Pinf)
~~~
~~~text
----------------------------------------------------------------------------------
Running one-step algorithm with: epsl = 1e-05 | maxIt = 1000.
Convergence reached with: epsl = 1e-05 | maxIt = 1000.
A total of 13 iterations were run.
----------------------------------------------------------------------------------
Kinf =
    0.1619         0    0.3159   -0.0044
         0    0.1621         0    0.1978
         0         0    0.6605         0
    0.0558    0.3147   -0.2876         0
    0.3430   -0.0937         0   -0.1125
ans =
   24.3705
~~~
Notice that the filter gain has the desired sparsity pattern.\\
Simulate the error dynamics for the synthetic system
~~~m
% Generate random initial covariance
P0 = 10*rand(n,n);
P0 = (P0*P0');
% Simulation time
SimIt = 100;
% Initialise error cell
error = cell(1,SimIt);
% Generate random initial error
error0 = transpose(mvnrnd(zeros(n,1),P0));
for j = 1:SimIt
    if j == 1
        error{1,j} = (eye(n)-Kinf*C)*(A*error0+...
            mvnrnd(zeros(n,1),Q)')-Kinf*mvnrnd(zeros(o,1),R)';
    else
        error{1,j} = (eye(n)-Kinf*C)*(A*error{1,j-1}+...
            mvnrnd(zeros(n,1),Q))'-Kinf*mvnrnd(zeros(o,1),R)';
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
errorPlot = zeros(SimIt,1);
for j = 1:SimIt
   errorPlot(j,1) =norm(error{1,j}(:,1));
end
plot(0:SimIt, [norm(error0(:)); errorPlot(:,1)],'LineWidth',3);
set(gcf, 'Position', [100 100 900 550]);
ylabel('$\|\hat{\mathbf{x}_{OS}}(k|k)-\mathbf{x}(k)\|_2$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;
~~~
{:refdef: style="text-align: center;"}
![image-title-here](/assets/img/kalmanOneStepLTITutorial.svg){:class="img-responsive"}
{: refdef}

# References
[1] <a href="https://www.sciencedirect.com/science/article/pii/S0967066118300571" target="_blank">Viegas, D., Batista, P., Oliveira, P. and Silvestre, C., 2018. Discrete-time distributed Kalman filter design for formations of autonomous vehicles. Control Engineering Practice, 75, pp.55-68.</a>
{: .text-justify}
