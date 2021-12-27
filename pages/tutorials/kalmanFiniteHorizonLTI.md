---
layout: single
title: Finite-horizon Kalman filter tutorial for LTI systems
showTitle: true
excerpt: "Tutorial on decentralized Kalman filter synthesis using the finite-horizon method."
permalink: /tutorials/kalmanFiniteHorizonLTI/
classes: wide
sidebar:
  title: "Tutorials"
  nav: sidebar-tutorials
header:
  teaser: "/assets/img/kalmanFiniteHorizonLTITutorial.svg"
tags:
  - tutorial
  - finite-horizon
  - lti
  - estimation
date: "2021-02-14"
last_modified_at: "2021-12-27"
---
{{page.excerpt}}\\
See [documentation for kalmanFiniteHorizonLTI](/documentation/kalmanFiniteHorizonLTI/) for more information.

***

To open this tutorial execute the following command in the MATLAB command window
~~~m
open kalmanFiniteHorizonLTITutorial
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

Synthesize Kalman filter gain using the finite-horizon method (with some optional parameters)
~~~m
opts.verbose = true;
opts.W = 20;
opts.maxOLIt = 10;
[Kinf,Pinf] = kalmanFiniteHorizonLTI(A,C,Q,R,E,opts);
~~~
~~~text
----------------------------------------------------------------------------------
Running finite-horizon algorithm with:
epsl = 1e-05 | W = 20 | maxOLIt = 10 | findWindowSize = false.
Finite-horizon algorithm was unable to reach convergence with the specified
parameters: epsl = 1e-05 | W = 20 | maxOLIt = 10
A total of 10 outer loop iterations were run, out of which 10.0% converged within
the specified minimum improvement.
Sugested actions:
- Manually tune 'W', 'epsl' and 'maxOLIt' (in this order);
- Increase 'W', the finite window length;
- Increase 'epsl', the minimum relative improvement on the objective function
optimization problem.
- Increase 'maxOLIt', the maximum number of outer loop iterations.
----------------------------------------------------------------------------------
~~~
Notice that it was not possible to reach convergence with the selected parameters. In fact, only 10% of the outer loop iterations (just one iteration) converged.  Thus, increasing the window size should allow for convergence.
{: .text-justify}
~~~m
opts.verbose = true;
opts.W = 30;
opts.maxOLIt = 10;
[Kinf,Pinf] = kalmanFiniteHorizonLTI(A,C,Q,R,E,opts);
Kinf
trace(Pinf)
~~~
~~~text
----------------------------------------------------------------------------------
Running finite-horizon algorithm with:
epsl = 1e-05 | W = 30 | maxOLIt = 10 | findWindowSize = false.
Convergence reached with: epsl = 1e-05 | W = 30 | maxOLIt = 10
A total of 6 outer loop iterations were run, out of which 100.0% converged within
the specified minimum improvement.
----------------------------------------------------------------------------------
Kinf =
   -0.0512         0    0.4820    0.0983
         0    0.1743         0    0.3023
         0         0    0.8033         0
   -0.0162    0.2856   -0.2934         0
    0.1032   -0.1872         0    0.0835
ans =
   20.4429
~~~
Notice that the finite-horizon synthesis converged and the filter gain has the desired sparsity pattern.

Alternatively, the finite-window size may be found iteratively
~~~m
opts.verbose = true;
opts.maxOLIt = 10;
opts.W = 10;
opts.findWindowLength = true;
[Kinf,Pinf] = kalmanFiniteHorizonLTI(A,C,Q,R,E,opts);
Kinf
trace(Pinf)
~~~

~~~text
----------------------------------------------------------------------------------
Running finite-horizon algorithm with:
epsl = 1e-05 | W = 10 | maxOLIt = 10 | findWindowSize = true.
Trying new window length W = 15
Trying new window length W = 23
Convergence reached with: epsl = 1e-05 | W = 23 | maxOLIt = 10
A total of 6 outer loop iterations were run, out of which 100.0% converged within
the specified minimum improvement.
----------------------------------------------------------------------------------
Kinf =
   -0.0512         0    0.4820    0.0983
         0    0.1743         0    0.3023
         0         0    0.8033         0
   -0.0162    0.2856   -0.2934         0
    0.1032   -0.1872         0    0.0835
ans =
   20.4429
~~~

Simulate the error dynamics for the synthetic system
{: .text-justify}
~~~m
% Generate random initial covariance
P0 = rand(n,n);
P0 = 100*(P0*P0');
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
%title('Norm of estimation error simulation - centralized gain');
errorPlot = zeros(SimIt,1);
for j = 1:SimIt
   errorPlot(j,1) =norm(error{1,j}(:,1));
end
plot(0:SimIt, [norm(error0(:)); errorPlot(:,1)],'LineWidth',3);
set(gcf, 'Position', [100 100 900 550]);
ylabel('$\|\hat{\mathbf{x}}_{FH}(k|k)-\mathbf{x}(k)\|_2$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;
~~~
{:refdef: style="text-align: center;"}
![image-title-here](/assets/img/kalmanFiniteHorizonLTITutorial.svg){:class="img-responsive"}
{: refdef}

{: .text-justify}

# References
[1] {% for paper in site.data.references%}{% if paper.key == "Viegas2018Discrete" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}
