---
title: kalmanCentralizedLTI
showTitle: true
permalink: /documentation/kalmanCentralizedLTI/
layout: single
classes: wide
sidebar:
  title: "kalmanCentralizedLTI"
  nav: sidebar-kalmanCentralizedLTI
---
# Sintax
~~~m
[K_inf,P_inf] = kalmanCentralizedLTI(A,C,Q,R)
[K_inf,P_inf] = kalmanCentralizedLTI(A,C,Q,R,opts)
~~~
***

# Description
Consider a generic **LTI system** of the form

$$
\begin{cases}
\mathbf{x}(k+1)=\mathbf{A}\mathbf{x}(k)+\mathbf{B}\mathbf{u}(k)+\mathbf{w}(k)\\
\mathbf{y}(k)=\mathbf{C}\mathbf{x}(k)+\mathbf{v}(k)
\end{cases}\:,
$$

where $\mathbf{x}(k)\in\mathbb{R}^{n}$ is the state vector, $\mathbf{u}(k)\in \mathbb{R}^{m}$ is the input vector, which is assumed to be known, and $\mathbf{y}(k)\in\mathbb{R}^{o}$ is the output of the system. The vectors $\mathbf{w}(k)$ and $\mathbf{v}(k)$ are the process and observation noise, modelled as zero-mean uncorrelated white Gaussian processes with associated covariance matrices $\mathbf{Q} \succeq \mathbf{0}\in\mathbb{R}^{n\times n}$ and  $\mathbf{R} \succ \mathbf{0}\in\mathbb{R}^{o\times o}$, respectively.
{: .text-justify}

Consider a standard **Kalman filter**. The prediction step follows

$$
\hat{\mathbf{x}}(k|k-1) = \mathbf{A}\hat{\mathbf{x}}(k-1|k-1)+\mathbf{B}\mathbf{u}(k-1)\:,
$$

where $\hat{\mathbf{x}}(k\|k-1)$ denotes the predicted state estimate at time instant $k$ and $\hat{\mathbf{x}}(k\|k)$ the filtered state estimate at time instant $k$. The filtering step follows
{: .text-justify}

$$
\hat{\mathbf{x}}(k|k) = \hat{\mathbf{x}}(k|k-1)+\mathbf{K}(k)\left(\mathbf{y}(k) - \mathbf{C}\hat{\mathbf{x}}(k|k-1)\right)\:,
$$

where $\mathbf{K}(k)\in\mathbb{R}^{n\times o}$ is the **filter gain**. It is often very useful to find a **steady-state constant gain** $\mathbf{K}_{\infty}$ instead.
{: .text-justify}

The commands
{% highlight m%}[K_inf,P_inf] = kalmanCentralizedLTI(A,C,Q,R,E)
[K_inf,P_inf] = kalmanCentralizedLTI(A,C,Q,R,E,opts){% endhighlight m%} compute the **steady-state centralized kalman gain**.
{: .text-justify}

***

# Input arguments
### Required
-  ```A``` : matrix $\mathbf{A}$ of the dynamics of the LTI system
-  ```C``` : matrix $\mathbf{C}$ of the dynamics of the LTI system
-  ```Q``` : covariance matrix of the process noise, $\mathbf{Q}$
-  ```R``` : covariance matrix of the observation noise, $\mathbf{R}$

### Optional
- ```opts```: struct of **optional** arguments (**assumes default** value for each parameter which is not assigned by the user)
  - ```epsl```: minimum relative improvement on the objective function of the optimization problem (default: ```opts.epsl = 1e-5```)
  {: .text-justify}
  - ```maxIt```: maximum number of iterations until convergence (default: ```opts.maxIt = 1000```)
  - ```verbose```: display algorithm status messages (default: ```opts.verbose = false```)
  - ```P0```: initialization estimation error covariance matrix (default: ```opts.P0 = zeros(n,n)```)

***

# Output Arguments

- ```K_inf```: steady-state filter gain matrix $\mathbf{K}_{\infty}$
- ```P_inf```: steady-state estimation error covariance matrix $\mathbf{P}_{\infty}$

***

# Examples
### Synthetic system
To open this tutorial execute the following command in the MATLAB command window
~~~m
open kalmanCentralizedLTITutorial
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
Synthesize the centralized Kalman filter gain (with some optional parameters)
~~~m
opts.verbose = true;
[Kinf,Pinf] = kalmanCentralizedLTI(A,C,Q,R,opts);
Kinf
trace(Pinf)
~~~
~~~text
----------------------------------------------------------------------------------
Computing centralized kalman filter with: epsl = 1e-05 | maxIt = 1000.
Convergence reached with: epsl = 1e-05 | maxIt = 1000.
A total of 8 iterations were run.
----------------------------------------------------------------------------------
Kinf =
    0.1472   -0.0416    0.4134   -0.0271
   -0.6140    0.2344    1.0917   -0.1059
    0.3854   -0.0458   -0.0093    0.4284
   -0.0032    0.2610   -0.6101    0.3653
    0.4213   -0.1861   -0.4752    0.0999
ans =
    9.5423
~~~

***

# References
[1] <a href="https://www.sciencedirect.com/science/article/pii/S0967066118300571" target="_blank">Viegas, D., Batista, P., Oliveira, P. and Silvestre, C., 2018. Discrete-time distributed Kalman filter design for formations of autonomous vehicles. Control Engineering Practice, 75, pp.55-68.</a>
{: .text-justify}
