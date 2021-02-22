---
title: LQRCentralizedLTI
showTitle: true
permalink: /documentation/LQRCentralizedLTI/
layout: single
classes: wide
sidebar:
  title: "LQRCentralizedLTI"
  nav: sidebar-LQRCentralizedLTI

tags:
    - control
    - LTI
    - documentation
date: "2021-02-14"
last_modified_at: "2021-02-22"
---
# Sintax
~~~m
[K_inf,P_inf] = LQRCentralizedLTI(A,B,Q,R,E)
[K_inf,P_inf] = LQRCentralizedLTI(A,B,Q,R,E,opts)
~~~
***

# Description
Consider a generic **LTI system** of the form

$$
\mathbf{x}(k+1)=\mathbf{A}\mathbf{x}(k)+\mathbf{B}\mathbf{u}(k)\;
$$

where $\mathbf{x}(k)\in\mathbb{R}^{n}$ is the state vector, $\mathbf{u}(k)\in \mathbb{R}^{m}$ is the input vector, and $\mathbf{A}$ and $\mathbf{B}$ are constant matrices of appropriate dimensions.
{: .text-justify}

Consider a standard **LQR regulator**

$$
\mathbf{u}(k) = -\mathbf{K}(k)\mathbf{x}(k)\:,
$$

where $\mathbf{K}(k)\in\mathbb{R}^{m\times n}$ is the **regulator gain**. It is often very useful to find a **steady-state constant gain** $\mathbf{K}_{\infty}$ instead. Consider a regulator **cost function**
{: .text-justify}

$$
J(k) = \sum_{\tau=k}^{\infty}\left(\mathbf{x}^T(\tau)\mathbf{Q}\mathbf{x}(\tau)+\mathbf{u}^T(\tau)\mathbf{R}\mathbf{u}(\tau)\right)\:,
$$

where $\mathbf{Q} \succeq \mathbf{0}\in\mathbb{R}^{n\times n}$ and  $\mathbf{R} \succ \mathbf{0}\in\mathbb{R}^{m\times m}$ are the weighing matrices of the state vector and control action, respectively. It can be shown that there is a matrix $\mathbf{P}(k) \succ \mathbf{0}\in\mathbb{R}^{n\times n}$ such that
{: .text-justify}

$$
J(k) = \mathbf{x}^T(k)\mathbf{P}(k)\mathbf{x}(k)\:.
$$

The commands
{% highlight m%}[K_inf,P_inf] = LQRCentralizedLTI(A,B,Q,R,E)
[K_inf,P_inf] = LQRCentralizedLTI(A,B,Q,R,E,opts){% endhighlight m%} compute the well-known **steady-state centralized gain** that solves the optimization problem
{: .text-justify}

$$
\begin{aligned}
& \underset{\begin{subarray}{c}\mathbf{K}_{\infty}\in \mathbb{R}^{m\times n} \end{subarray}}{\text{minimize}}
& & J(0)\:.
\end{aligned}
$$
{: .text-justify}

***

# Input arguments
### Required
-  ```A``` : matrix $\mathbf{A}$ of the dynamics of the LTI system
-  ```B``` : matrix $\mathbf{B}$ of the dynamics of the LTI system
-  ```Q``` : state weighting matrix, $\mathbf{Q} \succeq \mathbf{0}$
-  ```R``` : control action weighting matrix, $\mathbf{R} \succeq \mathbf{0}$

### Optional
- ```opts```: struct of **optional** arguments (**assumes default** value for each parameter which is not assigned by the user)
  - ```epsl```: minimum relative improvement on the objective function of the optimization problem (default: ```opts.epsl = 1e-5```)
  {: .text-justify}
  - ```maxIt```: maximum number of iterations until convergence (default: ```opts.maxIt = 1000```)
  - ```verbose```: display algorithm status messages (default: ```opts.verbose = false```)

***

# Output Arguments

- ```K_inf```: steady-state regulator gain matrix $\mathbf{K}_{\infty}$
- ```P_inf```: steady-state matrix $\mathbf{P}_{\infty}$

***

# Examples
### Synthetic system
To open this example execute the following command in the MATLAB command window
~~~m
open LQRCentralizedLTITutorial
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
Synthesize the centralized regulator gain (with some optional parameters)
~~~m
opts.verbose = true;
[Kinf,Pinf] = LQRCentralizedLTI(A,B,Q,R,opts);
Kinf
~~~
~~~text
----------------------------------------------------------------------------------
Computing centralized LQR gain with: epsl = 1e-05 | maxIt = 1000.
Convergence reached with: epsl = 1e-05 | maxIt = 1000.
A total of 2 iterations were run.
----------------------------------------------------------------------------------
Kinf =
    0.1064   -0.0372   -0.7317   -0.0232    0.1165
    0.9813    1.3256    1.3547    1.0714    0.7619
   -0.3304   -0.2918    0.1180   -0.2828   -0.2849
~~~
