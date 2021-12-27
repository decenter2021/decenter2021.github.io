---
title: kalmanOneStepLTI
showTitle: true
excerpt: Documentation for function kalmanOneStepLTI.
permalink: /documentation/kalmanOneStepLTI/
layout: single
classes: wide
sidebar:
  title: "kalmanOneStepLTI"
  nav: sidebar-kalmanOneStepLTI
tags:
    - estimation
    - lti
    - one-step
    - documentation
date: "2021-02-14"
last_modified_at: "2021-12-27"

---
# Sintax
~~~m
[K_inf,P_inf] = kalmanOneStepLTI(A,C,Q,R,E)
[K_inf,P_inf] = kalmanOneStepLTI(A,C,Q,R,E,opts)
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

Let matrix $\mathbf{E}  \in\mathbb{R}^{n\times o}$ denote a **sparsity pattern**. The set of matrices which obey the sparsity constraint determined by $\mathbf{E}$ is defined as
{: .text-justify}

$$
\mathrm{Sparse}(\mathbf{E}) :=\left\{[\mathbf{K}]_{ij}\in\mathbb{R}^{n\times o}: [\mathbf{E}_{ij}] = 0 \implies [\mathbf{K}]_{ij}= 0;\: i= 1,...,n, \:j=1,...,o \right\}.
$$

The commands
{% highlight m%}[K_inf,P_inf] = kalmanOneStepLTI(A,C,Q,R,E)
[K_inf,P_inf] = kalmanOneStepLTI(A,C,Q,R,E,opts){% endhighlight m%} compute the **steady-state distributed gain** that aims at solving the optimization problem

$$
\begin{aligned}
& \underset{\begin{subarray}{c}\mathbf{K}_{\infty}\in \mathbb{R}^{n\times o} \end{subarray}}{\text{minimize}}
& & \mathrm{tr}(\mathbf{P}_{\infty}) \\
& \text{subject to}
& & \mathbf{K}_{\infty} \in \mathrm{Sparse}(\mathbf{E})\:,
\end{aligned}
$$

where $\mathbf{P}_{\infty}$ is the **steady-state estimation error covariance matrix**, using the **one-step method** proposed in [[Section 4, 1]](#references).
{: .text-justify}

***

# Computational complexity
The one-step optimization problem is solved using the efficient sparse equation solver proposed in [[2]](#references). See [sparseEqSolver](/documentation/sparseEqSolver/) for the implementation of the solver.
{: .text-justify}

Define the set $\chi$ of integer pairs of the form $(i,j)$ to index the nonzero entries of $\mathbf{E}$ as
{: .text-justify}

$$
\begin{cases}
(i,j) \in \chi &\;,\;\left[\mathbf{E}\right]_{i,j} \neq 0\\
(i,j) \notin \chi &\;,\;\text{otherwise}
\end{cases}, i = 1,...,n,\: j = 1,...,o\:.
$$

It is shown in [[2]](#references) that each gain computation of the algorithm requires $\mathcal{O}(\|\chi\|^3)$ floating-point operations, where $\|\chi\|$ denotes the cardinality of set $\chi$. In the field of distributed estimation and control theory, $\|\chi\|$ is usually given by $\|\chi\| \approx cn$, where $c\in \mathbb{N}$ is a constant. It, thus, follows that each iteration requires $\mathcal{O}(n^3)$ floating-point operations, thus it has the same complexity as a centralized gain computation.
{: .text-justify}

***

# Input arguments
### Required
-  ```A``` : matrix $\mathbf{A}$ of the dynamics of the LTI system
-  ```C``` : matrix $\mathbf{C}$ of the dynamics of the LTI system
-  ```Q``` : covariance matrix of the process noise, $\mathbf{Q}$
-  ```R``` : covariance matrix of the observation noise, $\mathbf{R}$
-  ```E``` : sparsity pattern $\mathbf{E}$

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

See [One-step Kalman filter tutorial for LTI systems](/tutorials/kalmanOneStepLTI/) for a tutorial.

***

# References
[1] {% for paper in site.data.references%}{% if paper.key == "Viegas2018Discrete" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}

[2] {% for paper in site.data.references%}{% if paper.key == "Pedroso2021Efficient" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}
