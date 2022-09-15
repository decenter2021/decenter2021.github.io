---
title: MHEMovingFiniteHorizonLTI
showTitle: true
excerpt: Documentation for function MHEMovingFiniteHorizonLTI.
permalink: /documentation/MHEMovingFiniteHorizonLTI/
layout: single
classes: wide
sidebar:
  title: "MHEMovingFiniteHorizonLTI"
  nav: sidebar-MHEMovingFiniteHorizonLTI
tags:
    - estimation
    - lti
    - moving-finite-horizon
    - documentation
    - moving-horizon-estimation
date: "2022-04-05"
last_modified_at: "2022-09-15"
published: true
---

# Sintax
~~~m
[Kinf,Pinf,Pseq] = MHEMovingFiniteHorizonLTI(A,C,Q,R,E,W)
[Kinf,Pinf,Pseq] = MHEMovingFiniteHorizonLTI(A,C,Q,R,E,W,opts)
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

Consider a novel **moving horizon estimation (MHE)** framework proposed in [[1, Section II]](#references).

Let $\hat{\mathbf{x}}(\tau+1\|\tau\|k)$ denote the global predicted state estimate at time instant $\tau+1$ as computed at time instant $k$ and $\hat{\mathbf{x}}(\tau\|\tau\|k)$ denote the global filtered state estimate at time instant $\tau$ as computed at time instant $k$.
{: .text-justify}

For each time instant $k$, consider the finite window $\{k-W+1,\ldots,k\}$, where $W \in\mathbb{N}$ is the finite window length. The novel MHE framework proposed in [[1, Section II]](#references), from which a new filter design stems, is built on multiple prediction-filtering steps employed in a Luenberger Kalman filter for each individual system. An iteration of the proposed filter for time instant $k$ follows
{: .text-justify}

$$
\begin{cases}
\hat{\mathbf{x}}(k-W|k-W|k) = \hat{\mathbf{x}}(k-W|k-W|k-W)\\
\hat{\mathbf{x}}(\tau|\tau\!-\!1|k) = \mathbf{A}\hat{\mathbf{x}}(\tau\!-\!1|\tau\!-\!1|k)+\mathbf{B}\mathbf{u}(\tau\!-\!1)\\
\hat{\mathbf{x}}(\tau|\tau|k) = \hat{\mathbf{x}}(\tau|\tau-1|k)+\mathbf{K}(\tau|k)\left(\mathbf{y}(\tau) - \mathbf{C}\hat{\mathbf{x}}(\tau|\tau-1|k)\right)
\end{cases},
$$

 and $\mathbf{K}(\tau\|k)\in \mathbb{R}^{n\times o}$ is the global filter gain for time instant $\tau$ computed at time instant $k$, with $\tau = k-W+1,\ldots,k$.
 {: .text-justify}

 Let matrix $\mathbf{E}  \in\mathbb{R}^{n\times o}$ denote a **sparsity pattern**. The set of matrices which obey the sparsity constraint determined by $\mathbf{E}$ is defined as
 {: .text-justify}

 $$
 \mathrm{Sparse}(\mathbf{E}) :=\left\{[\mathbf{K}]_{ij}\in\mathbb{R}^{n\times o}: [\mathbf{E}_{ij}] = 0 \implies [\mathbf{K}]_{ij}= 0;\: i= 1,...,n, \:j=1,...,o \right\}.
 $$


Let $\mathbf{P}(\tau+1\|\tau\|k)$ denote the global predicted estimation error covariance matrix at time instant $\tau+1$ as computed at time instant $k$ and $\mathbf{P}(\tau\|\tau\|k)$ denote the global global filtered estimation error covariance matrix at time instant $\tau$ as computed at time instant $k$. The dynamics of the estimation error covariance matrix are given by
{: .text-justify}

$$
\begin{cases}
\mathbf{P}(k-W|k-W|k) = \mathbf{P}(k-W|k-W|k-W)\\
\mathbf{P}(\tau|\tau-1|k) = \mathbf{A}\mathbf{P}(\tau-1|\tau-1|k)\mathbf{A}^T+\mathbf{Q}\\
\mathbf{P}(\tau|\tau|k) = \mathbf{K}(\tau|k)\mathbf{R}\mathbf{K}^T(\tau|k)+(\mathbf{I}-\mathbf{K}(\tau|k)\mathbf{C})\mathbf{P}(\tau|\tau-1|k)(\mathbf{I}-\mathbf{K}(\tau|k)\mathbf{C})^T
\end{cases}\:,
$$

with $\tau = k-W+1,\ldots,k$, which is a recursive expression of prediction-filtering estimation error covariance steps of the Luenberger Kalman filter.
{: .text-justify}

The goal of [[1]](#references) is to design a steady-state sequence of global gains
$$\mathbf{K}_{\infty}(\tau), \tau = 1,...,W_{ss},$$
instead of a single constant gain. If such sequence stabilizes the error dynamics of the filter then the estimation error covariance converges to a steady-state solution $\mathbf{P}_{\infty}$.

The global steady-state filter dynamics are given by

$$
\begin{cases}
\hat{\mathbf{x}}(k-W_{ss}|k-W_{ss}|k) = \hat{\mathbf{x}}(k-W_{ss}|k-W_{ss}|k-W_{ss})\\
\hat{\mathbf{x}}(\tau|\tau\!-\!1|k) = \mathbf{A}\hat{\mathbf{x}}(\tau\!-\!1|\tau\!-\!1|k)+\mathbf{B}\mathbf{u}(\tau\!-\!1)\\
\hat{\mathbf{x}}(\tau|\tau|k) = \hat{\mathbf{x}}(\tau|\tau-1|k)+\mathbf{K}_{\infty}(\tau-(k-W_{ss}))\left(\mathbf{y}(\tau) - \mathbf{C}\hat{\mathbf{x}}(\tau|\tau-1|k)\right)
\end{cases}.
$$

The commands
{% highlight m%}[Kinf,Pinf] = MHEMovingFiniteHorizonLTI(A,C,Q,R,E,W)
[Kinf,Pinf] = MHEMovingFiniteHorizonLTI(A,C,Q,R,E,W,opts){% endhighlight m%} compute the **steady-state distributed sequence of gains** that aims at solving the optimization problem

$$
\begin{aligned}
& \underset{\begin{subarray}{c}\mathbf{K}_{\infty}(\tau) \in \mathbb{R}^{n\times o},\\ \tau = 1,...,W_{ss} \end{subarray}}{\text{minimize}}
& & \mathrm{tr}(\mathbf{P}_{\infty}) \\
& \text{subject to}
& & \mathbf{K}_{\infty}(\tau) \in \mathrm{Sparse}(\mathbf{E})\:, \tau = 1,...,W_{ss}
\end{aligned}
$$

***

# Computational complexity
The moving finite-horizon optimization problem is solved using the efficient sparse equation solver proposed in [[2]](#references). See [sparseEqSolver](/documentation/sparseEqSolver/) for the implementation of the solver.
{: .text-justify}

Define the set $\chi$ of integer pairs of the form $(i,j)$ to index the nonzero entries of $\mathbf{E}$ as
{: .text-justify}

$$
\begin{cases}
(i,j) \in \chi &\;,\;\left[\mathbf{E}\right]_{i,j} \neq 0\\
(i,j) \notin \chi &\;,\;\text{otherwise}
\end{cases}, i = 1,...,n,\: j = 1,...,o\:.
$$

It is shown in [[2]](#references) that each iteration of the algorithm requires $\mathcal{O}(\|\chi\|^3)$ floating-point operations, where $\|\chi\|$ denotes the cardinality of set $\chi$. In the field of distributed estimation and control theory, $\|\chi\|$ is usually given by $\|\chi\| \approx cn$, where $c\in \mathbb{N}$ is a constant. It, thus, follows that each iteration requires $\mathcal{O}(n^3)$ floating-point operations, thus it has the same complexity as a centralized gain computation.
{: .text-justify}

***

# Input arguments
### Required
-  ```A``` : matrix $\mathbf{A}$ of the dynamics of the LTI system
-  ```C``` : matrix $\mathbf{C}$ of the dynamics of the LTI system
-  ```Q``` : covariance matrix of the process noise, $\mathbf{Q}$
-  ```R``` : covariance matrix of the observation noise, $\mathbf{R}$
-  ```E``` : sparsity pattern $\mathbf{E}$
-  ```W``` : steady-state window length  $W_{ss}$

### Optional
- ```opts```: struct of **optional** arguments (**assumes default** value for each parameter which is not assigned by the user)
  - ```epsl_inf```: minimum relative improvement on the objective function of the steady optimization problem (default: ```opts.epsl_inf = 1e-4```)
  {: .text-justify}
  - ```epsl```: minimum relative improvement on the objective function of the optimization problem in each finite window (default: ```opts.epsl = opts.epsl_inf/10```)
  {: .text-justify}
  - ```maxIt```: maximum number of iterations to run until convergence (default: ```opts.maxIt = 100```)
  {: .text-justify}
  - ```verbose```: display algorithm status messages (default: ```opts.verbose = false```)
  - ```P0```: initialization estimation error covariance matrix (default: ```opts.P0 = zeros(n,n)```)

***

# Output Arguments

- ```Kinf```: $W_{ss} \times 1$ cell array with the steady-state filter gain matrix sequence $$\mathbf{K}_{\infty}(\tau), \tau = 1,...,W_{ss},$$
- ```Pinf```: steady-state estimation error covariance matrix $\mathbf{P}_{\infty}$
- ```Pseq```: $(W_{ss}+1) \times 1$ cell array of the covariance matrices throughout the window in the last iteration

***

# Examples

See [moving finite-horizon Kalman filter tutorial for LTI systems](/tutorials/MHEMovingFiniteHorizonLTI/) for a tutorial.

***

# References

[1] *[not published yet]*

[2] {% for paper in site.data.references%}{% if paper.key == "Pedroso2021Efficient" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}
