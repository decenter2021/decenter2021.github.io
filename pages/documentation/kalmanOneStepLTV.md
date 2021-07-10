---
title: kalmanOneStepLTV
showTitle: true
excerpt: Documentation for function kalmanOneStepLTV.
permalink: /documentation/kalmanOneStepLTV/
layout: single
classes: wide
sidebar:
  title: "kalmanOneStepLTV"
  nav: sidebar-kalmanOneStepLTV
tags:
  - estimation
  - ltv
  - one-step
  - documentation
date: "2021-07-10"
last_modified_at: "2021-07-10"
---
# Sintax
~~~m
[K,Ppred,Pfilt] = kalmanOneStepLTV(system,E,Pprev)
~~~
***

# Description
Consider a generic **LTV system** of the form

$$
\begin{cases}
\mathbf{x}(k+1)=\mathbf{A}(k)\mathbf{x}(k)+\mathbf{B}(k)\mathbf{u}(k)+\mathbf{w}(k)\\
\mathbf{y}(k)=\mathbf{C}(k)\mathbf{x}(k)+\mathbf{v}(k)
\end{cases}\:,
$$

where $\mathbf{x}(k)\in\mathbb{R}^{n}$ is the state vector, $\mathbf{u}(k)\in \mathbb{R}^{m}$ is the input vector, which is assumed to be known, and $\mathbf{y}(k)\in\mathbb{R}^{o}$ is the output of the system. The vectors $\mathbf{w}(k)$ and $\mathbf{v}(k)$ are the process and observation noise, modelled as zero-mean uncorrelated white Gaussian processes with associated covariance matrices $\mathbf{Q}(k) \succeq \mathbf{0}\in\mathbb{R}^{n\times n}$ and  $\mathbf{R}(k) \succ \mathbf{0}\in\mathbb{R}^{o\times o}$, respectively.
{: .text-justify}

Consider a standard **Kalman filter**. The prediction step follows

$$
\hat{\mathbf{x}}(k|k-1) = \mathbf{A}(k-1)\hat{\mathbf{x}}(k-1|k-1)+\mathbf{B}(k-1)\mathbf{u}(k-1)\:,
$$

where $\hat{\mathbf{x}}(k\|k-1)$ denotes the predicted state estimate at time instant $k$ and $\hat{\mathbf{x}}(k\|k)$ the filtered state estimate at time instant $k$. The filtering step follows
{: .text-justify}

$$
\hat{\mathbf{x}}(k|k) = \hat{\mathbf{x}}(k|k-1)+\mathbf{K}(k)\left(\mathbf{y}(k) - \mathbf{C}(k)\hat{\mathbf{x}}(k|k-1)\right)\:,
$$

where $\mathbf{K}(k)\in\mathbb{R}^{n\times o}$ is the **filter gain** at time instant $k$.

Let matrix $\mathbf{E}  \in\mathbb{R}^{n\times o}$ denote a **sparsity pattern**. The set of matrices which obey the sparsity constraint determined by $\mathbf{E}$ is defined as
{: .text-justify}

$$
\mathrm{Sparse}(\mathbf{E}) :=\left\{[\mathbf{K}]_{ij}\in\mathbb{R}^{n\times o}: [\mathbf{E}_{ij}] = 0 \implies [\mathbf{K}]_{ij}= 0;\: i= 1,...,n, \:j=1,...,o \right\}.
$$

The command
{% highlight m%}[K,Ppred,Pfilt] = kalmanOneStepLTV(system,E,Pprev){% endhighlight m%} computes the **one-step filter gain** at time instant $k$, that aims at solving the optimization problem
{: .text-justify}

$$
\begin{aligned}
& \underset{\begin{subarray}{c}\mathbf{K}(k)\in \mathbb{R}^{n\times o} \end{subarray}}{\text{minimize}}
& & \mathrm{tr}(\mathbf{P}(k|k)) \\
& \text{subject to}
& & \mathbf{K}(k) \in \mathrm{Sparse}(\mathbf{E})\:,
\end{aligned}
$$

where $\mathbf{P}(k\|k)$ is the **estimation error covariance matrix**, using the **one-step method** proposed in [[Section 3, 1]](#references).
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
-  ```system``` : $$1\times 4$$ cell array of the time-varying dynamics matrices of the LTV system, <i>i.e.</i>,
    - ```system{i,1}```: $$\mathbf{A}(k)$$
    - ```system{i,2}```: $$\mathbf{C}(k)$$
    - ```system{i,3}```: $$\mathbf{Q}(k)$$
    - ```system{i,4}```: $$\mathbf{R}(k)$$
-  ```E``` : sparsity pattern $\mathbf{E}$
-  ```Pprev``` : predicted estimation error covariance matrix, <i>i.e.</i>, $\mathbf{P}(k\|k-1)$

***

# Output Arguments

- ```K```: filter gain matrix $\mathbf{K}(k)$
- ```Ppred```: predicted estimation error covariance matrix $\mathbf{P}(k+1\|k)$
- ```Pfilt```: filtered estimation error covariance matrix $\mathbf{P}(k\|k)$

***

# Examples

See [One-step Kalman filter tutorial for LTV systems](/tutorials/kalmanOneStepLTV/) for a tutorial.

***

# References
[1] <a href="" target="_blank">[Not published yet]</a>
{: .text-justify}

[2] <a href="https://doi.org/10.3390/math9131497" target="_blank">Pedroso, L.; Batista, P. Efficient Algorithm for the Computation of the Solution to a Sparse Matrix Equation in Distributed Control Theory. Mathematics 2021, 9, 1497. https://doi.org/10.3390/math9131497</a>
{: .text-justify}
