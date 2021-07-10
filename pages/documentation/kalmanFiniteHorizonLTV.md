---
title: kalmanFiniteHorizonLTV
showTitle: true
excerpt: Documentation for function kalmanFiniteHorizonLTV.
permalink: /documentation/kalmanFiniteHorizonLTV/
layout: single
classes: wide
sidebar:
  title: "kalmanFiniteHorizonLTV"
  nav: sidebar-kalmanFiniteHorizonLTV
tags:
  - estimation
  - ltv
  - finite-horizon
  - documentation
date: "2021-07-10"
last_modified_at: "2021-07-10"
---
# Sintax
~~~m
[K,P] = kalmanFiniteHorizonLTV(system,E,T,P0)
[K,P] = kalmanFiniteHorizonLTV(system,E,T,P0,opts)
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

The commands
{% highlight m%}[K,P] = kalmanFiniteHorizonLTV(system,E,T,P0)
[K,P] = kalmanFiniteHorizonLTV(system,E,T,P0,opts){% endhighlight m%} compute a window of **finite-horizon filter gains**, that aims at solving the optimization problem
{: .text-justify}

$$
\begin{aligned}
& \underset{\begin{subarray}{c}\mathbf{K}(i) \in \mathbb{R}^{n\times o}\\i = 1,...,T \end{subarray}}{\text{minimize}}
& & \sum_{k=1}^{T}\mathrm{tr}(\mathbf{P}(k|k)) \\
& \text{subject to}
& & \mathbf{K}(i) \in \mathrm{Sparse}(\mathbf{E})\:, i = 1,...,T
\end{aligned}
$$

where $\mathbf{P}(k\|k)$ is the **estimation error covariance matrix** at time instant $k$, using the **finite-horizon method** proposed in [[Section 4, 1]](#references).
{: .text-justify}

***

# Computational complexity
The finite-horizon optimization problem is solved using the efficient sparse equation solver proposed in [[2]](#references). See [sparseEqSolver](/documentation/sparseEqSolver/) for the implementation of the solver.
{: .text-justify}

Define the set $\chi$ of integer pairs of the form $(i,j)$ to index the nonzero entries of $\mathbf{E}$ as
{: .text-justify}

$$
\begin{cases}
(i,j) \in \chi &\;,\;\left[\mathbf{E}\right]_{i,j} \neq 0\\
(i,j) \notin \chi &\;,\;\text{otherwise}
\end{cases}, i = 1,...,n,\: j = 1,...,o\:.
$$

It is shown in [[2]](#references) that each iteration of the algorithm requires $\mathcal{O}(\|\chi\|^3)$ floating-point operations, where $\|\chi\|$ denotes the cardinality of set $\chi$. In the field of distributed estimation and control theory, $\|\chi\|$ is usually given by $\|\chi\| \approx cn$, where $c\in \mathbb{N}$ is a constant. It, thus, follows that each iteration requires $\mathcal{O}(n^3)$ floating-point operations.
{: .text-justify}

***

# Input arguments
### Required
-  ```system``` : $$T\times 4$$ cell array of the time-varying dynamics matrices of the LTV system, <i>i.e.</i>,
    - ```system{i,1}```: $$\mathbf{A}(i)$$, $i = 1,...,T$
    - ```system{i,2}```: $$\mathbf{C}(i)$$, $i = 1,...,T$
    - ```system{i,3}```: $$\mathbf{Q}(i)$$, $i = 1,...,T$
    - ```system{i,4}```: $$\mathbf{R}(i)$$, $i = 1,...,T$
-  ```E``` : sparsity pattern $\mathbf{E}$
-  ```T``` : finite window length
-  ```Pprev``` : predicted estimation error covariance matrix, <i>i.e.</i>, $\mathbf{P}(1\|0)$

### Optional
- ```opts```: struct of **optional** arguments (**assumes default** value for each parameter which is not assigned by the user)
  - ```epsl```: minimum relative improvement on the objective function of the optimization problem (default: ```opts.epsl = 1e-5```)
  {: .text-justify}
  - ```maxOLIt```: maximum number of outer loop iterations to run until convergence (default: ```opts.maxOLIt = 100```)
  {: .text-justify}
  - ```verbose```: display algorithm status messages (default: ```opts.verbose = false```)

***

# Output Arguments

- ```K```: $$T\times 1$$ cell array of filter gain matrices $\mathbf{K}(i)$, $i = 1,...,T$
- ```P```: $$T\times 1$$ cell array of filtered estimation error covariance matrices $\mathbf{P}(i\|i)$,  $i = 1,...,T$

***

# Examples

See [Finite-horizon Kalman filter tutorial for LTV systems](/tutorials/kalmanFiniteHorizonLTV/) for a tutorial.

***

# References
[1] <a href="" target="_blank">[Not published yet]</a>
{: .text-justify}

[2] <a href="https://doi.org/10.3390/math9131497" target="_blank">Pedroso, L.; Batista, P. Efficient Algorithm for the Computation of the Solution to a Sparse Matrix Equation in Distributed Control Theory. Mathematics 2021, 9, 1497. https://doi.org/10.3390/math9131497</a>
{: .text-justify}
