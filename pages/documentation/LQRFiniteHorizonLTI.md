---
title: LQRFiniteHorizonLTI
showTitle: true
excerpt: Documentation for function LQRFiniteHorizonLTI.
permalink: /documentation/LQRFiniteHorizonLTI/
layout: single
classes: wide
sidebar:
  title: "LQRFiniteHorizonLTI"
  nav: sidebar-LQRFiniteHorizonLTI
tags:
  - control
  - lti
  - finite-horizon
  - documentation
date: "2021-02-14"
last_modified_at: "2021-02-22"
---

# Sintax
~~~m
[K_inf,P_inf] = LQRFiniteHorizonLTI(A,B,Q,R,E)
[K_inf,P_inf] = LQRFiniteHorizonLTI(A,B,Q,R,E,opts)
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
J(k) = \mathbf{x}^T(k)\mathbf{P}(k)\mathbf{x}(k)
$$

Suppose that $\mathbf{x}(0)$ is sampled from a normal distribution with zero mean and covariance $\alpha \mathbf{I}$, where $\alpha \in \mathbb{R}^{+}$ and that a stabilizing steady-state regulator gain $\mathbf{K}_{\infty}$ is used. Then,
{: .text-justify}

$$
\mathrm{E}[J(0)] = \alpha \mathrm{tr}(\mathbf{P}_{\infty})\:,
$$

where $\mathbf{P}_{\infty}$ is the steady-state value of $\mathbf{P}(k)$ as $k \to \infty$, as shown in [[Section 4, 1]](#references).
{: .text-justify}

Let matrix $\mathbf{E}  \in\mathbb{R}^{m\times n}$ denote a **sparsity pattern**. The set of matrices which obey the sparsity constraint determined by $\mathbf{E}$ is defined as
{: .text-justify}

$$
\mathrm{Sparse}(\mathbf{E}) :=\left\{[\mathbf{K}]_{ij}\in\mathbb{R}^{m\times n}: [\mathbf{E}_{ij}] = 0 \implies [\mathbf{K}]_{ij}= 0;\: i= 1,...,m, \:j=1,...,n \right\}.
$$

The commands
{% highlight m%}[K_inf,P_inf] = LQRFiniteHorizonLTI(A,B,Q,R,E)
[K_inf,P_inf] = LQRFiniteHorizonLTI(A,B,Q,R,E,opts){% endhighlight m%} compute the **steady-state decentralized gain** that aims at solving the optimization problem
{: .text-justify}

$$
\begin{aligned}
& \underset{\begin{subarray}{c}\mathbf{K}_{\infty}\in \mathbb{R}^{m\times n} \end{subarray}}{\text{minimize}}
& & \mathrm{tr}(\mathbf{P}_{\infty}) \\
& \text{subject to}
& & \mathbf{K}_{\infty} \in \mathrm{Sparse}(\mathbf{E})\:,
\end{aligned}
$$

using the **finite-horizon method** proposed in [[Section 4, 1]](#references).
{: .text-justify}

***

# Input arguments
### Required
-  ```A``` : matrix $\mathbf{A}$ of the dynamics of the LTI system
-  ```B``` : matrix $\mathbf{B}$ of the dynamics of the LTI system
-  ```Q``` : state weighting matrix, $\mathbf{Q} \succeq \mathbf{0}$
-  ```R``` : control action weighting matrix, $\mathbf{R} \succeq \mathbf{0}$
-  ```E``` : sparsity pattern $\mathbf{E}$

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

See [Regulator design using the finite-horizon method](/tutorials/LQRFiniteHorizonLTI/) for an example.

***

# References
[1] <a href="https://onlinelibrary.wiley.com/doi/abs/10.1002/oca.2669" target="_blank">Viegas D, Batista P, Oliveira P, Silvestre C. Distributed controller design and performance optimization for discrete-time linear systems. Optim Control Appl Meth. 2020;1–18. https://doi.org/10.1002/oca.2669</a>
{: .text-justify}
