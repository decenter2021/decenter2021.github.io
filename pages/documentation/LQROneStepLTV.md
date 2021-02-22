---
title: LQROneStepLTV
showTitle: true
permalink: /documentation/LQROneStepLTV/
layout: single
classes: wide
sidebar:
  title: "LQROneStepLTV"
  nav: sidebar-LQROneStepLTV

tags:
    - control
    - LTV
    - one-step
    - documentation
date: "2021-02-14"
last_modified_at: "2021-02-22"
---
# Sintax
~~~m
[K,P] = LQROneStepLTV(system,T,E)
[K,P] = LQROneStepLTV(system,T,E,opts)
~~~
***

# Description
Consider a generic **LTV system** of the form

$$
\mathbf{x}(k+1)=\mathbf{A}(k)\mathbf{x}(k)+\mathbf{B}(k)\mathbf{u}(k)\;
$$

where $\mathbf{x}(k)\in\mathbb{R}^{n}$ is the state vector, $\mathbf{u}(k)\in \mathbb{R}^{m}$ is the input vector, and $\mathbf{A}(k)$ and $\mathbf{B}(k)$ are time-varying matrices of appropriate dimensions.
{: .text-justify}

Consider a standard **LQR regulator**

$$
\mathbf{u}(k) = -\mathbf{K}(k)\mathbf{x}(k)\:,
$$

where $\mathbf{K}(k)\in\mathbb{R}^{m\times n}$ is the **regulator gain**. Consider a regulator **cost function**
{: .text-justify}

$$
J_{\infty}(k) = \sum_{\tau=k}^{\infty}\left(\mathbf{x}^T(\tau)\mathbf{Q}(\tau)\mathbf{x}(\tau)+\mathbf{u}^T(\tau)\mathbf{R}(\tau)\mathbf{u}(\tau)\right)\:,
$$

where $\mathbf{Q}(\tau) \succeq \mathbf{0}\in\mathbb{R}^{n\times n}$ and  $\mathbf{R}(\tau) \succ \mathbf{0}\in\mathbb{R}^{m\times m}$ are the time-varying weighing matrices of the state vector and control action, respectively.
{: .text-justify}
Let matrix $\mathbf{E}  \in\mathbb{R}^{m\times n}$ denote a **sparsity pattern**. The set of matrices which obey the sparsity constraint determined by $\mathbf{E}$ is defined as
{: .text-justify}

$$
\mathrm{Sparse}(\mathbf{E}) :=\left\{[\mathbf{K}]_{ij}\in\mathbb{R}^{m\times n}: [\mathbf{E}_{ij}] = 0 \implies [\mathbf{K}]_{ij}= 0;\: i= 1,...,m, \:j=1,...,n \right\}.
$$

The one-step method computes a **sequence of decentralized gains** that aims at solving the optimization problem
{: .text-justify}

$$
\begin{aligned}
& \underset{\begin{subarray}{c}\mathbf{K}(i)\in \mathbb{R}^{m\times n},\: i\in \mathbb{N}_0 \end{subarray}}{\text{minimize}}
& & J_{\infty}(0) \\
& \text{subject to}
& & \mathbf{K}(i) \in \mathrm{Sparse}(\mathbf{E}),\: i\in \mathbb{N}_0\:.
\end{aligned}
$$

Define the finite-horizon performance index, over a given finite window $ \\{ k,\ldots,k+T \\} $, where $T\in\mathbb{N}$, as

$$
	J(k) := \mathbf{x}^T(k+T)\mathbf{Q}(k+T)\mathbf{x}(k+T) + \sum_{\tau = k}^{k+T-1} \left(\mathbf{x}^T(\tau)\mathbf{Q}(\tau) \mathbf{x}(\tau)+\mathbf{u}^T(\tau)\mathbf{R}(\tau) \mathbf{u}(\tau)\right) \:.
$$

The one-sep method computes the close-form solution to the relaxed finite-window optimization problem

$$
\begin{aligned}
	& \underset{\begin{subarray}{c}\mathbf{K}(\tau)\in \mathbb{R}^{m\times n} \\\tau = k,...,k+T-1 \end{subarray}}{\text{minimize}}
	& & J(k)\\
	& \text{subject to}
	& & \mathbf{K}(\tau) \in \mathrm{Sparse}(\mathbf{E}),\: \tau = k,...,k+T-1\:,
	\end{aligned}
$$

using the **one-step method** procedure proposed in [[Section 3, 1]](#references). It is shown in [[1]](#references) that there is a matrix $\mathbf{P}(\tau) \succ \mathbf{0}\in\mathbb{R}^{n\times n},\tau = k,\ldots,k+T$, such that
{: .text-justify}

$$
J(\tau) = \mathbf{x}^T(\tau)\mathbf{P}(\tau)\mathbf{x}(\tau)\:.
$$

The commands
{% highlight m%}[K,P] = LQROneStepLTV(system,T,E)
[K,P] = LQROneStepLTV(system,T,E,opts){% endhighlight m%} compute the **sequence of decentralized gain** that solves the relaxed problem, as well as the sequence of matrices $\mathbf{P}(\tau).$
{: .text-justify}

***

# Input arguments
### Required
-  ```system``` : $$(T+1)\times 4$$ cell array of the time-varying dynamics matrices of the LTV system, <i>i.e.</i>,
    - ```system{i,1}```: $$\mathbf{A}(k+i-1),\: i = 1,\ldots, T$$
    - ```system{i,2}```: $$\mathbf{B}(k+i-1),\: i = 1,\ldots, T$$
    - ```system{i,3}```: $$\mathbf{Q}(k+i-1),\: i = 1,\ldots, T+1$$
    - ```system{i,4}```: $$\mathbf{R}(k+i-1),\: i = 1,\ldots, T$$

-  ```E``` : sparsity pattern $\mathbf{E}$
-  ```T``` : length of the finite window

### Optional
- ```opts```: struct of **optional** arguments (**assumes default** value for each parameter which is not assigned by the user)
  - ```verbose```: display algorithm status messages (default: ```opts.verbose = false```)

***

# Output Arguments
- ```K```: $$T\times 1$$ cell array of the sequence of decentralized LQR gains $$\mathbf{K}(\tau),\: \tau = k,\ldots,k+T-1$$, <i>i.e.</i>,
    - ```K{i,1}```: $$\mathbf{K}(k+i-1)$$

- ```P```: $$(T+1)\times 1$$ cell array of the sequence of matrices $$\mathbf{P}(\tau),\: \tau = k,\ldots,k+T$$, <i>i.e.</i>,
    - ```P{i,1}```: $$\mathbf{P}(k+i-1)$$

***

# Examples

See [Regulator design using the one-step method](/tutorials/LQROneStepLTV/) for a tutorial.

***

# References
[1] <a href="" target="_blank">L. Pedroso, and P. Batista (xxx), Discrete-time decentralized linear quadratic control for linear time-varying systems, Int J Robust Nonlinear Control, xxx;xx:x–x. <i>[Submitted to journal]</i></a>
{: .text-justify}
