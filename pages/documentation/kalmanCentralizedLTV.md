---
title: kalmanCentralizedLTV
showTitle: true
excerpt: Documentation for function kalmanCentralizedLTV.
permalink: /documentation/kalmanCentralizedLTV/
layout: single
classes: wide
sidebar:
  title: "kalmanCentralizedLTV"
  nav: sidebar-kalmanCentralizedLTV
tags:
  - estimation
  - ltv
  - documentation
date: "2021-07-10"
last_modified_at: "2021-07-10"
---
# Sintax
~~~m
[K,Ppred,Pfilt] = kalmanCentralizedLTV(system,Pprev)
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

The command
{% highlight m%}[K,Ppred,Pfilt] = kalmanCentralizedLTV(system,Pprev){% endhighlight m%} computes the **centralized kalman filter gain** at time instant $k$.
{: .text-justify}

***

# Input arguments
### Required
-  ```system``` : $$1\times 4$$ cell array of the time-varying dynamics matrices of the LTV system, <i>i.e.</i>,
    - ```system{i,1}```: $$\mathbf{A}(k)$$
    - ```system{i,2}```: $$\mathbf{C}(k)$$
    - ```system{i,3}```: $$\mathbf{Q}(k)$$
    - ```system{i,4}```: $$\mathbf{R}(k)$$

-  ```Pprev``` : predicted estimation error covariance matrix, <i>i.e.</i>, $\mathbf{P}(k\|k-1)$

***

# Output Arguments

- ```K```: filter gain matrix $\mathbf{K}(k)$
- ```Ppred```: predicted estimation error covariance matrix $\mathbf{P}(k+1\|k)$
- ```Pfilt```: filtered estimation error covariance matrix $\mathbf{P}(k\|k)$
