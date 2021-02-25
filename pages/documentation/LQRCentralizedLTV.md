---
title: LQRCentralizedLTV
showTitle: true
excerpt: Documentation for function LQRCentralizedLTV.
permalink: /documentation/LQRCentralizedLTV/
layout: single
classes: wide
sidebar:
  title: "LQRCentralizedLTV"
  nav: sidebar-LQRCentralizedLTV
tags:
    - control
    - ltv
    - documentation
date: "2021-02-24"
last_modified_at: "2021-02-24"
---
# Sintax
~~~m
[K,P] = LQRCentralizedLTV(system,T)
[K,P] = LQRCentralizedLTV(system,T,opts)
~~~
***

# Description
Consider a generic **LTV system** of the form

$$
\mathbf{x}(k+1)=\mathbf{A}(k)\mathbf{x}(k)+\mathbf{B}(k)\mathbf{u}(k)\;,
$$

where $\mathbf{x}(k)\in\mathbb{R}^{n}$ is the state vector, $\mathbf{u}(k)\in \mathbb{R}^{m}$ is the input vector, and $\mathbf{A}(k)$ and $\mathbf{B}(k)$ are time-varying matrices of appropriate dimensions.
{: .text-justify}

Consider a standard **LQR regulator**

$$
\mathbf{u}(k) = -\mathbf{K}(k)\mathbf{x}(k)\:,
$$

where $\mathbf{K}(k)\in\mathbb{R}^{m\times n}$ is the **regulator gain**. Consider a regulator **cost function**, over a given finite window $ \\{ k,\ldots,k+T \\}$, where $T\in\mathbb{N}$, as
{: .text-justify}

$$
	J(k) := \mathbf{x}^T(k+T)\mathbf{Q}(k+T)\mathbf{x}(k+T) + \sum_{\tau = k}^{k+T-1} \left(\mathbf{x}^T(\tau)\mathbf{Q}(\tau) \mathbf{x}(\tau)+\mathbf{u}^T(\tau)\mathbf{R}(\tau) \mathbf{u}(\tau)\right) \:.
$$

where $\mathbf{Q}(\tau) \succeq \mathbf{0}\in\mathbb{R}^{n\times n}$ and  $\mathbf{R}(\tau) \succ \mathbf{0}\in\mathbb{R}^{m\times m}$ are the time-varying weighing matrices of the state vector and control action, respectively.
{: .text-justify}
The commands
{% highlight m%}[K,P] = LQRCentralizedLTV(system,T)
[K,P] = LQRCentralizedLTV(system,T,opts){% endhighlight m%} compute the well-known **sequence of centralized LQR gains** that solves the optimization problem
{: .text-justify}

$$
\begin{aligned}
	& \underset{\begin{subarray}{c}\mathbf{K}(\tau)\in \mathbb{R}^{m\times n} \\\tau = k,...,k+T-1 \end{subarray}}{\text{minimize}}
	& & J(k)\:.
	\end{aligned}
$$

***

# Input arguments
### Required
-  ```system``` : $$(T+1)\times 4$$ cell array of the time-varying dynamics matrices of the LTV system, <i>i.e.</i>,
    - ```system{i,1}```: $$\mathbf{A}(k+i-1),\: i = 1,\ldots, T$$
    - ```system{i,2}```: $$\mathbf{B}(k+i-1),\: i = 1,\ldots, T$$
    - ```system{i,3}```: $$\mathbf{Q}(k+i-1),\: i = 1,\ldots, T+1$$
    - ```system{i,4}```: $$\mathbf{R}(k+i-1),\: i = 1,\ldots, T$$

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
### Synthetic system
To open this example execute the following command in the MATLAB command window
~~~m
open LQRCentralizedLTVTutorial
~~~

Generate synthetic time-varying system matrices $\mathbf{A}(k)$, $\mathbf{B}(k)$, $\mathbf{Q}(k)$, $\mathbf{R}(k)$

~~~m
T = 50;
n = 5;
m = 3;
rng(1); % Pseudo-random seed for consistency
% Alternatively comment out rng() to generate a random system
% Do not forget to readjust the synthesys parameters of the methods
system = cell(T+1,4);
for i = 1:T+1
    if i == 1
        system{i,1} = rand(n,n)-0.5;
        system{i,2} = rand(n,m)-0.5;
    elseif i == T+1
        system{i,1} = nan;
        system{i,2} = nan;
        system{i,3} = rand(n,n)-0.5;
        system{i,3} = system{i,3}*system{i,3}';
        system{i,4} = nan;
        continue;
    else % Generate time-varying dynamics preventing erratic behaviour
        system{i,1} = system{i-1,1}+(1/4)*(rand(n,n)-0.5);
        system{i,2} = system{i-1,2}+(1/4)*(rand(n,m)-0.5);
    end
    system{i,3} = rand(n,n)-0.5;
    system{i,3} = system{i,3}*system{i,3}';
    system{i,4} = rand(m,m)-0.5;
    system{i,4} = system{i,4}*system{i,4}';
end
~~~

Synthesize regulator gain using the centralized method (with some optional parameters)
~~~m
opts.verbose = true;
[K,P] = LQRCentralizedLTV(system,T,opts);
~~~

~~~text
----------------------------------------------------------------------------------
Running centralized algorithm with T = 50.
----------------------------------------------------------------------------------
~~~
