---
layout: single
title: "Decentralized control of nonlinear N-tank network"
showTitle: true
excerpt: "Application of decentralized control to the N-tank network, using the one-step method."
permalink: /examples/NTanksNetworkControlOneStep/
classes: wide
sidebar:
  title: "Examples"
  nav: sidebar-examples
header:
  teaser: "/assets/img/NTanksNetworkControlOneStepSchemeN.pdf"
tags:
  - example
  - mpc
  - ltv
  - nonlinear
  - one-step
  - control
date: "2021-02-24"
last_modified_at: "2021-02-24"
---
{{page.excerpt}}

***

The quadruple-tank network presented in [[1]](#references), whose MATLAB code is presented in [this example](/examples/QuadrupleTankNetworkControlOneStep/), can be generalized to N tanks. Consider $N$ interconnected tanks, where $N$ is an even integer. The water level of tank $i$ is denoted by $h_i$. The network is actuated by $N/2$ pumps, which are controlled by the lower tanks, whose inputs are denoted by $u_i$ for $i=1,...,N/2$, in accordance with the schematic. Each pump is connected to a three-way valve that regulates the fraction of the flow, held constant, that goes to each of the tanks supplied by the pump. Each tank has a sensor, which measures its water level. Making use of mass balances and Bernoulli's law, the system dynamics, in the absence of noise, are given by
{: .text-justify}

$$
	\begin{cases}
	A_i\dot{h}_i(t) = -a_i\sqrt{2gh_i(t)}+a_{\frac{N}{2}+i}\sqrt{2gh_{\frac{N}{2}+i}(t)}+\gamma_ik_iu_i(t), \:\: i = 1,...,N/2\\
	A_i\dot{h}_i(t) = -a_i\sqrt{2gh_i(t)}+(1-\gamma_{i-\frac{N}{2}-1})k_{i-\frac{N}{2}-1}u_{i-\frac{N}{2}-1}(t), \:\: i = \frac{N}{2}+2,...,N\\
	A_{\frac{N}{2}+1}\dot{h}_{\frac{N}{2}+1}(t) = -a_{\frac{N}{2}+1}\sqrt{2gh_{\frac{N}{2}+1}(t)}+(1-\gamma_{\frac{N}{2}})k_{\frac{N}{2}}u_{\frac{N}{2}}(t)
	\end{cases}\:,
$$

where $A_i$ and $a_i$ are the cross sections of tank $i$ and of its outlet hole, respectively; the constant $\gamma_i$ represents the fraction of the flow that passes through the valve $i$ to the lower tanks; $k_i$ is the constant of proportionality between the mass flow and the input of pump $i$; and $g$ denotes the acceleration of gravity. Furthermore, the input of each pump is subject to a hard constraint $u_i\in [0,u^{\mathrm{sat}}]$, where $u^{\mathrm{sat}} \in \mathbb{R}^+$.
{: .text-justify}

{:refdef: style="display: block; margin-left: auto; margin-right: auto; width: 70%;"}
![image-title-here](/assets/img/NTanksNetworkControlOneStepSchemeN.pdf){:class="img-responsive"}
{: refdef}

This network is presented to show the scalability of the one-step method. The same control approach as in the [example for the quadruple-tank network](/examples/QuadrupleTankNetworkControlOneStep/) is used, which is described in [[1]](#references). The simulation of the N-tank network is analogous to the quadruple tank network. To open this example execute the following command in the MATLAB command window
{: .text-justify}
~~~m
open NTanksNetworkControlOneStep
~~~
and set the number of tanks to an even integer number in the field ```N```of struct ```cte``` in private function ```getConstantsNTankNetwork```, as exemplified below
{: .text-justify}

~~~m
%% getConstantsNTankNetwork - Description
% This function outputs a struct of constants of the model dynamics.
% Input:    -flagEqMatrices: only if true compute equilibrium matrices
% Output:   -Cte: struct with the necessary constants and parameters
function cte = getConstantsNTankNetwork(flagEqMatrices)
% --- Define dimentions of the model ---
cte.N = 40;      % number of tanks

(...)

end
~~~

For $N=40$, the evolution of the water level in some of the tanks is shown below
<div class="row">
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepNh3.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepNh8.svg" style="width:100%">
  </div>
</div>
<div class="row">
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepNh10.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepNh17.svg" style="width:100%">
  </div>
</div>

The evolution of the inputs to some pumps is shown below
<div class="row">
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepNu3.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepNu8.svg" style="width:100%">
  </div>
</div>
<div class="row">
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepNu10.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/NTanksNetworkControlOneStepNu17.svg" style="width:100%">
  </div>
</div>


# References
[1] <a href="" target="_blank">L. Pedroso, and P. Batista (xxx), Discrete-time decentralized linear quadratic control for linear time-varying systems, Int J Robust Nonlinear Control, xxx;xx:x–x. <i>[Submitted to journal]</i></a>
{: .text-justify}
