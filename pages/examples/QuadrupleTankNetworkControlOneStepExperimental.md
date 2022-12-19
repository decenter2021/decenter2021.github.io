---
layout: single
title: "Experimental quadruple-tank network decentralized control"
showTitle: true
excerpt: "Experimental application of decentralized control to the quadruple-tank network, using the one-step method."
permalink: /examples/QuadrupleTankNetworkControlOneStepExperimental/
classes: wide
sidebar:
  title: "Examples"
  nav: sidebar-examples
header:
  teaser: "/assets/img/ExperimentalQuadrupleTankNetworkControlOneStep_teaser.png"
tags:
  - example
  - mpc
  - ltv
  - nonlinear
  - one-step
  - control
  - experimental
date: "2022-05-22"
last_modified_at: "2022-09-15"
---
{{page.excerpt}}

This example corresponds to the experimental implementation of the numeric example ["Decentralized control of nonlinear quadruple-tank network"](/examples/QuadrupleTankNetworkControlOneStep/).
{: .text-justify}

***

## The setup

This experimental example is performed on the **Reproducible Low-cost Flexible Quadruple-Tank Process Experimental Setup for Control Educators, Practitioners, and Researchers** proposed in [[1]](#references). This setup is very **easily reproducible**, costs under **650€**, and takes roughly **4 hours to assemble**. The CAD models, technical drawings, wiring schematics, and software are **open-source** and available <a target= "blank" href="https://github.com/decenter2021/quadruple-tank-setup">here</a>. A CAD model of the setup, as well as a physical implementation, can be seen below.
{: .text-justify}

<div class="row">
<div class="column">
  <p style="text-align:center;"><img src="https://user-images.githubusercontent.com/40807922/163567003-0c99b331-ad43-42da-af84-5637b5629d2c.png" width="52%" /></p>
</div>
  <div class="column">
    <p style="text-align:center;"><img src="https://user-images.githubusercontent.com/40807922/163566465-bf311f1e-8831-4145-b541-65d9046de89c.png" width="100%" /></p>
  </div>
</div>

A **user-friendly** dedicated **MATLAB/Simulink** interface is openly available as well. It allows for a	seamless shift between a **numeric simulation** and the **interface with the experimental plant**. The simulink model design for this example is shown below. It is available <a target="blank" href="https://github.com/decenter2021/quadruple-tank-setup/tree/master/simulink/examples/decentralized_LQR">here</a>.
{: .text-justify}


<p style="text-align:center;"><img src="/assets/img/ExperimentalQuadrupleTankNetworkControlOneStepSimulink.png" width="70%"></p>

***
## Experiment

In this example, the **decentralized tracker** put forward in [[2]](#references) for **linear time-varying systems** is experimentally applied to the quadruple-tank process. To employ this method, the process dynamics are approximated by an LTV system, **linearizing** and **discretizing** its dynamics about successive equilibrium points. **Impulsive and constant disturbances** are introduced, starting at $$t = 300 \mathrm{s}$$ until the end of the simulation. A recording of the experimental simulation is shown below.
{: .text-justify}


<iframe width="560" height="315" src="https://www.youtube.com/embed/NA25sSz-3jE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


***
## Results

The **experimental** results, which are analysed carefully in [[1]](#references), are shown below.

<div class="row">
  <div class="column">
    <img src="/assets/img/ExperimentalQuadrupleTankNetworkControlOneStep_DLQR_exp_h-eps-converted-to.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/ExperimentalQuadrupleTankNetworkControlOneStep_DLQR_exp_u-eps-converted-to.svg" style="width:100%">
  </div>
</div>


The **numeric simulation** results are shown below.

<div class="row">
  <div class="column">
    <img src="/assets/img/ExperimentalQuadrupleTankNetworkControlOneStep_DLQR_sim_h-eps-converted-to.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/ExperimentalQuadrupleTankNetworkControlOneStep_DLQR_sim_u-eps-converted-to.svg" style="width:100%">
  </div>
</div>


# References
[1] {% for paper in site.data.references%}{% if paper.key == "Pedroso2022Reproducible" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}

[2] {% for paper in site.data.references%}{% if paper.key == "Pedroso2021LQROSLTV" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}
