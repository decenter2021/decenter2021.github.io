---
layout: single
title: "Distributed Decentralized EKF for Satellite Mega-Constellations"
showTitle: true
excerpt: "Application of a novel distributed decentralized EKF framework to the navigation of a shell of the Starlink mega-constellation."
permalink: /examples/DDEKFStarlinkConstellation/
classes: wide
sidebar:
  title: "Examples"
  nav: sidebar-examples
header:
  teaser: "/assets/img/DDEKF_StarlinkConstellation_ground_track-teaser.png"
tags:
  - example
  - ltv
  - nonlinear
  - one-step
  - estimation
  - ekf
date: "2022-05-20"
last_modified_at: "2022-05-20"
---

Application of a novel distributed decentralized EKF framework to the navigation of very large-scale constellations of satellites, illustrated in particular for a shell of the Starlink mega-constellation.
{: .text-justify}

***

A novel distributed decentralized EKF framework for very large-scale networks was proposed in [[1]](#references). In this example, this solution is applied to the satellite mega-constellations navigation problem. An illustrative mega-constellation of a single shell inspired in the first shell of the Starlink constellation to be deployed is considered. The constellation is a Walker $$53.0º:1584/72/17$$. A snapshot of ground track and inter-satellite links (ISL) of the simulated constellation at 70 TDB seconds since J2000 is depicted below.
{: .text-justify}

<img src="/assets/img/DDEKF_StarlinkConstellation_ground_track.png" style="width:100%">


This example is organized into two parts: i) simulation of the constellations making use of an high-fidelity propagator; and ii) implementation of the EKF to obtain a distributed navigation solution in a decentralized framework.

***

## High fidelity simulation

The realistic nonlinear numeric simulation is computed making use of the high fidelity open-source **TU Delft’s Astrodynamic Toolbox (TUDAT)**. The documentation is available at [https://docs.tudat.space/](https://docs.tudat.space/) and source code at [https://github.com/tudat-team/tudat-bundle/](https://github.com/tudat-team/tudat-bundle/).  The orbit propagation of the satellites of the constellation accounts for several **perturbations**. The parameters that fully characterize the constellation, as well as the perturbations considered in the simulation, are detailed in [[1]](#references).
{: .text-justify}

The **TUDAT application source-code** can be found at

<tt>Examples/DistributedDecentralizedEKFStarlinkConstellation/tudatSimulation</tt>.

The <tt>.mat</tt> **output of a simulation** of roughly 10 full orbital periods can be **downloaded** (3.9 GB) <a target = 'blank' href = "https://drive.google.com/uc?export=download&id=15rWHJCAoboV3evRALkodtb09Jk_irWC4">here</a>.

The **TUDAT application source-code** consists of a C++ script that simulates the orbital dynamics of a constellation of satellites. It also establishes a UDP connection with a server running on a MATLAB instance to obtain thruster actuation feedback. For more details on how to setup the simulation and on the intricacies of the thruster actuation feedback, see the <a target = 'blank' href = "">dedicated GitHub repository (available soon)</a>.
{: .text-justify}

***

## Implementation of distributed decentralized EKF

To open this example execute the following command in the MATLAB command window
{: .text-justify}
~~~m
open DDEKFStarlinkConstellation
~~~






<div class="row">
  <div class="column">
    <img src="/assets/img/TrafficNetworkOS_occ.svg" style="width:100%">
  </div>
  <div class="column">
    <img src="/assets/img/TrafficNetworkOS_stage_time.svg" style="width:100%">
  </div>
</div>

{: .text-justify}


# References
[1] *Not published yet*
