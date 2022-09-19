---
layout: single
title: "MHE for networks of unconstrained linear systems"
showTitle: true
excerpt: "Moving horizon estimation for unconstrained networks is surprisingly well-performing in a decentralized setting."  
permalink: /examples/MHEMFH/
classes: wide
sidebar:
  title: "MHE Example"
  nav: sidebar-example-mhemfh
header:
  teaser: "/assets/img/big_net.png"
tags:
  - example
  - lti
  - moving-horizon-estimation
  - moving-finite-horizon
  - estimation
date: "2022-09-19"
last_modified_at: "2022-09-19"
---

{{page.excerpt}}

***

# Introduction

In a **centralized** setting, the **Kalman filter** achieves **optimal** performance recursively.\\
In a **decentralized** setting, recursive filters (Luenberger observers), are greedy and compromise long-term performance.\\
Surprisingly, in a decentralized setting, the **moving horizon estimation** (MHE) framework, even for **linear unconstrained** systems achieves **significantly higher performance** than Luenberger-based filters.
{: .text-justify}

In [[1]](#references) a **novel MHE framework** and the **moving finite horizon (MFH)** method are proposed, to achieve higher performance in a decentralized setting with a focus on the **implementation feasibility** for **very large-scale** networks of systems. See [documentation for MHEMovingFiniteHorizon](/documentation/MHEMovingFiniteHorizon/) for more information on the implementation of the MFH method.
{: .text-justify}

This example contains **extensive numerical simulations** to assess the performance of the proposed **MHE filter framework** and of the **MFH method**.

To open this example execute the following command in the MATLAB command window
~~~m
>> open MFH_SimulationResults
~~~

***

# Simulation results


The performance of the **novel MHE framework** and the **moving finite horizon (MFH)** method are compared against:
- Centralized solution
- One-step method [[2, Section 4]](#references) ([documentation](/documentation/kalmanOneStepLTI/))
- Finite-horizon method [[2, Section 5]](#references) ([documentation](/documentation/kalmanFiniteHorizonLTI/))
- PMHE1 method [[3]](#references)
{% comment %}- $\mathcal{H}_2$ norm minimization synthesis using $\mathcal{P}-\mathcal{K}$ iterations inspired in [[3]](#references) and [[4]](#references){% endcomment %}

The one-step and finite-horizon methods are based on a **Lueberger observer** and the PMHE1 method is a state-of-the-art **distributed MHE** solution whose **communication, computational, and memory requirements** are of the same magnitude as those of the MFH method.
{: .text-justify}

All the functions, scripts, randomly generated models (and their generation parameters) **are available in the example folder**.
{: .text-justify}

## Small network

Let's start with a rather small network of $N = 20$ systems. This network can be generated randomly using the function
~~~m
>> rng_seed = 1;
>> generate_random_network(rng_seed);
~~~
~~~text
Global state size: 40
Maximum absolute eigenvalue: 1.17425
~~~
The parameters inside ```generate_random_network``` can be adjusted to modify various parameters, e.g. the strength of the coupling between systems. This system was generated with coupled process noise, which is not supported by the PMHE1 method. For an example with the comparison with the PMHE1 method jump to [Large-scale network](#large-scale-network) and [Large-scale network with weak couplings](#large-scale-network-with-weak-couplings).

We obtain the following topology
<p style="text-align:center;"><img src="/assets/img/mhemfh_small_net.svg" width="80%"></p>


Let's synthesize the centralized and one-step methods with
~~~m
>> synthesis_luenberger(1);
~~~
~~~text
----------------------------------------------------------------------------------
Computing centralized kalman filter with: epsl = 1e-05 | maxIt = 1000.
Convergence reached with: epsl = 1e-05 | maxIt = 1000.
A total of 26 iterations were run.
----------------------------------------------------------------------------------
Elapsed time is 0.006611 seconds.
Trace centralized: 30.130313
----------------------------------------------------------------------------------
Running one-step algorithm with: epsl = 0.0001 | maxIt = 10000.
Convergence reached with: epsl = 0.0001 | maxIt = 10000.
A total of 43 iterations were run.
----------------------------------------------------------------------------------
Elapsed time is 0.020311 seconds.
Trace OS: 58.957642
Trace OS/C: 1.956755
----------------------------------------------------------------------------------
Running one-step algorithm with: epsl = 0.0001 | maxIt = 1000.
Convergence reached with: epsl = 0.0001 | maxIt = 1000.
A total of 108 iterations were run.
----------------------------------------------------------------------------------
Elapsed time is 0.048737 seconds.
Maximum absolute control CL eigenvalue: 0.972328
~~~

**Note:** This routine also computes a decentralized one-step controller just to guarantee that for unstable systems the state does not grow unbounded and causes numerical problems.

Let's synthesize the FH method with
~~~m
>> synthesis_fh(1,1e-4,75,10);
~~~
~~~text
----------------------------------------------------------------------------------
Running finite-horizon algorithm with:
epsl = 0.0001 | W = 75 | maxOLIt = 10 | findWindowSize = false.
Outer-loop initialization.
	Outer-loop initialization iteration: 1.
	(...)
	Outer-loop initialization iteration: 75.
Outer-loop iteration 1.
	Inner-loop iteration 1.
	(...)
	Inner-loop iteration 75.
Outer-loop iteration 1 finished.
Window convergence within 1.77207e-06.
Maximum absolute CL eigenvalue: 0.880973.
Trace: 50.7401
LTI gain convergence within Inf.
Outer-loop iteration 2.
	Inner-loop iteration 1.
	(...)
	Inner-loop iteration 75.
Outer-loop iteration 2 finished.
Window convergence within 1.30285e-07.
Maximum absolute CL eigenvalue: 0.889327.
Trace: 48.3818
LTI gain convergence within 0.0464766.
Outer-loop iteration 3.
	Inner-loop iteration 1.
	(...)
	Inner-loop iteration 75.
Outer-loop iteration 3 finished.
Window convergence within 1.12747e-07.
Maximum absolute CL eigenvalue: 0.888831.
Trace: 48.3003
LTI gain convergence within 0.00168444.
Outer-loop iteration 4.
	Inner-loop iteration 1.
	(...)
	Inner-loop iteration 75.
Outer-loop iteration 4 finished.
Window convergence within 4.34028e-08.
Maximum absolute CL eigenvalue: 0.888993.
Trace: 48.2961
LTI gain convergence within 8.80485e-05.
Convergence reached with: epsl = 0.0001 | W = 75 | maxOLIt = 10
A total of 4 outer loop iterations were run, out of which 100.0% converged within
the specified minimum improvement.
----------------------------------------------------------------------------------
Elapsed time is 9.011818 seconds.
Trace FH: 48.296081
Trace FH/C: 1.602907
Maximum absolute CL eigenvalue: 0.888993
~~~

Finally, let's synthesize the MFH method for various window sizes and with a null covariance initialization
~~~m
>> for W = 1:10
synthesis_mfh(1,1e-4,W,1e4);
end
~~~
~~~text
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 1 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 1 | maxOLIt = 10000
A total of 43 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 58.957642
Trace MFH/C: 1.956755
For W_ss = 1 (tr = 58.9576) elapsed time is 0.15574 seconds.
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 2 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 2 | maxOLIt = 10000
A total of 34 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 52.535678
Trace MFH/C: 1.743615
For W_ss = 2 (tr = 52.5357) elapsed time is 0.189517 seconds.
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 3 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 3 | maxOLIt = 10000
A total of 33 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 48.760071
Trace MFH/C: 1.618306
For W_ss = 3 (tr = 48.7601) elapsed time is 0.249681 seconds.
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 4 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 4 | maxOLIt = 10000
A total of 37 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 46.909798
Trace MFH/C: 1.556897
For W_ss = 4 (tr = 46.9098) elapsed time is 0.632296 seconds.
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 5 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 5 | maxOLIt = 10000
A total of 33 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 45.750378
Trace MFH/C: 1.518417
For W_ss = 5 (tr = 45.7504) elapsed time is 1.62892 seconds.
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 6 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 6 | maxOLIt = 10000
A total of 32 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 45.149851
Trace MFH/C: 1.498486
For W_ss = 6 (tr = 45.1499) elapsed time is 1.36903 seconds.
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 7 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 7 | maxOLIt = 10000
A total of 34 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 44.726017
Trace MFH/C: 1.484419
For W_ss = 7 (tr = 44.726) elapsed time is 1.88515 seconds.
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 8 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 8 | maxOLIt = 10000
A total of 35 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 44.027306
Trace MFH/C: 1.461230
For W_ss = 8 (tr = 44.0273) elapsed time is 2.40886 seconds.
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 9 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 9 | maxOLIt = 10000
A total of 30 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 43.226535
Trace MFH/C: 1.434653
For W_ss = 9 (tr = 43.2265) elapsed time is 2.93729 seconds.
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 10 | maxIt = 10000 .
Convergence reached with: epsl_inf = 0.0001 | W = 10 | maxOLIt = 10000
A total of 29 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 43.079594
Trace MFH/C: 1.429776
For W_ss = 10 (tr = 43.0796) elapsed time is 4.15198 seconds.
~~~

The performance of the different decentralized methods is graphically depicted below
<p style="text-align:center;"><img src="/assets/img/mhemfh_small_net_perf.svg" width="70%"></p>

It is interesting to note that since the MFH method with $W_{ss} = 1$ and the OS method are equivalent, theire performance is the same.

We choose $W_{ss} = 5$ henceforth. It is also interesting to see the evolution of the estimation error covariance throughout the last iteration of the MFH algorithm, i.e. $\mathbf{P}(\tau \mid \tau \mid k),\:\tau = k-W_{ss},\ldots,k$, which is the [third output of the MFH synthesis method](/documentation/MHEMovingFiniteHorizon/).

<p style="text-align:center;"><img src="/assets/img/mhemfh_small_net_ev_trace.svg" width="70%"></p>

Notice that:
- Since **convergence was reached**, the trace at the beginning and end of the window is **equal**
- The estimation performance in the **middle** of the window is **very poor**
- But, it allows for a **very good estimate at the end of the window**
- Because the intermediate estimates **do not compromise future performance**
- This is the principle behind the **performance improvement** of the **MFH method**

Let's assess the performance of the different estimation solutions resorting to Monte Carlo simulations:
~~~m
>> model = 1;
>> parameters = struct('sim_T',100,'MFH_w_ss',5,'mc_it',1e4);
>> methods = [true; true; true; true; false];
>> simulation_mc(model, methods, parameters);
~~~
~~~text
Trace C: 	30.130833 	(ri: 0)
Trace OS/C: 	1.956730 	(ri: 1.61975e-08)
Trace FH/C: 	1.602878 	(ri: 1.53113e-11)
Trace MFH/C: 	1.521114	(ri: 4.31462e-11)
Monte Carlo simulation 1/10000.
(...)
Monte Carlo simulation 9999/10000.
Monte Carlo simulation 10000/10000.
Finished Monte Carlo simulations.
Processing simulations.
Processing method 1/5.
Processing method 2/5.
Processing method 3/5.
Processing method 4/5.
~~~

After 10000 Monte Carlo simulations the evolution estimation error covariance is depicted below
<p style="text-align:center;"><img src="/assets/img/mhemfh_small_net_mc.svg" width="70%"></p>

The steady-state trace of the estimation error covariance matrix is depicted below, which was obtained by averaging the 25 last instants of the MC simulations.

| | C | OS | FH | MFH ($W_{ss} = 5$) |
|:-: | :-: | :-: | :-: | :-: |
| $\mathrm{tr}(\mathbf{P_{\mathrm{MC}}})$ | 30.15 | 59.21 |  48.51 | 46.03 |
| Relative to MFH |  $-34.49\%$ |    $+28.63\%$ |  $+5.39\%$ | $--$  |

**Conlusions**:
- The MFH method **converged** for different window sizes
- The MFH method seems to have **faster synthesis times** than the FH method
- The MFH method **performs better** than state-of-the-art decentralized Luenberger-based filters
- The **principle** behind the performance of the MFH is evident for this example
- The use of the second best performing method (FH) incurs in a **performance penalty** of  $+5.39\%$

***

## Large-scale network

Let's start increase the dimension of the network and generate a new model with $N = 500$ systems. This network was generated randomly using the function
~~~m
>> rng_seed = 40;
>> generate_random_network(rng_seed);
~~~
~~~text
Global state size: 1000
Maximum absolute eigenvalue: 1.21061
~~~

This is one of the systems briefly used in [[1]](#references). We obtain the following topology
<p style="text-align:center;"><img src="/assets/img/mhemfh_large_net.pdf" width="80%"></p>


Let's synthesize the centralized and one-step methods with

~~~m
>> synthesis_luenberger(40);
~~~
~~~text
----------------------------------------------------------------------------------
Computing centralized kalman filter with: epsl = 1e-05 | maxIt = 1000.
Convergence reached with: epsl = 1e-05 | maxIt = 1000.
A total of 42 iterations were run.
----------------------------------------------------------------------------------
Elapsed time is 6.007283 seconds.
Trace centralized: 844.055346
----------------------------------------------------------------------------------
Running one-step algorithm with: epsl = 1e-05 | maxIt = 10000.
Convergence reached with: epsl = 1e-05 | maxIt = 10000.
A total of 138 iterations were run.
----------------------------------------------------------------------------------
Elapsed time is 259.215993 seconds.
Trace OS: 1788.026977
Trace OS/C: 2.118376
----------------------------------------------------------------------------------
Running one-step algorithm with: epsl = 1e-05 | maxIt = 1000.
Convergence reached with: epsl = 1e-05 | maxIt = 1000.
A total of 678 iterations were run.
----------------------------------------------------------------------------------
Elapsed time is 1277.864145 seconds.
Maximum absolute control CL eigenvalue: 0.992713
~~~

Synthesize finite-horizon with

~~~m
>> synthesis_fh(40,1e-2,100,10);
~~~
~~~text
----------------------------------------------------------------------------------
Running finite-horizon algorithm with:
epsl = 0.01 | W = 100 | maxOLIt = 10 | findWindowSize = false.
Outer-loop recomputation.
	Outer-loop recomputation iteration: 1.
	(...)
	Outer-loop recomputation iteration: 100.
Outer-loop iteration 1.
	Inner-loop iteration 1.
	(...)
	Inner-loop iteration 99.
	Inner-loop iteration 100.
Outer-loop iteration 1 finished.
Window convergence within 1.73139e-06.
Trace: 1446.84
LTI gain convergence within Inf.
Outer-loop iteration 2.
	Inner-loop iteration 1.
	(...)
	Inner-loop iteration 99.
	Inner-loop iteration 100.
Outer-loop iteration 2 finished.
Window convergence within 1.31444e-05.
Trace: 1448.36
LTI gain convergence within 0.0578383.
Outer-loop iteration 3.
	Inner-loop iteration 1.
    (...)
	Inner-loop iteration 99.
	Inner-loop iteration 100.
Outer-loop iteration 3 finished.
Window convergence within 2.09779e-05.
Trace: 1434.14
LTI gain convergence within 0.00767651.
Convergence reached with: epsl = 0.01 | W = 100 | maxOLIt = 10
A total of 3 outer loop iterations were run, out of which 100.0% converged within
the specified minimum improvement.
----------------------------------------------------------------------------------
Elapsed time is 13445.180700 seconds.
Trace FH: 1442.817245
Trace FH/C: 1.709387
Maximum absolute CL eigenvalue: 0.954752
~~~

Synthesize MFH gain sequences for window sizes $1--7$

~~~m
>> parfor W = 1:7
    synthesis_mfh(40,1e-4,W,100);
end
~~~
~~~text
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 1 | maxIt = 100 .
Convergence reached with: epsl_inf = 0.0001 | W = 1 | maxOLIt = 100
A total of 97 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 1785.363719
Trace MFH/C: 2.115221
For W_ss = 1 (tr = 1785.36) elapsed time is 809.953 seconds.

----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 2 | maxIt = 100 .
Convergence reached with: epsl_inf = 0.0001 | W = 2 | maxOLIt = 100
A total of 84 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 1608.731635
Trace MFH/C: 1.905955
For W_ss = 2 (tr = 1608.73) elapsed time is 1894.05 seconds.

----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 3 | maxIt = 100 .
Convergence reached with: epsl_inf = 0.0001 | W = 3 | maxOLIt = 100
A total of 69 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 1486.333674
Trace MFH/C: 1.760943
For W_ss = 3 (tr = 1486.33) elapsed time is 3636.26 seconds.

----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 4 | maxIt = 100 .
Convergence reached with: epsl_inf = 0.0001 | W = 4 | maxOLIt = 100
A total of 51 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 1385.031524
Trace MFH/C: 1.640925
For W_ss = 4 (tr = 1385.03) elapsed time is 7772.36 seconds.

----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 5 | maxIt = 100 .
Convergence reached with: epsl_inf = 0.0001 | W = 5 | maxOLIt = 100
A total of 59 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 1340.934405
Trace MFH/C: 1.588681
For W_ss = 5 (tr = 1340.93) elapsed time is 12913.6 seconds.

----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 6 | maxIt = 100 .
Convergence reached with: epsl_inf = 0.0001 | W = 6 | maxOLIt = 100
A total of 59 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 1311.597588
Trace MFH/C: 1.553924
For W_ss = 6 (tr = 1311.6) elapsed time is 20756.7 seconds.

----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 7 | maxIt = 100 .
Convergence reached with: epsl_inf = 0.0001 | W = 7 | maxOLIt = 100
A total of 57 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 1281.632054
Trace MFH/C: 1.518422
For W_ss = 7 (tr = 1281.63) elapsed time is 27268.1 seconds.
~~~


It is very important to note that the projected performance that is output by the implementation of the methods **may have a lower precision** than the displayed significant digits. It depends on the convergence criteria chosen for each method. One can compute the precise projected performance of the synthesis by making use, for instance, of the initial projection performed in ```simulation_mc```. For the MFH synthesis, with $\mathrm{tr}(\mathbf{P}_{\infty}^{\mathrm{Cent}}) = 844.1$, we get

| $W_{ss}$ | | 2 | 3 | 4 | 5 | 6 | 7 |
|:- |:-  | :-: | :-: | :-: | :-: | :-: | :-: |
| $\text{tr}(\mathbf{P}_{\infty})$ |/$\;\;\;\mathrm{tr}(\mathbf{P}_{\infty}^{\mathrm{Cent}})$ | $1.911$ | $1.764$ | $1.647$ | $1.590$ | $1.555$ | $1.520$ |

The performance of the different decentralized methods is graphically depicted below

<p style="text-align:center;"><img src="/assets/img/mhemfh_large_net_perf.svg" width="70%"></p>

We will consider $W_{ss} = 5$ henceforth.

Similarly to the previous synthetic network ([Small network](#small-network)), we can observe the evolution of the trace of the covariance over the window of the last iteration of the MFH method. The same conclusions apply.

<p style="text-align:center;"><img src="/assets/img/mhemfh_large_net_ev_trace.svg" width="70%"></p>

Lastly, we can check if the PMHE1 method is guaranteed to converge (with a window size equal to the size chosen for the MFH method)
~~~m
>> farina_et_al_2010_convergence(40,5);
~~~
~~~text
PMHE1 is not guaranteed to converge.
~~~
We conclude that the **PMHE1 method is not guaranteed to converge**. Note that we considered the process noise between coupled systems to be **correlated**, which **PMHE1 does not support**. Nevertheless, the convergence analysis only depends on $\mathbf{A}$ and $\mathbf{C}$, so that had no influence. Also note that, according to [[3]](#references), there is no indication that it does not converge (it is just not guaranteed to converge).

Removing correlations between process noise of coupled systems (model ```model41```), we can run a simulation to see whether the error dynamics of the PMHE1 method are stable

~~~m
>> model = 41;
>> parameters = struct('sim_T',1,'PMHE1_N',5);
>> methods = [true; true; false; false; true];
>> simulation(model, methods, parameters);
~~~
~~~text
Luenberger filters simulation completed.
PMHE1 filter iteration: 1
Starting parallel pool (parpool) using the 'local' profile ...
Connected to the parallel pool (number of workers: 30).
PMHE1 filter iteration: 2
(...)
PMHE1 filter iteration: 100
PMHE1 filter simulation completed.
~~~

<p style="text-align:center;"><img src="/assets/img/mhemfh_large_net_pmhe1_div.svg" width="70%"></p>

We conclude that, even without process noise correlations, the **PMHE1 method does not converge** for this network.


Let's assess the performance of the different estimation solutions resorting to Monte Carlo simulations:
~~~m
>> model = 40;
>> parameters = struct('sim_T',200,'MFH_w_ss',5,'mc_it',1000);
>> methods = [true; true; true; true; false];
>> simulation_mc(model, methods, parameters);
~~~
~~~text
Trace C: 	844.089937 	(ri: 1.34686e-16)
Trace OS/C: 	2.117501 	(ri: 1.83691e-05)
Trace FH/C: 	1.710140 	(ri: 4.34463e-11)
Trace MFH/C: 	1.590347	(ri: 4.31416e-09)
Monte Carlo simulation 1/1000.
(...)
Monte Carlo simulation 999/1000.
Monte Carlo simulation 1000/1000.
Finished Monte Carlo simulations.
Processing simulations.
Processing method 1/5.
Processing method 2/5.
Processing method 3/5.
Processing method 4/5.
~~~

After 1000 Monte Carlo simulations the evolution estimation error covariance is depicted below
<p style="text-align:center;"><img src="/assets/img/mhemfh_large_net_mc.svg" width="70%"></p>

The steady-state trace of the estimation error covariance matrix is depicted below, which was obtained by averaging the 20 last instants of the MC simulations.

| | C | OS | FH | MFH ($W_{ss} = 5$) |
|:-: | :-: | :-: | :-: | :-: |
| $\mathrm{tr}(\mathbf{P_{\mathrm{MC}}}) \times 10^{-3} $ | 0.8449 |  1.797 |  1.451  | 1.348 |
| Relative to MFH |  $-37.34\%$ |    $+33.29\%$ |  $+7.59\%$ | $--$  |

In [[1]](#references) it is argued that the MHE filter synthesize with the MFH method is **consistent**. We can perform a simulation and look at a component of the estimation error to confirm that.
~~~m
>> model = 40;
>> parameters = struct('sim_T',1000,'MFH_w_ss',5);
>> methods = [true; true; true; true; false];
>> simulation(model, methods, parameters);
~~~
~~~text
Luenberger filters simulation completed.
MFH filter simulation completed.
~~~

The evolution of a randomly chosen component of the error and the corresponding $3\sigma$ bounds is depicted below.

<p style="text-align:center;"><img src="/assets/img/mhemfh_large_net_3sigma.svg" width="70%"></p>

We can confirm that it is **zero-mean** and the calculated estimation covariance is **not over-confident**.

**Conclusions**:
- The **PMHE1** method **does not converge**
- The **MFH** method **converges** for different window sizes
- The **MFH** method supports **correlated process noise**
- The **MFH** method is **scalable**
- For low window sizes the **MFH** method is significantly **faster to synthesize** than the FH method
- The MFH method **performs better** than state-of-the-art decentralized Luenberger-based filters
- The use of the second best performing method (FH) incurs in a **performance penalty** of  $+7.59\%$
- The **MHE** framework of the **MFH** method is consistent

***

## Case of FH convergence issues

Let's generate another synthetic network with $N = 500$ systems. This network was generated randomly using the function
~~~m
>> rng_seed = 39;
>> generate_random_network(rng_seed);
~~~
~~~text
Global state size: 1000
Maximum absolute eigenvalue: 1.33875
~~~

We obtain the following topology
<p style="text-align:center;"><img src="/assets/img/mhemfh_large_FHdiv_net.pdf" width="80%"></p>

The centralized and OS gains can be computed similarly to the previous synthetic networks. Let's attempt to synthesize the FH algorithm

~~~m

~~~
~~~text

~~~

**Conclusions**:
- The **FH** method sometimes **fails to converge** to a constant gain
- In those cases the **MFH** still coverges

***

## Large-scale network with weak couplings


Let's increase the dimension of the network even more and generate a new model with $N = 1000$ systems. The parameters for the generation of this network are set so that:
- The dynamic **couplings** between systems are **weak** (so that the PMHE1 method converges)
- The **process noise** is **uncorrelated** between systems (to employ the PMHE1 method)

This is the second synthetic network used in [[1]](#references). This network was generated randomly using the function
~~~m
>> seed = 42;
>> generate_random_network(seed);
~~~
~~~text
Global state size: 2000
Maximum absolute eigenvalue: 1.13125
~~~

We obtain the following topology
<p style="text-align:center;"><img src="/assets/img/mhemfh_large_weak_net.pdf" width="80%"></p>

Lets's synthesize the centralized and OS methods

~~~m
>> synthesis_luenberger(42);
~~~

~~~text
----------------------------------------------------------------------------------
Running one-step algorithm with: epsl = 0.0001 | maxIt = 1000.
Convergence reached with: epsl = 0.0001 | maxIt = 1000.
A total of 319 iterations were run.
----------------------------------------------------------------------------------
Elapsed time is 17276.108066 seconds.
Maximum absolute control CL eigenvalue: 1.000605
~~~

It was **not possible** to compute the **FH** synthesis in a reasonable amount of time (a few days).

Lets synthesize the MFH method with $W_{ss} = 2$

~~~m
>> synthesis_mfh(42,1e-4,2,1e3);
~~~
~~~text
----------------------------------------------------------------------------------
Running moving finite horizon algorithm with:
epsl_inf = 0.0001 | W = 2 | maxIt = 1000 .
Convergence reached with: epsl_inf = 0.0001 | W = 2 | maxOLIt = 1000
A total of 246 outer loop iterations were run.
----------------------------------------------------------------------------------
Trace MFH: 6722.959282
Trace MFH/C: 2.601179
For W_ss = 2 (tr = 6722.96) elapsed time is 74398.1 seconds.
~~~

Lastly, we find that the **PMHE1** method **is guaranteed to converge** (with a window size equal to the size chosen for the MFH method)
~~~m
>> farina_et_al_2010_convergence(42,2);
~~~
~~~text
PMHE1 is guaranteed to converge.
~~~

We can simulate this methods can be simulated using function ```simulation``` for $W_{\text{PMHE1}} = 2$ and $W_{\text{PMHE1}} = 5$. The evolution of the error norm is depicted below

<p style="text-align:center;"><img src="/assets/img/mhemfh_large_weak_net_sim.svg" width="70%"></p>

The average of the error norm for each of the methods is depicted below

| | C | OS | MFH ($W_{ss} = 2$) | PMHE1 ($W_{\text{PMHE1}} = 2$) | PMHE1 ($W_{\text{PMHE1}} = 5$) |
| |:-: | :-: | :-: | :-: | :-: |
| | 49.95 |  173.9 |  81.92 |  132.3 |  140.1 |
| Relative to MFH | $-39.02\%$ | $+112.3\%$ | $--$ | $+61.54\%$ | $+70.98\%$ |


**Conclusions**:
- The **FH** method **could not be synthesized** in a reasonable amount of time for $N=1000$ systems
- The **MFH** method is **scalable**
- The use of the second best performing method (PMHE1) incurs in a **performance penalty** of  $+61.54\%$

***

# References
[1] *Not published yet*

[2] {% for paper in site.data.references%}{% if paper.key == "Viegas2018Discrete" %}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endif %}{% endfor %}

[3] [Farina, M., Ferrari-Trecate, G. and Scattolini, R., 2010. Moving-horizon partition-based state estimation of large-scale systems. Automatica, 46(5), pp.910-918.](https://www.sciencedirect.com/science/article/pii/S0005109810000634)
{: .text-justify}

{% comment %}
[3] [C. M. Fransson and B. Lennartson, "Low order multicriteria /spl Hscr//sub /spl infin// design via bilinear matrix inequalities," 42nd IEEE International Conference on Decision and Control (IEEE Cat. No.03CH37475), 2003, pp. 5161-5167 Vol.5, doi: 10.1109/CDC.2003.1272456.](https://ieeexplore.ieee.org/abstract/document/1272456)
{: .text-justify}

[4] [Viegas, D., Batista, P., Oliveira, P. and Silvestre, C., 2013, June. Gas decentralized navigation filters in a continuous-discrete fixed topology framework. In 21st Mediterranean Conference on Control and Automation (pp. 1286-1291). IEEE.](https://ieeexplore.ieee.org/abstract/document/6608885)
{: .text-justify}



[6] [Lofberg, J., 2004, September. YALMIP: A toolbox for modeling and optimization in MATLAB. In 2004 IEEE international conference on robotics and automation (IEEE Cat. No. 04CH37508) (pp. 284-289). IEEE.](https://ieeexplore.ieee.org/abstract/document/1393890)
{: .text-justify}

[7] [Sturm, J.F., 1999. Using SeDuMi 1.02, a MATLAB toolbox for optimization over symmetric cones. Optimization methods and software, 11(1-4), pp.625-653.](https://www.tandfonline.com/doi/abs/10.1080/10556789908805766)
{: .text-justify}

Let's compute the $\mathcal{H}_2$ minimization synthesis with

~~~m
synthesis_h2(1);
~~~
~~~text
Iteration 1.
SeDuMi 1.3.5 by AdvOL, 2005-2008 and Jos F. Sturm, 1998-2003.
Alg = 2: xz-corrector, theta = 0.250, beta = 0.500
eqs m = 1641, order n = 303, dim = 29203, blocks = 5
nnz(A) = 17922 + 0, nnz(ADA) = 2689681, nnz(L) = 1345661
 it :     b*y       gap    delta  rate   t/tP*  t/tD*   feas cg cg  prec
  0 :            9.54E+00 0.000
  1 :  -2.91E+00 4.91E+00 0.000 0.5148 0.9000 0.9000   3.37  1  1  3.6E+00
(...)
 28 :  -5.90E+01 5.83E-10 0.000 0.2902 0.9000 0.9000   1.00  1  1  9.9E-10

iter seconds digits       c*x               b*y
 28     83.6   9.8 -5.8962751422e+01 -5.8962751431e+01
|Ax-b| =   4.1e-09, [Ay-c]_+ =   2.6E-10, |x|=  2.1e+02, |y|=  6.7e+01

Detailed timing (sec)
   Pre          IPM          Post
1.197E-01    2.790E+01    2.424E-02    
Max-norms: ||b||=1, ||c|| = 2.219032e+00,
Cholesky |add|=0, |skip| = 0, ||L.L|| = 1.76149.
SeDuMi 1.3.5 by AdvOL, 2005-2008 and Jos F. Sturm, 1998-2003.
Alg = 2: xz-corrector, theta = 0.250, beta = 0.500
eqs m = 861, order n = 263, dim = 27603, blocks = 4
nnz(A) = 3578 + 0, nnz(ADA) = 674081, nnz(L) = 337471
 it :     b*y       gap    delta  rate   t/tP*  t/tD*   feas cg cg  prec
  0 :            2.98E+01 0.000
  1 :  -3.17E+01 1.58E+01 0.000 0.5304 0.9000 0.9000   0.79  1  1  3.0E+00
(...)  
Trace H2: 58.961951
Trace H2/C: 1.956898
Maximum absolute CL eigenvalue: 0.916798
~~~
which uses YALMIP [[6]](#references) and SeDuMi [[7]](#references).

{% endcomment %}
