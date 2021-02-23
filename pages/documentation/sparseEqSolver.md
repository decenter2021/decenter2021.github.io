---
title: sparseEqSolver
showTitle: true
excerpt: Documentation for function sparseEqSolver.
permalink: /documentation/sparseEqSolver/
layout: single
classes: wide
sidebar:
  title: "sparseEqSolver"
  nav: sidebar-sparseEqSolver
tags:
    - solver
    - documentation
date: "2021-02-14"
last_modified_at: "2021-02-22"
---
# Sintax
~~~m
X = sparseEqSolver(A,B,C,E)
~~~
***

# Description

Consider the matrix equation

$$
\begin{cases}
\left[\mathbf{A}\mathbf{X}\mathbf{B}-\mathbf{C}\right]_{i,j} = 0 & \;,\; \mathbf{E}_{i,j} \neq 0\\
\left[\mathbf{X}\right]_{i,j} = 0  & \;,\; \mathbf{E}_{i,j} = 0
\end{cases}\;,
$$

where $\mathbf{A}\in \mathbb{R}^{n\times n}$, $\mathbf{B}\in \mathbb{R}^{o\times o}$, $\mathbf{C}\in \mathbb{R}^{n\times o}$, and $\mathbf{E}\in \mathbb{R}^{n\times o}$, which represents a sparsity pattern, are known and $\mathbf{X} \in \mathbb{R}^{n\times o}$ is the unknown. This equation arises, for instance, in the computation of the decentralized finite-horizon Kalman filter gain and decentralized finite-horizon LQR gain.
{: .text-justify}

The function
{% highlight m%}X = sparseEqSolver(A,B,C,E){% endhighlight m%}
returns the solution to the  matrix equation above, assuming an unique solution exists. The algorithm used is detailed in [[Section 3, 1]](#references).
{: .text-justify}
# Input arguments
-  ```A``` : matrix $\mathbf{A}\in \mathbb{R}^{n\times n}$
-  ```B``` : matrix $\mathbf{B}\in \mathbb{R}^{o\times o}$
-  ```C``` : matrix $\mathbf{C}\in \mathbb{R}^{n\times o}$
-  ```E``` : sparsity pattern $\mathbf{E}\in \mathbb{R}^{n\times o}$

# Output Arguments
- ```X```: solution $\mathbf{X}\in \mathbb{R}^{n\times o}$

# Examples
### Random equation
To open this example execute the following command in the MATLAB command window
~~~m
open sparseEqSolverTutorial
~~~
Generate matrices $\mathbf{A}$, $\mathbf{B}$, $\mathbf{C}$, and $\mathbf{E}$ randomly
~~~m
n = 5;
o = 4;
A = rand(n,n)
A =
    0.9425    0.6404    0.8957    0.7099    0.8676
    0.5626    0.0884    0.8046    0.9169    0.5056
    0.7992    0.0585    0.4475    0.1403    0.1463
    0.4330    0.0572    0.4644    0.8736    0.8064
    0.0730    0.8537    0.0831    0.9491    0.6901
B = rand(o,o)
B =
    0.0217    0.4170    0.1672    0.8852
    0.4153    0.1643    0.3241    0.7918
    0.7735    0.6083    0.3607    0.4422
    0.3626    0.3697    0.4211    0.8731
C = rand(n,o)
C =
    0.1569    0.5736    0.3799    0.6590
    0.0243    0.9060    0.4589    0.5584
    0.6823    0.8052    0.4666    0.7979
    0.2590    0.6943    0.5368    0.4517
    0.4906    0.7305    0.1866    0.5506
E = round(rand(n,o))
E =
     1     0     1     1
     1     1     0     0
     1     1     1     0
     1     0     1     1
     0     0     1     1
~~~
Compute the solution to the random equation
~~~m
X = sparseEqSolver(A,B,C,E)
X =

   -2.5837         0    0.9561    1.5036
    2.5151   -2.5040         0         0
    4.4700   -3.0202    0.7491         0
   -2.4387         0   -3.5041    6.5209
         0         0    2.5341   -4.0368
~~~
which has the desired sparsity pattern. Finally, verify that it is the solution to the matrix equation
~~~m
sum(sum(abs((A*X*B-C).*E)))
ans =
   6.0958e-15
~~~

# References
[1] <a href="" target="_blank">Pedroso, L., Batista, P., 2021. Efficient algorithm for the computation of the solution to a sparse matrix equation in distributed control theory, <i>[Submitted to journal]</i>.</a>


{: .text-justify}
