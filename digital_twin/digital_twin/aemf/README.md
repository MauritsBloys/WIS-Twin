# AEMF - Active Estimation of Multiplicative Faults

Welcome to the AEMF repository! It contains Matlab implmentations of the algorithms for filter design and input design presented in the paper "Active Estimation of Multiplicative Faults in Dynamical Systems", by Gabriel de Albuquerque Gleizer, Peyman Mohajerin Esfahani and Tamas Keviczky. These algorithms provide a scalable method for high-accuracy fault estimation in systems described by linear DAEs, when the faults are modeled as multiplicative.

## How to run
The following sequence of scripts can be used to reproduce the results for the paper.

```
>>> main;  % generates results of Section 5.3.1
>>> main_bounds;  % computes theoretical bounds for Section 5.3.1
>>> main_gaussnewton;  % generates results of Section 5.3.2
>>> main_ltv;  % generates results of Section 5.3.3
```

They should also provide a good way of understanding the fault estimation pipeline, depending on the assumptions placed on the fault. See the paper for further information.

## Requirements
This software is written in Matlab. The code to generate the filters and optimal input only requires the Control Systems Toolbox. The convex relaxation to get a suboptimality gap certificate for the input requires [CVX 3](https://cvxr.com/).

The simulations to reproduce the paper's results were built in Simulink. The toolboxes Simulink Real-Time and Stateflow are needed for them.

## Support
Please use the issue tracking system if bugs are found. For more information, contact the first author of the paper.

## Authors and acknowledgment
This work was supported by the Digital Twin project with project number P18-03 of the research programme TTW-Perspectief, which is partly financed by the Dutch Research Council (NWO). Peyman Mohajerin Esfahani acknowledges the support of the ERC grant TRUST949796. The authors are with Delft University of Technology, The Netherlands; Peyman is also with University of Toronto, Canada.

## License
This is an open-source software under the BSD 3-Clause license. See `LICENSE` for details.

## Project status
This project is complete, but there are many opportunities for extensions and improvement.
