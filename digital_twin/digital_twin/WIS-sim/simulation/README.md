# Simulation

## Controller
Controller creation based on the work of *Jacob Lont*
* Run cantoni_LMI.m to create controller
* Shape local compensators in lab_setup_values.m
 
## Full simulation
* simulate_3pools.slx

Run cantoni_LMI.m first to create controller and models.
Run plot_sim.m to save and plot simulation.

## HIL-simulation
* simulate_3pools_hil.slx

Run model_hil.m first to create controller and models.
Run save_hil.m to save simulation.
Run plot_hil.m to plot simulation.

## Create C code
Run cantoni_LMI.m first to create controller and models.

* compensator_parameters_c.m

Prints c code with local compensator parameters.

* expand_matrix_multiplication_c.m

Expands a matrix multiplication as c code. Used to create code for the global controller.
```
expand_matrix_multiplication_c(comb_contr.D, "uc", "cont_ctrl_array", 0)
```