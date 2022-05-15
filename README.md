# LEXO-ANO
control and simulation for an exo-skeleton LEXO from STARX @ UMich  
This is a simple demo of our control and parameters are not tuned yet.
## Instruction
Run `virtual_model\TwoLinkTestBed.m` to see simulated trajectory and then run `virtual_model\animation_robot.m` to visualize.  
Run `ANO_sim\Ano_sim.m` to see state space simulation of the Adaptive Neuro Oscillator.
## Results
Position tracking and frequency convergence under Gaussian disturbance.
### Tracking a sine wave with converging frequency
![ANO-tracking](docs/ANO-tracking.png)
### State-space plot of two trajectories with random initial position
![ANO-state-space](docs/ANO-state-space.png)
## Reference
1. The built-in simulator of this project is developed by [jakub-kaminski](https://github.com/jakub-kaminski/2link-robot)
2. The [Adaptive Neuro Oscillator](https://www.sciencedirect.com/science/article/pii/S0921889016307485).
3. Theoretical analysis of an [Adaptive Frequency Oscillator](https://www.sciencedirect.com/science/article/pii/S0167278906000819).