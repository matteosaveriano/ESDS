# Energy-based Stabilizer of Dynamical Systems (ESDS)

An implementation of the ESDS approach described in [(Saveriano, 2020)](https://arxiv.org/pdf/2003.11290.pdf).

## Demo description
- `main_LASA.m`: a demo to run ESDS on the LASA Handwriting dataset.

## Software Requirements
The code is developed and tested under `Ubuntu 18.04` and `Matlab2019b`.

## References
Please acknowledge the authors in any academic publication that used parts of these codes.
```
@inproceedings{saveriano2020energy,
	author = {Saveriano, M.},
	booktitle = {IEEE International Conference on Robotics and Automation},
	title = {An Energy-based Approach to Ensure the Stability of Learned Dynamical Systems},
	pages={7041--7047},
	year = {2020}
}

```

## Third-party material
Third-party code and dataset have been included in this repository for convenience.

- **LASA Handwriting dataset**: please acknowledge the authors in any academic publications that have made use of the LASA HandWriting dataset by citing: *S. M. Khansari-Zadeh and A. Billard, "Learning Stable Non-Linear Dynamical Systems with Gaussian Mixture Models", IEEE Transaction on Robotics, 2011*.

- **GMR library**: please acknowledge the authors in any academic publications that have made use of the GMR library by citing: *S. Calinon et al., "On Learning, Representing and Generalizing a Task in a Humanoid Robot", IEEE Transactions on Systems, Man and Cybernetics, Part B., 2006*.

- **Energy-Tank**: The energy tank-based stabilization re-uses part of the code of the approach described in: *K. Kronander and A. Billard, "Passive Interaction Control with Dynamical Systems", RA-L, 2017*.

## Note
This source code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY.
