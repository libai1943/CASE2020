# On-Road Planning with Spatio-Temporal RRT* and QP Smoothing

**Search a trajectory in space and time, then refine it with a quadratic program.** This is the MATLAB research implementation of **“On-road Trajectory Planning with Spatio-temporal RRT* and Always-feasible Quadratic Program”**, presented at **IEEE CASE 2020**.

Moving traffic couples where the ego vehicle drives with when it gets there. The method searches directly in longitudinal position, lateral position and time, instead of fixing a path first and assigning its speed afterward. A modified RRT* supplies a coarse trajectory; a QP smoother refines it inside local admissible regions.

## Architecture

```mermaid
flowchart LR
    A[Road, vehicle and moving obstacles] --> B[Spatio-temporal RRT*]
    B --> C[Coarse s-l-t trajectory]
    C --> D[Resampling and local box construction]
    D --> E[AMPL QP / IPOPT]
    E --> F[Smoothed trajectory and display]
```

`SearchCoarseTrajectoryViaRRTStar.m` contains the search logic, including sampling, candidate parents/children, collision checks and tree updates. `OptimizeTrajectory.m` constructs the waypoint and box data used by `QP.mod`.

The QP objective combines squared first/second differences with attraction to the reference waypoints. Its constraints are linear integration relations, boundary conditions and box bounds. The supplied implementation uses IPOPT through AMPL even though this stage is a quadratic program. “Always-feasible” refers to the paper's smoother construction under its assumptions; it does not guarantee that every changed scene will yield a valid coarse search result.

## Run

Requirements: **MATLAB on Windows**, the supplied P-code helpers, and a working **AMPL/IPOPT** installation with a valid AMPL license and compatible support libraries. The original `.exe` workflow uses relative filenames and Windows shell commands.

```matlab
cd('C:/path/to/CASE2020');
rng(0);                       % Optional: repeat the randomized search sequence
RunMe;
```

The driver creates the scenario, searches, optimizes and invokes the protected dynamic display. Keep the repository writable: `Waypoints`, `Boxes`, `NFE0`, `x.txt` and `y.txt` are generated/overwritten during optimization. No MATLAB–AMPL API is needed.

`rr.run` exports solver values but does not implement a separate robust success-checking interface. When troubleshooting a failed run, examine the current AMPL/IPOPT output before treating exported files as a successful new solution.

## Function and file guide

| File | Role |
| --- | --- |
| `RunMe.m` | Entry script, physical/search parameters, obstacle setup and stage orchestration. |
| `SearchCoarseTrajectoryViaRRTStar.m` | Spatio-temporal RRT* search and its local sampling, steering and tree helpers. |
| `OptimizeTrajectory.m` | Resample the reference, construct local bounds, write AMPL inputs and read the refined trajectory. |
| `GenerateObstacles.p` | Protected moving-obstacle scenario generation. |
| `Resample3DPath` | Local function in `OptimizeTrajectory.m`; resample the space–time reference. |
| `DynamicPlot.p` | Protected visualization of the ego trajectory and traffic. |
| `QP.mod` | Quadratic objective and linear constraints for trajectory smoothing. |
| `rr.run` | AMPL solve and coordinate export. |
| `ipopt.opt` | IPOPT options. |

The search and optimization files contain their own local geometry and collision-checking helpers. Keep both protected `.p` components with the entry script.

## Main settings

The defaults in `RunMe.m` use an 8 s search horizon, 200 search iterations, a 20 m/s speed ceiling, a 15 m/s suggested cruising speed and five moving obstacle vehicles. The road spans lateral coordinates 0–7 m. `param_.Nfe = 100` sets the reference resolution; the AMPL QP internally uses `3 * Nw` points before selecting output waypoints.

Change scenario and search parameters in `RunMe.m`, sampling/steering behavior in the search function, and the smoothing objective or constraints in `QP.mod`. This is a research planning demo, not a vehicle-control stack.

## Citation

Please cite the paper when using this code or its planning method:

> Bai Li, Qi Kong, Youmin Zhang, Zhijiang Shao, Yumeng Wang, Xiaoyan Peng, and Daxun Yan, “On-road Trajectory Planning with Spatio-temporal RRT* and Always-feasible Quadratic Program,” *2020 IEEE 16th International Conference on Automation Science and Engineering (CASE)*, pp. 942–947, 2020. [DOI](https://doi.org/10.1109/CASE48305.2020.9217044).

```bibtex
@inproceedings{Li2020SpatioTemporalRRT,
  author = {Li, Bai and Kong, Qi and Zhang, Youmin and Shao, Zhijiang
            and Wang, Yumeng and Peng, Xiaoyan and Yan, Daxun},
  title = {On-road Trajectory Planning with Spatio-temporal {RRT*}
           and Always-feasible Quadratic Program},
  booktitle = {2020 IEEE 16th International Conference on Automation
               Science and Engineering (CASE)},
  pages = {942--947}, year = {2020},
  doi = {10.1109/CASE48305.2020.9217044}
}
```

## License

See [GNU GPL v3](LICENSE) and the notices on the bundled third-party executables and libraries.
