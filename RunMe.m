% ==============================================================================
% Source Codes for "On-road Trajectory Planning with Spatio-temporal RRT*
% and Always-feasible Quadratic Program". accepted in IEEE CASE 2020.
% ==============================================================================
%   Copyright (C) 2020 Bai Li
%   Users of this code are suggested to cite the following article. Bai Li
%   et al., "On-road Trajectory Planning with Spatio-temporal RRT* and
%   Always-feasible Quadratic Programh", 16th IEEE International Conference
%   on Automation Science and Engineering, accepted, 2020.   
%   License GNU General Public License v3.0
% ==============================================================================
clear all; close all; clc;
% % Parametric Settings
global param_
% Geometric size of the ego vehicle and the surrouding social vehicles (assumed to be identical)
param_.vehicle_wheelbase = 2.8;
param_.vehicle_front_hang = 0.96;
param_.vehicle_rear_hang = 0.929;
param_.vehicle_width = 1.942;
param_.vehicle_length = param_.vehicle_wheelbase + ...
    param_.vehicle_front_hang + param_.vehicle_rear_hang;
% Vehicle kinematics
param_.vehicle_v_max_long = 20.0;
param_.vehicle_v_suggested_long = 15.0;
% RRT and QP related params
param_.tf = 8.0;
param_.Nmax_iter = 200;
param_.s0 = 0;
param_.l0 = 1.75;
param_.ltf = 1.75;
param_.L_norminal = 1.75;
param_.s_max = param_.s0 + param_.vehicle_v_max_long * param_.tf;
param_.ds0 = 10;
param_.dds0 = 0;
param_.dl0 = 0.1;
param_.ddl0 = 0;
param_.Nlat_samples = 50;
param_.prob_long = 0.8;
param_.prob_lat = 0.2;
param_.distance_near = 50;
param_.weight_inside_exp = 2;
param_.weight_for_collision_risks = 5;
param_.weight_for_drastic_long_vel_change = 10;
param_.weight_for_drastic_lat_vel_change = 20;
param_.weight_for_biased_long = 1;
param_.weight_for_biased_lat = 1;
param_.Nfe = 100;

% % Randomly generate environmental obstacles
global environment_
environment_.num_obs = 5;
environment_.obs_agv_vel = 10;
environment_.road_left_barrier = 7;
environment_.road_right_barrier = 0;
environment_.obstacles = GenerateObstacles();

% % Coarse trajectory planning via spatio-temporal RRT*
global coarse_trajectory precise_trajectory
coarse_trajectory = SearchCoarseTrajectoryViaRRTStar();
precise_trajectory = OptimizeTrajectory(coarse_trajectory);

% % Produce Results
DynamicPlot();