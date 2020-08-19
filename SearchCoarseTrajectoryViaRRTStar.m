function path = SearchCoarseTrajectoryViaRRTStar()
global param_ environment_
q_init.slt = [param_.s0, param_.l0, 0];
q_init.cost = 0; % Herein the cost is cost-to-come, not cost-to-go.
q_init.parentid = -999; % -999 represents the Null in the paper.
q_init.ds = param_.ds0;
q_init.dl = param_.dl0;

global center_right_barrier center_left_barrier
center_left_barrier = environment_.road_left_barrier - param_.vehicle_width * 0.5;
center_right_barrier = environment_.road_right_barrier + param_.vehicle_width * 0.5;
global node_list_slt node_list
node_list_slt = q_init.slt;
node_list = q_init;

q_goal.slt = [param_.s_max, param_.ltf, param_.tf]; % This is the nominal goal node, which is not necessarily reachable.
q_best_cost = Inf;

time_sampling_val_index = randperm(param_.Nmax_iter);
for iter = 1 : param_.Nmax_iter
    q_new.slt(3) = param_.tf * time_sampling_val_index(iter) / param_.Nmax_iter;
    q_new.slt(1) = SpecifyLongValueRandomly(q_new.slt(3));
    q_new.slt(2) = SpecifyLatValueRandomly(q_new.slt(1), q_new.slt(3));
    if (q_new.slt(2) == -999)
        continue;
    end
    
    [q_nearest_underlying_parent_id, q_nearest_underlying_child_id] = FindParentAndChild(q_new.slt);
    
    parent_current_best_cost_value = Inf;
    q_new.parentid = -999;
    for kk = 1 : length(q_nearest_underlying_parent_id)
        current_candidate_parent = node_list(q_nearest_underlying_parent_id(kk));
        candidate_cost = current_candidate_parent.cost + ComputeDistance(current_candidate_parent, q_new);
        if ((candidate_cost < parent_current_best_cost_value)&&(IsSteerCollisionFree(current_candidate_parent.slt, q_new.slt)))
            parent_current_best_cost_value = candidate_cost;
            q_new.parentid = q_nearest_underlying_parent_id(kk);
            q_new.cost = parent_current_best_cost_value;
        end
    end
    if (q_new.parentid == -999)
        continue;
    end
    
    dt = q_new.slt(3) - node_list(q_new.parentid).slt(3);
    q_new.ds = (q_new.slt(1) - node_list(q_new.parentid).slt(1)) / dt;
    q_new.dl = (q_new.slt(2) - node_list(q_new.parentid).slt(2)) / dt;
    node_list = [node_list, q_new];
    node_list_slt = [node_list_slt; q_new.slt];
    
    % Rewire
    for kk = 1 : length(q_nearest_underlying_child_id)
        id = q_nearest_underlying_child_id(kk);
        current_candidate_child = node_list(id);
        candidate_cost = q_new.cost + ComputeDistance(q_new, current_candidate_child);
        if (candidate_cost < current_candidate_child.cost) && (IsSteerCollisionFree(q_new.slt, current_candidate_child.slt))
            node_list(id).parentid = length(node_list); % Since q_new is the last one added in the node_list, the length is also its ID.
            node_list(id).cost = candidate_cost;
            dt = node_list(id).slt(3) - q_new.slt(3);
            node_list(id).ds = (node_list(id).slt(1) - q_new.slt(1)) / dt;
            node_list(id).dl = (node_list(id).slt(2) - q_new.slt(2)) / dt;
        end
    end
    
    % Update current best node
    q_best_cost_candidate = MeasureDistancePure(q_new, q_goal);
    if (q_best_cost_candidate < q_best_cost)
        q_best_cost = q_best_cost_candidate;
        q_best_id = length(node_list);
    end
end

path = [];
parent_id = q_best_id;
while (parent_id ~= -999)
    path = [node_list(parent_id).slt; path];
    parent_id = node_list(parent_id).parentid;
end
end

function s = SpecifyLongValueRandomly(time)
global param_
if (rand < param_.prob_long)
    v_avg = param_.vehicle_v_max_long * rand;
else
    v_avg = param_.vehicle_v_suggested_long;
end
s = v_avg * time;
end

function l_val = SpecifyLatValueRandomly(s_ego, time)
global environment_ param_ center_right_barrier center_left_barrier
num_obs = length(environment_.obstacles);
obs_ss = [];
obs_ll = [];
for ii = 1 : num_obs
    obs_s = environment_.obstacles{1,ii}.st(time);
    if (abs(s_ego - obs_s) > param_.vehicle_length)
        continue;
    end
    obs_l = environment_.obstacles{1,ii}.lt(time);
    obs_ss = [obs_ss, obs_s];
    obs_ll = [obs_ll, obs_l];
end

for counter = 1 : param_.Nlat_samples
    if (rand < param_.prob_lat)
        l_val = param_.L_norminal;
    else
        l_val = center_right_barrier + (center_left_barrier - center_right_barrier) * rand;
    end
    is_ready = 1;
    for ii = 1 : length(obs_ss)
        if (IsV1CollidingWithV2(s_ego, l_val, obs_ss(ii), obs_ll(ii)))
            is_ready = 0;
            break;
        end
    end
    if (is_ready == 1)
        return;
    end
end
l_val = -999;
end

function [q_nearest_underlying_parent_id, q_nearest_underlying_child_id] = FindParentAndChild(q_new_slt)
global node_list_slt param_ node_list
num_nodes = length(node_list);
mat = node_list_slt - q_new_slt;
mat = mat .^ 2;
error_vec = sum(mat')';
[error_vec, index_sequence] = sort(error_vec);
q_nearest_underlying_parent_id = [];
q_nearest_underlying_child_id = [];
cur_time = q_new_slt(3);
threshold = param_.distance_near^2;

for ii = 1 : num_nodes
    if (error_vec(ii) > threshold)
        return;
    end
    ind = index_sequence(ii);
    cur_node = node_list(ind);
    if ((cur_node.slt(3) > cur_time)&&(cur_node.slt(1) > q_new_slt(1)))
        q_nearest_underlying_child_id = [q_nearest_underlying_child_id, ind];
    end
    if ((cur_node.slt(3) < cur_time)&&(cur_node.slt(1) < q_new_slt(1)))
        q_nearest_underlying_parent_id = [q_nearest_underlying_parent_id, ind];
    end
end
end

function val = ComputeDistance(node_a, node_b)
global environment_ param_ center_left_barrier
init_velocity_long = node_a.ds;
init_velocity_lat = node_a.dl;

lon_distance = node_b.slt(1) - node_a.slt(1);
lat_distance = node_b.slt(2) - node_a.slt(2);
dt = node_b.slt(3) - node_a.slt(3);

avg_velocity_long = lon_distance / dt;
avg_velocity_lat = lat_distance / dt;

num_obs = length(environment_.obstacles);
num_samples = round(dt / 0.2) + 2;
timeline = linspace(node_a.slt(3), node_b.slt(3), num_samples);
sline = linspace(node_a.slt(1), node_b.slt(1), num_samples);
lline = linspace(node_a.slt(2), node_b.slt(2), num_samples);

cost_for_collision = 0;
for ii = 1 : num_obs
    obs_st = environment_.obstacles{1,ii}.st;
    obs_lt = environment_.obstacles{1,ii}.lt;
    for jj = 1 : num_samples
        time = timeline(1,jj);
        s_ego = sline(jj);
        l_ego = lline(jj);
        s_other = obs_st(time);
        l_other = obs_lt(time);
        distance = hypot(s_ego - s_other, l_ego - l_other);
        cost_for_collision = cost_for_collision + exp((distance^2) * -param_.weight_inside_exp);
    end
end
cost_for_collision = cost_for_collision / (num_obs * num_samples);

val = param_.weight_for_collision_risks * cost_for_collision + ...
    param_.weight_for_drastic_long_vel_change * (abs(init_velocity_long - avg_velocity_long) / dt) + ...
    param_.weight_for_drastic_lat_vel_change * (abs(init_velocity_lat - avg_velocity_lat) / dt) + ...
    param_.weight_for_biased_long * abs(avg_velocity_long - param_.vehicle_v_suggested_long) + ...
    param_.weight_for_biased_lat * abs(0.5 * (node_b.slt(2) + node_a.slt(2)) - param_.L_norminal) / (center_left_barrier - param_.L_norminal);
end

function val = IsV1CollidingWithV2(s_ego, l_ego, s_other, l_other)
global param_
vehicle_half_width = param_.vehicle_width * 0.5;
v1.xmin = s_ego - param_.vehicle_rear_hang;
v1.xmax = s_ego + param_.vehicle_front_hang + param_.vehicle_wheelbase;
v1.ymin = l_ego - vehicle_half_width;
v1.ymax = l_ego + vehicle_half_width;

v2.xmin = s_other - param_.vehicle_rear_hang;
v2.xmax = s_other + param_.vehicle_front_hang + param_.vehicle_wheelbase;
v2.ymin = l_other - vehicle_half_width;
v2.ymax = l_other + vehicle_half_width;

if ((v2.ymax < v1.ymin) || (v1.ymax < v2.ymin) || (v2.xmax < v1.xmin) || (v1.xmax < v2.xmin))
    val = 0;
else
    val = 1;
end
end

function is_valid = IsSteerCollisionFree(a, b)
is_valid = 0;

lat_distance = b(2) - a(2);
lon_distance = b(1) - a(1);
dt = b(3) - a(3);

avg_velocity_long = lon_distance / dt;
avg_velocity_lat = lat_distance / dt;

global param_
if (hypot(avg_velocity_long, avg_velocity_lat) > param_.vehicle_v_max_long)
    return;
end
if (abs(avg_velocity_lat / avg_velocity_long) > 0.5)
    return;
end

global environment_
num_samples = round(abs(b(3) - a(3)) / 0.1) + 2;
sline = linspace(a(1), b(1), num_samples);
lline = linspace(a(2), b(2), num_samples);
timeline = linspace(a(3), b(3), num_samples);

for ii = 1 : length(environment_.obstacles)
    obs_st = environment_.obstacles{1,ii}.st;
    obs_lt = environment_.obstacles{1,ii}.lt;
    for jj = 1 : num_samples
        time = timeline(1,jj);
        s_ego = sline(jj);
        l_ego = lline(jj);
        s_other = obs_st(time);
        l_other = obs_lt(time);
        if (abs(s_other - s_ego) >= param_.vehicle_length)
            continue;
        else
            if (IsV1CollidingWithV2(s_ego, l_ego, s_other, l_other))
                return;
            end
        end
    end
end
is_valid = 1;
end

function distance3dim = MeasureDistancePure(node_a, node_b)
global param_ environment_
lon_distance = node_b.slt(1) - node_a.slt(1);
lat_distance = node_b.slt(2) - node_a.slt(2);
time_distance = node_b.slt(3) - node_a.slt(3);

distance3dim = (abs(lon_distance) / (param_.s_max - param_.s0)) + ...
    (abs(time_distance) / param_.tf) + ...
    (abs(lat_distance - param_.L_norminal) / (environment_.road_left_barrier - environment_.road_right_barrier));
end

