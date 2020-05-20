function [path, ee] = RRT(x1_0, x2_0, x_ee, y_ee, x_obs, y_obs)

% Max iterations for motion planning
epoch = 10000;
fprintf('Max epochs set to %d \n', epoch);

% Tweaking when near goal
tweak = 1;

% Max and min x and y values of task space
x1_min = 0;
x2_min = 0;
x1_max = 2 * pi;
x2_max = 2 * pi;

% Acceptable threshold for final solution
final_thresh = 0.0707;

% Acceptable threshold for distance between states - "Visibility"
dist_thresh = 15 * pi/180;
fprintf('Visibility set to %f degrees \n', dist_thresh);

% Checkpoints
dist_max = 30 * pi/180; % L2 norm of angle distance, for each new branch
dist_check = pi/180; % distance between checkpoints
n_check = round(dist_max/dist_check); % number of checkpoints

% counter = 0; % counter starts at zero
% counter_scale = 15; % degrees "wiggle" in 3d plot
% counter_wriggle = 0.5; % how fast to "wiggle";

% Tree initalizations
tree.vertex(1).x1 = x1_0;
tree.vertex(1).x2 = x2_0;
tree.vertex(1).x1_prev = x1_0;
tree.vertex(1).x2_prev = x2_0;
tree.vertex(1).index_prev = 0;

% Plot congiguration
hold on
grid on
plot(x1_0, x2_0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor','k')

% Tree generation
check_break = 0; %
for i = 2:1:epoch
    
    if (mod(i, 100) == 0)
        fprintf('%d epochs done \n', i);
    end
    
    collision = 1;
    ct = 0; %
    
    while(collision == 1)
        % Generate random states
        region_rand1 = 1 * rand; % joint space region
        if (region_rand1 >= 0.5)
            region_rand1 = 1;
        else
            region_rand1 = -1;
        end
        x1_rand = region_rand1 * (x1_max - x1_min) * rand; % Random x1 generation
        
        region_rand2 = 1 * rand; % joint space region
        if (region_rand2 >= 0.5)
            region_rand2 = 1;
        else
            region_rand2 = -1;
        end
        x2_rand = region_rand2 * (x2_max - x2_min) * rand; % Random x2 generation
        
        % Find proximal node
        dist = Inf * ones(1,length(tree.vertex)); % Initializing distances corresponding to all vertices in the tree until current vertex to infinity
        for j = 1:length(tree.vertex) % Iterating from the first vertex to current vertex
            dist(j) = sqrt( (x1_rand - tree.vertex(j).x1)^2 + (x2_rand - tree.vertex(j).x2)^2 ); % Distance between each vertex and randomly generated state
        end
        [dist_min, index_min] = min(dist); % Minimum distance and corresponding vertex
        
        % Grow branch from proximal node to random node
        dist_grow1 = x1_rand - tree.vertex(index_min).x1;
        dist_grow2 = x2_rand - tree.vertex(index_min).x2;
        dist_extend = min(dist_min, dist_max);
        dist_way = dist_extend/dist_min; % only go this fraction of the way TOWARD the random point
        
        % Re-calculated random state
        for n_way = (1/n_check):(1/n_check):1
            x1_rand = tree.vertex(index_min).x1 + (n_way * dist_way * dist_grow1);
            x2_rand = tree.vertex(index_min).x2 + (n_way * dist_way * dist_grow2);
            
            collision = collision_avoidance(x1_rand, x2_rand, x_obs, y_obs);
            
            if collision==1
                break
            end
        end
    end
    
    tree.vertex(i).x1 = x1_rand;
    tree.vertex(i).x2 = x2_rand;
    tree.vertex(i).x1_prev = tree.vertex(index_min).x1;
    tree.vertex(i).x2_prev = tree.vertex(index_min).x2;
    tree.vertex(i).index_prev = index_min;
    
    plot([tree.vertex(i).x1; tree.vertex(i).x1_prev], [tree.vertex(i).x2; tree.vertex(i).x2_prev], 'r.-');
    
    if (mod(i, 50) == 0)  % only redraw every nth time a node is added
        title([num2str(i) ' total nodes'])
%         counter = counter + 1;
%         view(120 + counter_scale*sin(counter_wriggle * counter), 40);
        grid on
%         axis vis3d
        pause(0);
    end
    
    arm = forward_kinematics(tree.vertex(i).x1, tree.vertex(i).x2);
    x_ee_temp = arm(end, 1);
    y_ee_temp = arm(end, 2);
    
    if sqrt( (x_ee_temp - x_ee)^2 + (y_ee_temp - y_ee)^2 ) <= final_thresh
        ee = [x_ee_temp y_ee_temp];
        % keyboard
        break
    end
    
    % Tweaking
    if tweak && sqrt( (x_ee_temp - x_ee)^2 + (y_ee_temp - y_ee)^2 ) <= final_thresh*5
        % estimate a jacobian and try to tweak things a little:
        fprintf('------------ We are close! ------------\n');
    drawnow
        damt = 1e-5;
        arm_0 = forward_kinematics(tree.vertex(i).x1,tree.vertex(i).x2);
        arm_1 = forward_kinematics(tree.vertex(i).x1+damt,tree.vertex(i).x2);
        arm_2 = forward_kinematics(tree.vertex(i).x1,tree.vertex(i).x2+damt);
        J_approx = [arm_1(end,:)'-arm_0(end,:)', arm_2(end,:)'-arm_0(end,:)'] * (1/damt);
        delta_angs = J_approx \ [x_ee - x_ee_temp; y_ee - y_ee_temp];

        dist_grow1 = delta_angs(1);
        dist_grow2 = delta_angs(2);
        dist_grow = (dist_grow1^2 + dist_grow2^2)^.5;
        dist_extend = min(dist_grow,dist_max);
        dist_way = dist_extend/dist_max; % only go this fraction of the way TOWARD the random point
        
        for n_way = (1/n_check):(1/n_check):1
            x1_rand = tree.vertex(i).x1 + (n_way * dist_way * dist_grow1);
            x2_rand = tree.vertex(i).x2 + (n_way * dist_way * dist_grow2);
            
            collision = collision_avoidance(x1_rand, x2_rand, x_obs, y_obs);
            
            if collision==1
                break
            end
        end
        
        tree.vertex(i+1).x1 = x1_rand;
        tree.vertex(i+1).x2 = x2_rand;
        tree.vertex(i+1).x1_prev = tree.vertex(i).x1;
        tree.vertex(i+1).x2_prev = tree.vertex(i).x2;
        tree.vertex(i+1).index_prev = i;
        arm2 = forward_kinematics(tree.vertex(i+1).x1, tree.vertex(i+1).x2);
        x_ee_temp2 = arm2(end, 1);
        y_ee_temp2 = arm2(end, 2);
        
        if sqrt( (x_ee_temp2 - x_ee)^2 + (y_ee_temp2 - y_ee)^2 ) <= final_thresh
            ee = [x_ee_temp2 y_ee_temp2];
            check_break = true;
        end
    end
    
    if check_break
        break
    end
end

disp('Tree generated')
fprintf('Total epochs for RRT generation = %d \n', i);

% Find shortest path
if i < epoch

    path.state(1).x1 = tree.vertex(end).x1;
    path.state(1).x2 = tree.vertex(end).x2;
    pathIndex = tree.vertex(end).index_prev;

    j = 2;
    while (1)
        path.state(j).x1 = tree.vertex(pathIndex).x1;
        path.state(j).x2 = tree.vertex(pathIndex).x2;
        pathIndex = tree.vertex(pathIndex).index_prev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end
    path.state(end+1).x1 = x1_0;
    path.state(end).x2 = x2_0;

    for j = 1:1:(length(path.state) - 1)
        plot([path.state(j).x1; path.state(j+1).x1], [path.state(j).x2; path.state(j+1).x2], 'b', 'Linewidth', 3);

    end
    title([num2str(length(tree.vertex)) ' total nodes; path uses ' num2str(length(path.state)) ' nodes.'])
    plot(path.state(end).x1, path.state(end).x2, 'go', 'MarkerSize', 10, 'MarkerFaceColor','g')
    disp('Shortest path found');
    pause(1)

else
    disp('No path found. Increase number of iterations and retry');
    path = [];
    ee = [];
end










