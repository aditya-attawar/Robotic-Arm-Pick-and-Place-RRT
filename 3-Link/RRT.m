function [path, ee] = RRT(x1_0, x2_0, x3_0, x_ee, y_ee, x_obs, y_obs)

% Max iterations for motion planning
epoch = 10000;
fprintf('Max epochs set to %d \n', epoch);

% Max and min x and y values of task space
x1_min = 0;
x2_min = 0;
x3_min = 0;
x1_max = 2*pi;
x2_max = 2*pi;
x3_max = 2*pi;

% Acceptable threshold for final solution
final_thresh = 0.0707;

% Acceptable threshold for distance between states - "Visibility"
dist_thresh = 15 * pi/180;
fprintf('Visibility set to %f degrees \n', dist_thresh);


% Tree initalizations
tree.vertex(1).x1 = x1_0;
tree.vertex(1).x2 = x2_0;
tree.vertex(1).x3 = x3_0;
tree.vertex(1).x1_prev = x1_0;
tree.vertex(1).x2_prev = x2_0;
tree.vertex(1).x3_prev = x3_0;
tree.vertex(1).index_prev = 0;

% Plot congiguration
% figure(1);
hold on
grid on
plot3(x1_0, x2_0, x3_0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor','k')

% Tree generation
for i = 1:1:epoch
    
    if (mod(i, 100) == 0)
        fprintf('%d epochs done \n', i);
    end

    while(1)
        collision = 1;
        while(collision == 1)
            
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

            region_rand3 = 1 * rand; % joint space region
            if (region_rand3 >= 0.5)
                region_rand3 = 1;
            else
                region_rand3 = -1;
            end
            x3_rand = region_rand3 * (x3_max - x3_min) * rand; % Random x3 generation

            collision = collision_avoidance(x1_rand, x2_rand, x3_rand, x_obs, y_obs);
        end

        dist = Inf * ones(1,length(tree.vertex)); % Initializing distances corresponding to all vertices in the tree until current vertex to infinity
        for j = 1:length(tree.vertex) % Iterating from the first vertex to current vertex
            dist(j) = sqrt( (x1_rand - tree.vertex(j).x1)^2 + (x2_rand - tree.vertex(j).x2)^2 + (x3_rand - tree.vertex(j).x3)^2 ); % Distance between each vertex and randomly generated state
        end
        [dist_min, index_min] = min(dist); % Minimum distance and corresponding vertex
        
        if (dist_min > dist_thresh)
            continue
        else
            tree.vertex(i).x1 = x1_rand;
            tree.vertex(i).x2 = x2_rand;
            tree.vertex(i).x3 = x3_rand;
            tree.vertex(i).x1_prev = tree.vertex(index_min).x1;
            tree.vertex(i).x2_prev = tree.vertex(index_min).x2;
            tree.vertex(i).x3_prev = tree.vertex(index_min).x3;
            tree.vertex(i).index_prev = index_min;
            break
        end
        
    end

    plot3([tree.vertex(i).x1; tree.vertex(i).x1_prev], [tree.vertex(i).x2; tree.vertex(i).x2_prev], [tree.vertex(i).x3; tree.vertex(i).x3_prev], 'r');
    pause(0);
    
    arm = forward_kinematics(tree.vertex(i).x1, tree.vertex(i).x2, tree.vertex(i).x3);
    x_ee_temp = arm(end, 1);
    y_ee_temp = arm(end, 2);
    
    % Final states present within tolerance region
    if sqrt( (x_ee_temp - x_ee)^2 + (y_ee_temp - y_ee)^2 ) <= final_thresh
        ee = [x_ee_temp y_ee_temp];
        break
    end

end
disp('Tree generated')
fprintf('Total epochs for RRT generation = %d \n', i);

% Find shortest path
if i < epoch
    
    path.state(1).x1 = tree.vertex(end).x1;
    path.state(1).x2 = tree.vertex(end).x2;
    path.state(1).x3 = tree.vertex(end).x3;
    pathIndex = tree.vertex(end).index_prev;

    j = 2;
    while (1)
        path.state(j).x1 = tree.vertex(pathIndex).x1;
        path.state(j).x2 = tree.vertex(pathIndex).x2;
        path.state(j).x3 = tree.vertex(pathIndex).x3;
        pathIndex = tree.vertex(pathIndex).index_prev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end
    path.state(end+1).x1 = x1_0;
    path.state(end).x2 = x2_0;
    path.state(end).x3 = x3_0;

    for j = 1:1:(length(path.state) - 1)
        plot3([path.state(j).x1; path.state(j+1).x1], [path.state(j).x2; path.state(j+1).x2], [path.state(j).x3; path.state(j+1).x3], 'b', 'Linewidth', 3);
    end
    
    plot3(path.state(end).x1, path.state(end).x2, path.state(end).x3, 'go', 'MarkerSize', 10, 'MarkerFaceColor','g')
    disp('Shortest path found');
    
else
    disp('No path found. Increase number of iterations and retry');
end







