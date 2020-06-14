function [pos_obs, pos_obs_boundary, pos_source, pos_destination] = set_configuration()

animate([0; 0.0000001], [[0 0 0]; [0 0 0]], 0, {[0 0]}, [0 0], [0 0]);
hold on

obs_count = 1;
while(1)
    fprintf('Create obstacle %d (Click and drag on the workspace to create obstacle) \n', obs_count);
    obj = drawfreehand('Color', 'blue');
    hold on
    pos_obs_temp = obj.Position;
    obj.InteractionsAllowed = 'none';
    pos_obs{obs_count} = [pos_obs_temp(:, 1) pos_obs_temp(:, 2)];
    x_obs{obs_count} = pos_obs{obs_count}(:, 1);
    y_obs{obs_count} = pos_obs{obs_count}(:, 2);

    center{obs_count} = [mean(x_obs{obs_count}) mean(y_obs{obs_count})];
    pos_obs_boundary{obs_count} = [];
    l = 0.0707;
    for i = 1:1:length(pos_obs{obs_count})
        theta{obs_count} = atan2((pos_obs{obs_count}(i, 2) - center{obs_count}(2)), (pos_obs{obs_count}(i, 1) - center{obs_count}(1)));
        pos_obs_boundary_temp{obs_count}(i, 1) = (pos_obs{obs_count}(i, 1) + l*cos(theta{obs_count}));
        pos_obs_boundary_temp{obs_count}(i, 2) = (pos_obs{obs_count}(i, 2) + l*sin(theta{obs_count}));
    end
    h1 = plot(pos_obs_boundary_temp{obs_count}(:, 1), pos_obs_boundary_temp{obs_count}(:, 2), '--', 'Linewidth', 2, 'Color', 'k');
    hold on
    pause(1);
    [k, av] = convhull(pos_obs_boundary_temp{obs_count}(:, 1), pos_obs_boundary_temp{obs_count}(:, 2));
    pos_obs_boundary_conv{obs_count} = [pos_obs_boundary_temp{obs_count}(k, 1) pos_obs_boundary_temp{obs_count}(k, 2)];
    pos_obs_boundary{obs_count} = pos_obs_boundary_conv{obs_count};
    h2 = fill(pos_obs_boundary{obs_count}(:, 1), pos_obs_boundary{obs_count}(:, 2), 'g');
    hold on
    fprintf('Obstacle %d created \n', obs_count);
    while(1)
        response = input('To add another obstacle, press Y/N ', 's');
        if ((response == 'Y') | (response == 'y') | (response == 'N') | (response == 'n'))
            break;
        else
            fprintf('Please enter an appropriate input! \n');
            continue;
        end
    end
    if ((response == 'Y') | (response == 'y'))
        obs_count = obs_count + 1;
    else
        fprintf('Total number of obstacles created = %d \n', obs_count);
        break;
    end
    delete(h1);
end

while(1)
    fprintf('Place object at the desired location on the workspace \n');
    source = drawpoint;
    source.InteractionsAllowed = 'none';
    hold on
    pos_source = source.Position;
    L = linspace(0, 2*pi, 5);
    box_x = pos_source(1)+0.1*cos(L-(pi/4))';
    box_y = pos_source(2)+0.1*sin(L-(pi/4))';
    for i = 1:1:obs_count
        in1{i} = inpolygon(box_x, box_y, pos_obs_boundary{i}(:, 1), pos_obs_boundary{i}(:, 2));
        num1(i, 1) = numel(box_x(in1{i}));
    end
    if (num1(:, 1) == 0)
        plot(box_x, box_y, 'k', 'LineWidth', 2)
        hold on
        fprintf('Object placed at coordinates: %f, %f \n', pos_source(1), pos_source(2));
        break;
    else
        fprintf('Object is too close to obstacle! Please place it a little away. \n');
        continue;
    end
end

while(1)
    fprintf('Specify destination point on the workspace \n');
    destination = drawpoint;
    destination.InteractionsAllowed = 'none';
    hold on
    pos_destination = destination.Position;
    b = pos_destination(1);
    for i = 1:1:obs_count
        in2{obs_count} = inpolygon(pos_destination(1), pos_destination(2), pos_obs_boundary{i}(:, 1), pos_obs_boundary{i}(:, 2));
        num2(i, 1) = numel(b(in2{i}));
    end
    if (num2(:, 1) == 0)
        plot(pos_destination(1), pos_destination(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor','g')
        hold on
        fprintf('Destination created at coordinates: %f, %f \n', pos_destination(1), pos_destination(2));
        break;
    else
        fprintf('Destination is too close to obstacle! Please place it a little away. \n');
        continue;
    end
end

pause(0.5);