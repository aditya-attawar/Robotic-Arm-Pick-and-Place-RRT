clear all;
close all;
clc;

bShowIK = true; % shows feasible points in joint space
rng(1000); % start with a particular "seed", for rand() function

disp('Start')

params;
disp('Parameters initilized')

fig1 = figure(1);
pos = get(figure(1), 'position');
set(figure(1), 'position', [pos(1:2) pos(3:4)])
axis auto

[pos_obs, pos_obs_boundary, pos_source, pos_destination] = set_configuration();
disp('Robot workspace created')

x_obs = [];
y_obs = [];
for i = 1:1:length(pos_obs_boundary)
    x_obs = [x_obs; pos_obs_boundary{i}(:, 1)];
    y_obs = [y_obs; pos_obs_boundary{i}(:, 2)];
end

x0_ee = pos_source(1);
y0_ee = pos_source(2);

x1_ee = pos_destination(1);
y1_ee = pos_destination(2);

disp('Begin 2 RRTs')

close(fig1)
for n1=1:30
    fig2 = figure(2); clf
    if bShowIK
        hold on;
        [th1, th2] = find_ik(pos_source(1), pos_source(2));
        bOK = 0*th1;
        for n=1:length(th1)
            col = collision_avoidance(th1(n), th2(n), x_obs, y_obs);
            if ~col
                bOK(n) = 1;
            end
        end
        fi = find(bOK);
        plot(th1(fi), th2(fi), 'k.');
        axis equal;
        axis([-pi pi -pi pi])
    end
    disp('RRT 1 begin - initial position to source')
    [states1, ee1] = RRT_new(0, 0, x0_ee, y0_ee, x_obs, y_obs);
    if ~isempty(states1)
        break
    end
end
disp('RRT 1 done')
pause(1);


xout1 = [];
for i = 1:1:length(states1.state)
    xout1(i, 1) = real(states1.state(i).x1);
    xout1(i, 2) = real(states1.state(i).x2);
end
xout1 = flip(xout1);

for n2=1:30
    fig3 = figure(3); clf
    if bShowIK
        hold on;
        [th1, th2] = find_ik(pos_destination(1), pos_destination(2));
        plot(th1, th2, 'k.');
        axis equal;
        axis([-pi pi -pi pi])
    end
    axis equal
    disp('RRT 2 begin - source to destination')
    [states2, ee2] = RRT_new(xout1(end, 1), xout1(end, 2), x1_ee, y1_ee, x_obs, y_obs);
    if ~isempty(states2)
        break
    end
end
disp('RRT 2 done')

xout2 = [];
for i = 1:1:length(states2.state)
    xout2(i, 1) = real(states2.state(i).x1);
    xout2(i, 2) = real(states2.state(i).x2);
end
xout2 = flip(xout2);

disp('Both RRTs done')


pause(2);

fig1 = figure(1);
axis equal
axis auto

flag = 1; % Object not yet held

tout1 = linspace(0, 2, length(states1.state))';

animate(tout1, xout1, flag, pos_obs, pos_source, pos_destination);
hold off

pause(1);

flag = 2; % Object held and being carried to destination

tout2 = linspace(0, 2, length(states2.state))';

animate(tout2, xout2, flag, pos_obs, pos_source, pos_destination);

disp('Animation done')


