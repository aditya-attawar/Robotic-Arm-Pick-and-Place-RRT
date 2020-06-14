function col = collision_avoidance(x1_fn, x2_fn, x3_fn, x_obs, y_obs)

arm = forward_kinematics(x1_fn, x2_fn, x3_fn);

a = arm(:,1);

in = inpolygon(arm(:,1), arm(:,2), x_obs, y_obs);

num = numel(a(in));

if (num == 0)
    col = 0;
else
    col = 1;
end