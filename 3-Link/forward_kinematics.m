function arm_fn = forward_kinematics(x1_fn, x2_fn, x3_fn)

params;

th1 = x1_fn;
th2 = x1_fn + x2_fn;
th3 = x1_fn + x2_fn + x3_fn;

x_O = 0;
y_O = 0;

x_A = l1*sin(th1);
x_B = x_A + l2*sin(th2);
x_C = x_B + l3*sin(th3);

y_A = -l1*cos(th1);
y_B = y_A - l2*cos(th2);
y_C = y_B - l3*cos(th3);

x_arm1 = linspace(x_O, x_A, 100);
x_arm2 = linspace(x_A, x_B, 100);
x_arm3 = linspace(x_B, x_C, 100);

y_arm1 = linspace(y_O, y_A, 100);
y_arm2 = linspace(y_A, y_B, 100);
y_arm3 = linspace(y_B, y_C, 100);

x_arm_fn = [x_arm1, x_arm2, x_arm3];
y_arm_fn = [y_arm1, y_arm2, y_arm3];

arm_fn = [x_arm_fn' y_arm_fn'];