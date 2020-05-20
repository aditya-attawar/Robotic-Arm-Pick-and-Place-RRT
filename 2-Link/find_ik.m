function [th1, th2rel] = find_ik(xee,yee)
params;

x2 = xee;
y2 = yee;
r2 = (x2.^2 + y2.^2).^.5;

a = l1; b = l2; c = r2;
theta_0_to_2 = atan2(y2,x2) + pi/2;
tha = acos((b.^2+c.^2-a.^2)./(2*b.*c));
thb = acos((a.^2+c.^2-b.^2)./(2*a.*c));
thc = acos((a.^2-c.^2+b.^2)./(2*a.*b));

% two solutions exist: +tha and -tha
th1a = theta_0_to_2 + thb;
th1b = theta_0_to_2 - thb;
th2a = th1a - pi + thc;
th2b = th1b + pi - thc;

th1 = [th1a, th1b];
th2 = [th2a, th2b];
th2rel = mod((th2-th1)+pi,2*pi)-pi;
th1 = mod(th1+pi,2*pi)-pi;