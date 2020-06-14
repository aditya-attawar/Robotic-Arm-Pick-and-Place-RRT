function out = animate(tout, xout, flag, pos_obs, pos_source, pos_destination)

params;
clf;

for i = 1:1:length(pos_obs)
    fill(pos_obs{i}(:, 1), pos_obs{i}(:, 2), 'g')
    hold on
end

if (flag == 1)
    L = linspace(0, 2*pi, 5);
    box_x = pos_source(1)+0.1*cos(L-(pi/4))';
    box_y = pos_source(2)+0.1*sin(L-(pi/4))';
    plot(box_x, box_y, 'k', 'LineWidth', 2)
    hold on
    plot(pos_destination(1), pos_destination(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor','g')
    hold on
end

if (flag == 2)
    plot(pos_destination(1), pos_destination(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor','g')
    hold on
end

    
if length(tout)>1
    
    t = 0:0.02:tout(end);

    for n = 1:1:length(t)

        q1 = interp1(tout,xout(:,1),t(n),'PCHIP');
        q2 = interp1(tout,xout(:,2),t(n),'PCHIP');
        q3 = interp1(tout,xout(:,3),t(n),'PCHIP');
        th1 = q1;
        th2 = q1 + q2;
        th3 = q1 + q2 + q3;

        x0 = 0;
        y0 = 0;
        x1 = l1*sin(th1);
        x2 = x1 + l2*sin(th2);
        x3 = x2 + l3*sin(th3);

        y1 = -l1*cos(th1);
        y2 = y1 - l2*cos(th2);
        y3 = y2 - l3*cos(th3);
        
        if (flag == 2)
            L = linspace(0, 2*pi, 5);
            box_x = x3 + 0.1*cos(L-(pi/4));
            box_y = y3 + 0.1*sin(L-(pi/4));
        else
            box_x = [];
            box_y = [];
        end

%         x = [x0,x1,x2,box_x];
%         y = [y0,y1,y2,box_y];
        
        xa = [x0, x1];
        ya = [y0, y1];
        
        xb = [x1, x2];
        yb = [y1, y2];
        
        xc = [x2, x3];
        yc = [y2, y3];
        
        xd = [box_x];
        yd = [box_y];

        if (n == 1)
            p1 = plot(xa,ya,'r-','LineWidth',2);
            p2 = plot(xb,yb,'b','LineWidth',2);
            p3 = plot(xc,yc,'g','LineWidth',2);
            p4 = plot(xd,yd,'k','LineWidth',2);
            %axis image;
            hold on;
            axis([x0+[-2 2] y0+[-2 2]]);
            grid on;
            axis off;
        else
            set(p1,'Xdata',xa,'Ydata',ya);
            set(p2,'Xdata',xb,'Ydata',yb);
            set(p3,'Xdata',xc,'Ydata',yc);
            set(p4,'Xdata',xd,'Ydata',yd);
            axis([x0+[-2 2] y0+[-2 2]]);
            axis off;
            M(n) = getframe;
        end
        
        drawnow;
    end
    
    out = xout(end,:);
end
        
        
        
        
        
