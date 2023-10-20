x = -1.5:0.01:1.5;
y = -1.5:0.01:1.5;
z = zeros(length(x),length(y));

xo = 0;
yo = 0;

for i = 1:length(x)
    for j = 1:length(y)
        z(i,j) = obst_pen(x(i),y(j),xo,yo);
    end
end

mesh(x,y,z)

function y = obstacles_pen(pos2d,obstacles,np)
    x_wc = pos2d(1);
    y_wc = pos2d(2);
    y = 0;
    for i = 1:2:np
        xo = obstacles(i);
%         xo = 3;
        yo = obstacles(i+1);
%         yo = 0;
        y = max(y,obst_pen(x_wc,y_wc,xo,yo));
%         y = y + obst_pen(x_wc,y_wc,xo,yo);
    end
end

function y = obst_pen(x_wc,y_wc,x_obst,y_obst)
    d = sqrt((x_wc - x_obst)^2 + (y_wc - y_obst)^2);
    y = pen(d);
end

function y = pen(d)
    costmap_resolution = 0.1;
    wc_radius = 0.8;
    d = d - wc_radius - costmap_resolution/2;
    c = 50; % steepness
    k = 100; % max penalty
    y = k*sigmoid(-d,c);
end

function y = sigmoid(x,c)
    y = 1/(1+exp(-x*c));
%     y = 0.1;
end