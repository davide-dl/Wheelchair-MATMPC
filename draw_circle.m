function draw_circle(c,r)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + c(1);
    yunit = r * sin(th) + c(2);
    plot(xunit, yunit)
end