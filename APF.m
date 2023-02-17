classdef APF
    methods(Static)

        function cost = point(pos, obst)
            dist = norm(pos-obst);
            cost = exp(-dist^2);
        end

        function cost = circle(pos, c, r)
            dist = norm(pos-c);
            dist = max(0.01,dist-r);
            cost = 0.1/(dist^2);
        end

    end
end