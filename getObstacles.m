function obstacles = getObstacles(obstacles,costmap,costmap_width)

    map = robotics.readBinaryOccupancyGrid(costmap,100);

    count = 1;
    
    for i = 1:costmap_width
        for j = 1:costmap_width
            value = robotics.getOccupancy(map,[i,j],"grid");
            disp(value)
            if 1==value
                xy = robotics.grid2world(map,[i,j]);
                obstacles(count:count+1) = xy;
                count = count + 2;
            end
        end
    end
end



