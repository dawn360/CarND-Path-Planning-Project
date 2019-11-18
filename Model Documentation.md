## Model Documentation

The goal of this path planner model was to complete at least 1 loop around the track
i.e 6946m as fast as possible while keeping to the 50MPH speed limit and avoiding jerks
and collusion.

The model approach implemented is similar to the approach explained in the Q&A section of the 
project instructions. 

For our new path is generated by concatenating a new set of points to the previous set.
We will describe how the new set of points is generated. We start off by adding to 
our new set with the last 2 points from our previous path. Next, we place 3 points 30m apart given the current/target lane. 
These points will be check marks, we'll fill in the gaps between them later. The `spline` library comes in play here. 
Once we load our new set of points onto the spline `s.set_points(ptsx,ptsy)` it would be able to tell us the `y` value corresponding a provided `x`
value. With this in mind, we just need to calculate the interval or spacing between our fill-in points between the check-points on the spline.
The formula for calculating the spacing between the points given the `target_distance` and `velocity` is `N = (target_dist/(.02*velocity/2.24))`.
Once we have the `N` value which is the number of points we can fit within that distance,we deduce the `x` points and utilize the spline to get the corresponding `y` values.

At this point, we have our new next set of points, however, to provide a smooth motion transition to our new points, we push back the new points onto our previous_points. The result is our final set of next point values.

The path planner implements a basic logic to slow down or speed up the car when needed. On each cycle we use the `sensor_fusion` data to determine if there is an car within 30m ahead of our future trajectory, we should begin to slow down else we'll speed up to the target speed of `48.5`

The path planner is able to change lanes to faster lanes depending on traffic. The model has 3 finite states `change_lane_left`, `change_lane_right` and `keep_lane`. Each state has an initial cost of `999` except the last state which has a cost of `998`. This makes the `keep_lane` the default state of the car. Whenever we encounter a car within 30m ahead of our future trajectory, we update the costs on each of these states as follows.
using the `sensor_fusion` data we check the cars in the `left_lane` and/or the `right_lane` to see if there is a clear path at least 30meters ahead 
or behind. if such a path is found then we reduce the cost of that lane by the `speed*distance_from_nearest_car` else cost of the lane remains unchanged. This approach helps us find a feasible lane to change to and also is optimized to pick the faster one since `speed` of cars in the target lane is also taken into account.

In the event of a lane change, the new lane reflects in the new set of points since the check-points are placed based on the target lane


    spread_waypoints = getXY(car_s+(30*i),(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);