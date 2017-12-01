# Path Planning Model #
In this write up I will explain my path planning model and how I addressed all the problems and rubric points.

## Sumary ##
I divided the path planning model in the following problems.

- Smooth movement problem
- Prediction problem
- Speed problem
- Safety distance
- Safe lane change

## Smooth movement problem ##
The code to solve this problem in based on **Project Walkthrough and Q&A video**

The idea is to create control points ahead of the vehicle. These control points are created by the actual state of the vehicle (position and speed), and where should the vehicle go (the lane and the future position).
Then, using splines, we get a smooth and jerkless path. And using the previous path we get the continuity. Once we have this, we calculate a linear approximation to get the waypoints x and y in world coordinates.


There is a function in line 285


```c++
void calculate_path(int lane_index, double ref_vel, json j,
                    vector<double> map_waypoints_s,vector<double> map_waypoints_x,vector<double> map_waypoints_y,
                    vector<double> &waypoints_x, vector<double> &waypoints_y)
```

The goal of this function is to get a waypoints_x and waypoints_y, given the actual state of the vehicle, the desired lane and the desired ref.


One important rubric point for the project was the jerk. In my project, this was solved using splines, line 335.

## Prediction Problem ##
The simulator retrieves a sensor fusion vector with traffic information. But this is an incomplete information for the purpose of the planner. We need to predict where the traffic will be in the future in order to drive the vehicle on the highway.

This is solved by the function in line 416.
```c++
void predict_traffic(vector<vector<double>> &sensor_fusion, double ego_s, double ego_d, int prev_size, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
```

Using the x, y, vx and vy, I predict the future vehicle position by x = x + vx\*dt and y = y + vy\*dt. With the future x and future y, I can estimate predcited_s, predicted_d and predicted_lane_index.
All this information is calculated for each traffic vehicle in sensor fusion and added to its vector.

## Cost Function and state machine ##
Before I explain how I solved the other problems, I would like to explain the main approach.
My idea is to calculate new possible states. Each state is {new_lane, new_speed}. With two for loops, I get 9 new states:

- {Move Left, Brake}
- {Move Left, Keep speed}
- {Move Left, Accelerate}
- {Keep Lane, Brake}
- {Keep Lane, Keep speed}
- {Keep Lane, Accelerate}
- {Move Right, Brake}
- {Move Right, Keep speed}
- {Move Right, Accelerate}

Then, with these possible new lanes and new speed, I calculate their cost. 
The cost is calculated by 3 function cost. Each function has a cost and a weight. The state with the minimum cost is chosen, and with this new lane and speed, I calculate the new path. This can be found at line 566.

## Speed ##
The first cost function takes into account the speed. It is totally based on lesson 4, Point 11. The cost is minimal when the speed is the required speed. If the speed is higher than the legal speed, the cost is maximum. The function can be found at line 348.
```c++
float speed_cost(double speed,double speed_limit, double buffer_v, double stop_cost)
```

## Safety Distance ##
The second cost function takes into account the safety distance. With this cost function, we achieved that the vehicle detects that there is a vehicle in front of it and it should slow down in order to keep the distance. This function is also responsible for lane changing looking for a free lane. This function can be found at line 387.
```c++
double distance_cost(vector<vector<double>> sensor_fusion, double ego_s, double speed, double dt, int lane_index, int current_lane, double max_distance)
```
## Safe lane change ##
This is a very important cost function and it has the highest weight. This function avoids that the vehicle change to a lane with dangerous traffic. It detects if there is a vehicle in this possible lane, either in front or behind, and has a maximum cost if this is detected. What we want to obtain is that the vehicle only changes its lane if it is a really safe movement. This cost function also makes the vehicle change to a safe lane if it detects that a vehicle behind has higher speed. In this case, if it is possible, the ego vehicle changes to a free lane and lets the other vehicle pass faster.

## Discussion: Floating-point precision issue ##
I am a professional Unity Developer. I have developed videogames and right now I am a Vehicle Dynamics programmer in vehicle training simulators made with Unity3D.
I have detected that my planner behaves a little worse when the vehicle drives farther. 

There is a very known issue in Unity3D knows as floating-point precision issue. Transform class (vector3d position and quaternion rotation) are floats in Unity, not double. This is an accuracy problem with big scenes, that I faced myself several times and I have the feeling that that is the reason why the planner behaves a little worse when it goes farther.
Here there are some useful resources:

- https://www.youtube.com/watch?v=mXTxQko-JH0
- http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.471.7201&rep=rep1&type=pdf
- https://feedback.unity3d.com/suggestions/double-precision-for-worldspace-and-objects-and-vector-slash-math-64bit
