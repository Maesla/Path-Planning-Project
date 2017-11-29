#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#define LANE_WIDTH 4
#define NEXT_WAYPOINT_TIME 0.02
#define WAYPOINT_COUNT 50
#define METER_PER_SECOND2MILLES_PER_HOUR 2.23694
#define ACCELERATION 0.224
#define MAX_SPEED 49.5

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/2)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

double get_lane_d_center(int lane_index)
{
  float lane_width = LANE_WIDTH;
  float half_lane_width = 0.5*lane_width;
  return half_lane_width+lane_width*lane_index;
}

bool is_in_lane(float d, int lane_index)
{
  float lane_width = LANE_WIDTH;
  float half_lane_width = 0.5*lane_width;
  return (d < (half_lane_width+lane_width*lane_index+half_lane_width) && d > (half_lane_width+lane_width*lane_index-half_lane_width));
}

int get_lane_index(double d)
{
  return ((int)d/LANE_WIDTH);
  //return (int)(((d - LANE_WIDTH*0.5) / LANE_WIDTH) + LANE_WIDTH*0.5);
  /*for(int i = -3; i < 3; i++)
  {
    if (is_in_lane(d, i))
      return i;
  }

  return -10;*/
}


bool is_in_front(double ego_vehicle_s, double check_s, double distance)
{
  return (check_s > ego_vehicle_s) && ((check_s-ego_vehicle_s) < distance);
}

bool is_obstacle_too_close_in_front(vector<vector<double>> sensor_fusion, int lane_index, double ego_vehicle_s, int prev_size)
{
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    float d = sensor_fusion[i][6];
    if (is_in_lane(d, lane_index))
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];

      //cout << "Car s: " << check_car_s << " Speed: " << check_speed << " Prev_size: " << prev_size << endl;
      check_car_s += ((double)prev_size*NEXT_WAYPOINT_TIME*check_speed);
      //cout << "New s:" << check_car_s << endl;

      if (is_in_front(ego_vehicle_s, check_car_s, 30))
      {
        return true;
      }
    }
  }
  return false;
}


void calculate_auxiliar_path_connecting_with_previous(double car_s,double car_x, double car_y, double car_yaw, int prev_size,
             vector<double> previous_path_x, vector <double> previous_path_y,
             double &ref_x, double &ref_y, double &ref_yaw, vector<double> &ptsx, vector<double> &ptsy)
{
  ref_x = car_x;
  ref_y = car_y;
  ref_yaw = deg2rad(car_yaw);

  if(prev_size < 2)
  {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else
  {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
}

void calculate_auxiliar_path_looking_ahead(double ego_vehicle_s, int lane_index,
                                           vector<double> map_waypoints_s,vector<double> map_waypoints_x,vector<double> map_waypoints_y,
                                           vector<double> &ptsx, vector<double> &ptsy)
{
  double d = get_lane_d_center(lane_index);

  double look_ahead_distance = 50;

  for (int i = 1; i <=3; i++)
  {
    vector<double> next_waypoint = getXY(ego_vehicle_s + i*look_ahead_distance, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_waypoint[0]);
    ptsy.push_back(next_waypoint[1]);

  }
 }

void transform_to_local(double ref_x, double ref_y, double ref_yaw, double &x, double &y)
{
  double shift_x = x - ref_x;
  double shift_y = y - ref_y;

  x = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
  y = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
}

void transform_to_world(double ref_x, double ref_y, double ref_yaw, double &x, double &y)
{
  double x_ref = x;
  double y_ref = y;

  x = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
  y = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

  x += ref_x;
  y += ref_y;
}

void calculate_path(double ref_x, double ref_y, double ref_yaw, double ref_vel, int prev_size, tk::spline s,
            vector<double> &next_x_vals, vector<double> &next_y_vals)
{
  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x+target_y*target_y);

  double x_add_on = 0;

  for (int i = 1; i <= WAYPOINT_COUNT - prev_size; i++)
  {
    double N = (target_dist/(NEXT_WAYPOINT_TIME*ref_vel/METER_PER_SECOND2MILLES_PER_HOUR));
    double x_point = x_add_on+(target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    transform_to_world(ref_x, ref_y, ref_yaw, x_point, y_point);

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

void calculate_path(int lane_index, double ref_vel, json j,
                    vector<double> map_waypoints_s,vector<double> map_waypoints_x,vector<double> map_waypoints_y,
                    vector<double> &waypoints_x, vector<double> &waypoints_y)
{

  double car_x = j[1]["x"];
  double car_y = j[1]["y"];
  double car_s = j[1]["s"];
  double car_d = j[1]["d"];
  double car_yaw = j[1]["yaw"];
  double car_speed = j[1]["speed"];

  // Previous path data given to the Planner
  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector <double> previous_path_y = j[1]["previous_path_y"];
  // Previous path's end s and d values
  double end_path_s = j[1]["end_path_s"];
  double end_path_d = j[1]["end_path_d"];

  int prev_size = previous_path_x.size();

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x;
  double ref_y;
  double ref_yaw;

  calculate_auxiliar_path_connecting_with_previous(car_s,car_x, car_y, car_yaw, prev_size,
               previous_path_x, previous_path_y,
               ref_x, ref_y, ref_yaw, ptsx, ptsy);



  calculate_auxiliar_path_looking_ahead(car_s, lane_index,
                                             map_waypoints_s, map_waypoints_x, map_waypoints_y,
                                             ptsx, ptsy);


  for(int i = 0; i < ptsx.size(); i++)
  {
    double x = ptsx[i];
    double y = ptsy[i];
    transform_to_local(ref_x, ref_y, ref_yaw, x, y);
    ptsx[i] = x;
    ptsy[i] = y;

  }


  tk::spline s;
  s.set_points(ptsx, ptsy);


  for(int i = 0; i < previous_path_x.size(); i++)
  {
    waypoints_x.push_back(previous_path_x[i]);
    waypoints_y.push_back(previous_path_y[i]);
  }

  calculate_path(ref_x, ref_y, ref_yaw, ref_vel, prev_size,s,waypoints_x, waypoints_y);
}

float speed_cost(double speed,double speed_limit, double buffer_v, double stop_cost)
{
  if (speed > speed_limit)
    return 1;

  double target_speed = speed_limit - buffer_v;
  if (speed < target_speed)
  {
    return stop_cost*((target_speed - speed)/target_speed);
  }
  else
  {
    return (speed-target_speed)/buffer_v;
  }

}

float lane_obstacle_cost(vector<vector<double>> sensor_fusion, int lane_index, double ego_vehicle_s, int prev_size)
{
  if (is_obstacle_too_close_in_front(sensor_fusion, lane_index, ego_vehicle_s, prev_size))
      return 1;

  return 0;
}

float lane_change_cost(int new_lane, int current_lane)
{
  if (new_lane == current_lane)
    return 0;

  return 1;
}

double neighbor_detector(vector<vector<double>> sensor_fusion, double ego_s, double speed, double dt, int current_lane, double threshold)
{
  double next_s = ego_s + speed*dt;
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    double predicted_s = sensor_fusion[i][7];
    int traffic_lane = sensor_fusion[i][9];
    bool is_neighbor = current_lane != traffic_lane;
    double distance = predicted_s - next_s;

    if(is_neighbor && (fabs(distance) < threshold))
    {
      cout << "Neightbour in lane: " << traffic_lane << " at distance: " << distance << endl;
    }
  }

  return 0.0;
}

double change_lane_cost(vector<vector<double>> sensor_fusion, double ego_s, double speed, double dt, int current_lane, int next_lane,  double threshold)
{
  double next_s = ego_s + speed*dt;
  bool is_lane_change = current_lane != next_lane;

  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    double predicted_s = sensor_fusion[i][7];
    int traffic_lane = sensor_fusion[i][9];
    bool is_neighbor = next_lane == traffic_lane;
    double distance = predicted_s - next_s;

    if(is_neighbor && (fabs(distance) < threshold))
    {
      cout << "Neightbour in lane: " << traffic_lane << " at distance: " << distance << endl;
      return 1.0;
    }
  }

  return 0.0;
}

double distance_cost(vector<vector<double>> sensor_fusion, double ego_s, double speed, double dt, int lane_index, int current_lane, double max_distance)
{
  double next_s = ego_s + speed*dt;
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    int traffic_lane = sensor_fusion[i][9];
    double predicted_s = sensor_fusion[i][7];

    if (traffic_lane == lane_index)
    {
      double distance = predicted_s - next_s;

      if (distance >  0 && distance < max_distance)
      {
        double cost = 1 - distance/max_distance;
        //cout << "!!!!!!: " << cost << " speed: " << speed << endl;
        return cost;
      }
    }
  }

  if (lane_index != current_lane)
    return 0.5;

  return 0;
}



void print_sensor_fusion(vector<vector<double>> &sensor_fusion, double ego_s, double ego_d, int prev_size, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
  //cout << "**************************" << endl;
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    double id = sensor_fusion[i][0];
    double x = sensor_fusion[i][1];
    double y = sensor_fusion[i][2];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double s = sensor_fusion[i][5];
    double d = sensor_fusion[i][6];


    double dt = (double)prev_size*NEXT_WAYPOINT_TIME;
    double next_x = x+vx*dt;
    double next_y = y+vy*dt;
    double speed = sqrt(vx*vx+vy*vy);

    int myWaypoint = ClosestWaypoint(x,y,map_waypoints_x, map_waypoints_y);
    int nextWaypoint = ClosestWaypoint(next_x,next_y,map_waypoints_x, map_waypoints_y);
    double lane_dx = map_waypoints_x[nextWaypoint] - map_waypoints_x[myWaypoint];
    double lane_dy = map_waypoints_y[nextWaypoint] - map_waypoints_y[myWaypoint];


    double yaw = atan2(vy,vx) - atan2(lane_dy, lane_dx);
    double v_s = speed*cos(yaw);
    double v_d = speed*cos(yaw);

    double predicted_s = s + v_s*dt;
    double predicted_d = d + v_d*dt;
    //vector<double> next_s_d = getFrenet(next_x, next_y, yaw, map_waypoints_x, map_waypoints_y);

    int lane_index = get_lane_index(d);
    int predicted_lane_index(predicted_d);

    sensor_fusion[i].push_back(predicted_s);
    sensor_fusion[i].push_back(predicted_d);
    sensor_fusion[i].push_back(lane_index);
    sensor_fusion[i].push_back(predicted_lane_index);



    double diff_s = predicted_s - ego_s;
    double diff_d = ego_d - d;


/*
      cout << "Id: " << id << " x: " << x << " y: " << y << " vx: " << vx << " vy: " << vy
          << " s: " << s << " d: " << d << " lane: " << lane_index
          << " diff s: " << diff_s << " diff d: " << diff_d << endl;
*/
  }
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  //double ref_vel = 49.5;//mph
  double ref_vel = 0.1;//mph


  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector <double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.

          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];


          	int prev_size = previous_path_x.size();


          	if (prev_size > 0)
          	{
          	  car_s = end_path_s;
          	}

            double dt = (double)prev_size*NEXT_WAYPOINT_TIME;


          	print_sensor_fusion(sensor_fusion, car_s, car_d, prev_size, map_waypoints_x, map_waypoints_y);
          	//neighbor_detector(sensor_fusion, car_s, car_speed, NEXT_WAYPOINT_TIME, lane, 10);



          	/*bool too_close = is_obstacle_too_close_in_front(sensor_fusion, lane, car_s, prev_size);
          	if (too_close)
          	{
          	  if (lane > 0)
          	  {
          	    lane = 0;
          	  }
          	  ref_vel -= ACCELERATION;
          	}
          	else if (ref_vel < MAX_SPEED)
          	{
          	  ref_vel += ACCELERATION;
          	}*/

          	double min_cost = 999999999.0;
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            int selected_lane;
            double selected_speed;
            dt = NEXT_WAYPOINT_TIME;
            for (int lane_change = -1; lane_change <= 1; lane_change++)
            {
              int new_lane = lane + lane_change;

            for (int acceleration_change = -2; acceleration_change <= 2; acceleration_change++)
            {

               double new_ref_vel = ref_vel + acceleration_change*ACCELERATION;

               if (new_ref_vel > 0.01 && new_lane >= 0 && new_lane <=2)
               {

                 double speed_cost_value = speed_cost(new_ref_vel,MAX_SPEED, 5, 0.7);
                 double distance_cost_value =  distance_cost(sensor_fusion, car_s, new_ref_vel, dt, new_lane, lane, 30);
                 double change_lane_cost_value = change_lane_cost(sensor_fusion, car_s, car_speed, NEXT_WAYPOINT_TIME, lane, new_lane, 30.0);


                 //double distance_cost_value = distance_cost(car_s, new_ref_vel, )

                 double cost = speed_cost_value + 100.0*distance_cost_value + 10000.0*change_lane_cost_value;
                 cout << "Cost: " << cost << endl;

                 if (cost < min_cost)
                 {
                   min_cost = cost;
                   //next_x_vals = aux_x;
                   //next_y_vals = aux_y;
                   selected_lane = new_lane;
                   selected_speed = new_ref_vel;
                 }
               }
              }
            }

            lane = selected_lane;
            ref_vel = selected_speed;
            //cout << "Selected speed: " << ref_vel <<  endl;
            calculate_path(lane, ref_vel, j,
                                           map_waypoints_s,map_waypoints_x,map_waypoints_y,
                                           next_x_vals, next_y_vals);


/*
            for (int lane_change = -1; lane_change <= 1; lane_change++)
            {
              for (int acceleration_change = -1; acceleration_change <= 1; acceleration_change++)
              {
                //int new_lane = lane;// + lane_change;
                int new_lane = lane + lane_change;

                double new_ref_vel = ref_vel + acceleration_change*ACCELERATION;

                if (new_ref_vel > 0.01 && new_lane >= 0 && new_lane <=2)
                {
                  vector<double> aux_x;
                  vector<double> aux_y;
                  calculate_path(new_lane, new_ref_vel, j,
                                 map_waypoints_s,map_waypoints_x,map_waypoints_y,
                                 aux_x, aux_y);

                  double speed_cost_value = speed_cost(new_ref_vel,MAX_SPEED, 5, 0.7);
                  double lane_cost = lane_obstacle_cost2(sensor_fusion, new_lane, car_s, prev_size);
                  double change_lane_cost_value = lane_change_cost(new_lane, lane);
                  double exp_cost = experimental_cost(sensor_fusion, aux_x, aux_y);

                  //double cost = speed_cost_value + 10*lane_cost;// + change_lane_cost_value;
                  double cost = 10*exp_cost + speed_cost_value;

                  if (cost < min_cost)
                  {
                    min_cost = cost;
                    next_x_vals = aux_x;
                    next_y_vals = aux_y;
                    selected_lane = new_lane;
                    selected_speed = new_ref_vel;

                    if (change_lane_cost_value > 0)
                    {
                      selected_speed = ref_vel - 2*ACCELERATION;
                      cout << "HOLA" << endl;
                    }
                    else
                    {
                      selected_speed = new_ref_vel;

                    }

                  }
                }
              }
            }
*/


            //cout << "Lane: " << lane << " Ref vel: " << ref_vel << endl;


            /*
            vector<double> next_x_vals;
            vector<double> next_y_vals;
          	calculate_path(lane, ref_vel, j,
          	                    map_waypoints_s,map_waypoints_x,map_waypoints_y,
          	                    next_x_vals, next_y_vals);
          	                    */

          	json msgJson;


          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
