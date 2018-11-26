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

  if(angle > pi()/4)
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


bool lane_switch_ok(vector<double> lane_db, double car_s)
{
	bool ok_lane = true;
	// check lane 0 cars
	for (int j=0; j<lane_db.size();j++)
	{
		if (abs(lane_db[j] - car_s) < 22)
		{
			ok_lane = false;
			break;
		}
	}

	return ok_lane;

}

int lane = 1;
double ref_vel = 0;

// Definition of each lane as distance d in frenet space.
// LANE 0
double d_0a = 2+4*0+2;
double d_0b = 2+4*0-2;
// LANE 1
double d_1a = 2+4*1+2;
double d_1b = 2+4*1-2;
// LANE 2
double d_2a = 2+4*2+2;
double d_2b = 2+4*2-2;

double brake_mulitpler = 1.0;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	// output packet
          	json msgJson;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

#ifdef SOL_1
          	// LEFT over from lesson material
          	double dist_inc = 0.3;
			for(int i = 0; i < 50; i++)
			{

				double next_s = car_s + (i+1) * dist_inc;
				double next_d = 6;
				vector<double> xy = getXY(next_s, next_d,
							map_waypoints_s,
							map_waypoints_x,
							map_waypoints_y);

				next_x_vals.push_back(xy[0]);
				next_y_vals.push_back(xy[1]);
			}
#endif

			// Get the previouos size of waypoints generated
			int prev_size = previous_path_x.size();

			// add this set of points to the last path waypoint
			if (prev_size > 0)
			{
				car_s = end_path_s;
			}

			//
			// solution closely follows sections Project.5 and Project.6 (video)
			//

			// flag to notify when to SLOW down/ Speed up
			bool too_close = false;

			/////////////////////////////////////////////////////////////////////////////
			// STEP 1. DO I need to change lanes?
			/////////////////////////////////////////////////////////////////////////////

			// Were going to create a data base to see what cars are in what lanes
			// this will be the basis of of a cost function
			// Basically the cost function is:
			// 1. All bias to stay i the left lane
			// 2. if there is no cars in the next lane, move to it.
			// 3. only allow one lane changes per time step

			// Database of cars at time = t
			vector<double> lane_0_cars;
			vector<double> lane_1_cars;
			vector<double> lane_2_cars;

			// organize the cars into lane DB, we will search for what lane is open
			// using these lists.
			for (int i=0; i<sensor_fusion.size(); i++)
			{
				float d = sensor_fusion[i][6];
				if (d < d_0a && d > d_0b)
				{
					lane_0_cars.push_back(sensor_fusion[i][5]);
				}
				else if (d < d_1a && d > d_1b)
				{
					lane_1_cars.push_back(sensor_fusion[i][5]);
				}
				else if (d < d_2a && d > d_2b)
				{
					lane_2_cars.push_back(sensor_fusion[i][5]);
				}
			}

			// Now check to see if there are any cars IN FRONT of you.
			for (int i=0; i<sensor_fusion.size(); i++)
			{
				float d = sensor_fusion[i][6];

				// there a car in front of me in my lane
				if (d < (2+4*lane+2) && d > (2+4*lane-2))
				{
					// Look at there distance in future time based on the current velocity of the car ahead.
					double vx = sensor_fusion[i][3];
					double vy = sensor_fusion[i][4];
					double check_speed = sqrt(vx*vx + vy*vy);
					double check_car_s = sensor_fusion[i][5];

					// Is the car in front going to be in front of you in the future
					// Is the car going slower
					check_car_s += ((double) prev_size*0.02*check_speed);

					// Is the car going to be < 30meters in front of you in the next time slice?
					if ((check_car_s > car_s) && (check_car_s-car_s) < 35)
					{

						//cout << (check_car_s-car_s) << endl;

						brake_mulitpler = 1.0;
						if ((check_car_s-car_s) < 20)
							brake_mulitpler = 4.0;

						// then we're going to be too close
						too_close = true;

						// Lets look at the lane alternatives
						int next_lane = lane;

						switch (lane)
						{
						// if I'm in lane 0, my cost says I can only move one lane.
						// check if lane 1 is clear, aka no car is within a specific distance around me
						// move to lane 1
						case 0:
							if (lane_switch_ok(lane_1_cars, car_s))
							next_lane = 1;
						break;
						// if I'm in lane 1, my cost says I can only move one lane.
						case 1:
							// check if lane 0 is clear, aka no car is within a specific distance around me
							// move to lane 0
							if (lane_switch_ok(lane_0_cars, car_s))
								next_lane = 0;

							// Cost also says bias to lane 0 first. But if lane 0 is blocked
							// check if lane 2 is clear, aka no car is within a specific distance around me
							// move to lane 2
							else if (lane_switch_ok(lane_2_cars, car_s))
								next_lane = 2;

							// Cost also says if there is an open lane (no cars detected in the scan)
							// that a priority, override the bias logic and move to the open lane.
							if (lane_0_cars.size() == 0)
								next_lane = 0;
							if (lane_2_cars.size() == 0)
								next_lane = 2;

						break;
						// if I'm in lane 2, my cost says I can only move one lane.
						// check if lane 1 is clear, aka no car is within a specific distance around me
						// move to lane 1
						case 2:
							if (lane_switch_ok(lane_1_cars, car_s))
							next_lane = 1;
						break;
						}

						// cost also says I shouldn't accelerate beyond the speed limit
						// let's limit when I can change lane based on current speed.
						// About 20% less.
						if (abs(next_lane - lane) <= 1 && car_speed <= 44)
							lane = next_lane;
					}
				}
			}

			/////////////////////////////////////////////////////////////////////////////
			// STEP 2. Should I change my speed?
			/////////////////////////////////////////////////////////////////////////////

			// After deciding if I need to change lanes,
			// control speed.
			if (too_close)
			{
				ref_vel -= (0.23 * brake_mulitpler);
			}
			else if (ref_vel < 49.5)
			{
				ref_vel += 0.22;
			}


			/////////////////////////////////////////////////////////////////////////////
			// STEP 3. What is m trajectory, aka path?
			/////////////////////////////////////////////////////////////////////////////

			//
			// Now lets look at the position of the car and calculate the
			// next set of waypoints SMOOTHLY into the lane I need to be in (can be the SAME lane)
			//
			vector<double> ptsx;
			vector<double> ptsy;

			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);

			// The start. Or someone braked in front of me, aka slammed the brakes...to 0 velocity.
			if (prev_size < 2)
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
				// add last set of waypoints.
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];

				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];

				ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);


			}

			// convert to frenet space and get waypoint 30,60,90 meters ahead of me in the next lane
			// (which maybe the same lane)
			vector<double> next_wp0 = getXY(car_s+30, (2+4*lane),
									map_waypoints_s,
									map_waypoints_x,
									map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s+60, (2+4*lane),
									map_waypoints_s,
									map_waypoints_x,
									map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s+90, (2+4*lane),
									map_waypoints_s,
									map_waypoints_x,
									map_waypoints_y);

			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);

			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);


			/////////////////////////////////////////////////////////////////////////////
			// STEP 4. Generate smooth path of waypoints, aka spline
			/////////////////////////////////////////////////////////////////////////////

			// reference to current car origin frame (pose = origin)
			for (int i=0; i< ptsx.size(); i++)
			{
				double shift_x = ptsx[i]-ref_x;
				double shift_y = ptsy[i]-ref_y;

				ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
				ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
			}


			// setup the spline and interpolate a new trajectory
			tk::spline s;
			s.set_points(ptsx, ptsy);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

			for (int i=0; i<prev_size; i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt( (target_x*target_x) + (target_y*target_y));

			double x_add_on = 0;

			// ignore last set of points and create the new trajectory against the current position of the car.
			// Again this reference section Project.6
			for(int i = 1; i < 50-prev_size; i++)
			{
				double N = (target_dist/(0.02*ref_vel/2.24));
				double x_point = x_add_on+target_x/N;
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

			}

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
