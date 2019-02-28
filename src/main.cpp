#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  //文件读取方式，之后可以看c++prime
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  //读取地图信息
  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  //define start lane
  int lane=1;
  // reference volecity
  double ref_vel=0;  //mph

  //上面两个参数需要传入
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int prev_size = previous_path_x.size();
          
          bool too_close(false);
          bool turn_left(true);
          bool turn_right(true);
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            float d=sensor_fusion[i][6];
            if(d<(2+lane*4+2) && d>(2+lane*4-2))
            {
              double vx=sensor_fusion[i][3];
              double vy=sensor_fusion[i][4];
              double check_speed=sqrt(vx*vx+vy*vy);
              double check_car_s=sensor_fusion[i][5];

              check_car_s+=((double)prev_size*0.02*check_speed);

              //跟之前轨迹产生的点比较
              //前车在之前轨迹的15米范围内
              if((check_car_s>end_path_s)&& ((check_car_s-end_path_s)<15))
              {
                too_close=true;
              }
            }
            else if(d<4*lane && d>(4*lane-4))
            {
              double vx=sensor_fusion[i][3];
              double vy=sensor_fusion[i][4];
              double check_speed=sqrt(vx*vx+vy*vy);
              double check_car_s=sensor_fusion[i][5];

              check_car_s+=((double)prev_size*0.02*check_speed);
              //转向时距离不够
              //旁边车道的车在未来在的范围，不要超过我现在的
              if(((check_car_s-car_s)>0)&& ((check_car_s-car_s)<30))
              {
                turn_left=false;
              }
            }
            else if(d<(4*lane+8) && d>(4*lane+4))
            {
              double vx=sensor_fusion[i][3];
              double vy=sensor_fusion[i][4];
              double check_speed=sqrt(vx*vx+vy*vy);
              double check_car_s=sensor_fusion[i][5];

              check_car_s+=((double)prev_size*0.02*check_speed);
              if(((check_car_s-car_s)>0)&& ((check_car_s-car_s)<30))
              {
                turn_right=false;
              }
            }

          }

          if(too_close){
            //1，2车道，可左转
            if(lane>0 && turn_left) lane--;
            else if(lane<2 && turn_right) lane++;
            else ref_vel-=0.224;
          }
          else if(ref_vel<49.5)
          {
             ref_vel+=0.224;
          }
          

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          /*/  简单创建一条匀速的轨迹
          double dist_inc = 0.4;
          vector<double> start({0,0.4/0.02,0});
          vector<double> end({60,0.4/0.02,0});
          vector<double> result=JMT(start,end,60/(0.4/0.02));
          for (int i = 0; i < 50; ++i) {
            double next_s = car_s + trajectory_pos(result,i*0.02);
            double next_d = 6;
            vector<double> xy=getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          */

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x=car_x;
          double ref_y=car_y;
          double ref_yaw=deg2rad(car_yaw);

          //前导点不足两个时
          if(prev_size<5){
            double prev_car_x=car_x-cos(car_yaw);
            double prev_car_y=car_y-sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          //有足够的前导点，选择前面轨迹最后两个
          else{
            ref_x=previous_path_x[4];
            ref_y=previous_path_y[4];

            double ref_prev_x=previous_path_x[3];
            double ref_prev_y=previous_path_y[3];
            //为后面坐标转换做准备
            ref_yaw=atan2(ref_y-ref_prev_y,ref_x-ref_prev_x);

            ptsx.push_back(ref_prev_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_prev_y);
            ptsy.push_back(ref_y);
          }

          //增加远处的点在frenet中，有利于曲线插值
          //way_points 路标点
          vector<double> next_wp0= getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1= getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2= getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          
          for(int i = 0; i < ptsx.size(); i++)
          {
            //从世界（地图）坐标转换到汽车坐标系中
            double shift_x=ptsx[i]-ref_x;
            double shift_y=ptsy[i]-ref_y;

            ptsx[i]=shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
            ptsy[i]=shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
          }

          //spline初始化
          tk::spline s;
          s.set_points(ptsx,ptsy);
         
          //放回未消耗的路径点
          
          for(int i = 0;i<5&&i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          

          double target_x=30;
          double target_y=s(target_x);
          double target_dist=sqrt(target_x*target_x+target_y+target_y);

          double N=target_dist/(0.02*ref_vel/2.24); //mile per hour --> meter per second /2.24
          double x_increment=target_x/N;  //0.02内移动的距离

          for(int i=0;i<(50-fmin(5,prev_size));i++){
            double x_point=(i+1)*x_increment;
            double y_point=s(x_point);

            //汽车坐标系中的点
            double x_ref=x_point;
            double y_ref=y_point;
            //转换到世界坐标
            x_point=x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
            y_point=x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

            x_point+=ref_x;
            y_point+=ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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