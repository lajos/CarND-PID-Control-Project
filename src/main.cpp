#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "Twiddle.h"
#include <chrono>

// for convenience
using json = nlohmann::json;
using namespace std::chrono;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");
	if (found_null != std::string::npos) {
		return "";
	} else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}


int main() {
	uWS::Hub h;

	PID pid;
	double Kp, Ki, Kd;

	Kp = 0.114255;
	Ki = 0;
	Kd = 0.092131;

	//Kp = 0.255734;
	//Ki = 0;
	//Kd = 0.11638166;

	//Kp = 0.381867;
	//Ki = 0.00001;
	//Kd = 0.133599;

	//Kp = 0.45748;
	//Ki = -0.0008;
	//Kd = 0.133599;

	pid.Init(Kp, Ki, Kd);

	size_t c_frame = 0;

	Twiddle twiddle;
	double t_err = 0;
	size_t t_step = 0;
	twiddle.setParams(Kp, Ki, Kd);
	twiddle.setDParams(0.07, 0.001, 0.01);
#define TWIDDLE_FRAMES 4500

#define DO_TWIDDLE

	double last_timestamp = (double)duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() / 1000.0;

#ifdef USE_LATEST_UWS
	h.onMessage([&pid, &twiddle, &t_err, &t_step, &c_frame, &last_timestamp](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
#else
	h.onMessage([&pid, &twiddle, &t_err, &t_step, &c_frame, &last_timestamp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
#endif
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {
			auto s = hasData(std::string(data).substr(0, length));
			if (s != "") {
				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					double cte = std::stod(j[1]["cte"].get<std::string>());
					double speed = std::stod(j[1]["speed"].get<std::string>());
					double angle = std::stod(j[1]["steering_angle"].get<std::string>());
					double throttle = 0.3;
					double steer_value;

					double current_timestamp = (double)duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() / 1000.0;
					double dt = current_timestamp - last_timestamp;   // delta t in seconds
					last_timestamp = current_timestamp;

#ifdef DO_TWIDDLE
					pid.Kp = twiddle.p[0];
					pid.Ki = twiddle.p[1];
					pid.Kd = twiddle.p[2];
#endif

					pid.UpdateError(cte, dt);
					steer_value = pid.TotalError(dt);

					if (speed < throttle * 20) {		// wait for car to start moving 20% at throttle speed
						steer_value = 0;
						pid.i_error = 0;
					} else {
#ifdef DO_TWIDDLE
						t_err += pow(cte, 2);
						t_step += 1;
						if (t_step % TWIDDLE_FRAMES == 0) {
							twiddle.step(t_err / TWIDDLE_FRAMES);
							t_err = 0;
							pid.i_error = 0;
						}
						//std::cout << "frame:" << c_frame++ << std::endl;
#endif
					}

					// DEBUG
					//std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

					//if (steer_value < -1) steer_value = -1;
					//if (steer_value > 1) steer_value = 1;

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//std::cout << msg << std::endl;
#ifdef USE_LATEST_UWS
					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
#ifdef USE_LATEST_UWS
				ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data, size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

#ifdef USE_LATEST_UWS
	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
		ws->close();
		std::cout << "Disconnected" << std::endl;
	});
#else
	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});
#endif

	auto host = "127.0.0.1";
	int port = 4567;
	if (h.listen(host, port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
