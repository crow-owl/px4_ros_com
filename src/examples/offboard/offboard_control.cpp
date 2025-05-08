/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>  // for SensorDataQoS
#include <stdint.h>
#include <vector>
#include <array>
#include <cmath>
#include <functional>
#include <chrono>
#include <iostream>
#include <fstream>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// custom waypoints out in class
const std::vector<std::array<float, 3>> Waypoints = {
    {10.0f, 0.0f, -5.0f},
    {10.0f, 10.0f, -5.0f},
    {0.0f, 10.0f, -5.0f},
    {0.0f, 0.0f, -5.0f},
	{0.0f, 0.0f, 0.0f}
};

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry",
			rclcpp::SensorDataQoS(),  
			std::bind(&OffboardControl::odometry_callback, this, std::placeholders::_1)
		);
		offboard_setpoint_counter_ = 0;
		dist_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/debug/dist", 10);
		wp_index_publisher_ = this->create_publisher<std_msgs::msg::UInt32>("/debug/current_wp", 10);

		csv_file_.open("flight_log.csv", std::ios::out);
	    	csv_file_ << "time,pos_x,pos_y,pos_z,set_x,set_y,set_z\n";


		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
			waypoint_counter();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}

		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}
	~OffboardControl() override { if (csv_file_.is_open()) {csv_file_.close();}}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
	px4_msgs::msg::VehicleOdometry current_odom_;

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_publisher_;
	rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr wp_index_publisher_;


	

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void waypoint_counter();
	void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

	size_t max_wp_count_ = Waypoints.size(); 
	size_t current_wp_count_ = 0; 
	std::ofstream csv_file_;
};

/**
 * @brief Counting waypoints
 */
 void OffboardControl::waypoint_counter()
 {
	 if (current_wp_count_ >= max_wp_count_) {
		 return;  
	 }
 
	 const auto& wp = Waypoints[current_wp_count_];
	 float dx = current_odom_.position[0] - wp[0];
	 float dy = current_odom_.position[1] - wp[1];
	 float dz = current_odom_.position[2] - wp[2];
	 float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
 
	// publish distance
	std_msgs::msg::Float32 dist_msg;
	dist_msg.data = dist;
	dist_publisher_->publish(dist_msg);

	// publish current waypoint index
	std_msgs::msg::UInt32 wp_msg;
	wp_msg.data = static_cast<uint32_t>(current_wp_count_);
	wp_index_publisher_->publish(wp_msg);

	 if (dist < 0.5f) {
		 if (current_wp_count_ < max_wp_count_ - 1) {
			 current_wp_count_++;
			 RCLCPP_INFO(this->get_logger(), "Reached waypoint. Advancing to [%zu]", current_wp_count_);
		 } else {
			 RCLCPP_INFO(this->get_logger(), "Final waypoint reached. Holding position.");
		 }
	 }
 }
 
 void OffboardControl::odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
	current_odom_ = *msg;
  }

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
 void OffboardControl::publish_trajectory_setpoint()
 { 
	 auto wp = Waypoints[current_wp_count_];
 
	 TrajectorySetpoint msg{};
	 msg.position = {wp[0], wp[1], wp[2]};
	 msg.yaw = 0.0f;  
	 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
 
	 trajectory_setpoint_publisher_->publish(msg);
 
	 RCLCPP_INFO(this->get_logger(), "Sent waypoint [%zu]: (%.2f, %.2f, %.2f)",
	 current_wp_count_, wp[0], wp[1], wp[2]);
	 
	 csv_file_ << this->get_clock()->now().nanoseconds() << ","  // nanoseconds
	 << current_odom_.position[0] << ","
	 << current_odom_.position[1] << ","
	 << current_odom_.position[2] << ","
	 << wp[0] << "," << wp[1] << "," << wp[2] << "\n";

	csv_file_.flush();


 }

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
