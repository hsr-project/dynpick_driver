/*
 * Copyright (c) 2017, Tokyo Opensource Robotics Kyokai Association
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include	<stdio.h>
#include	<fcntl.h>
#include	<time.h>
#include	<termios.h>
#include	<string.h>
#include	<unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#define true		1
#define false		0

const int kDataLength = 27;

int SetComAttr(int fdc)
	{
	int			n;

	struct termios	term;


	// Set baud rate
	n = tcgetattr(fdc, &term);
	if (n < 0)
		goto over;

	bzero(&term, sizeof(term));

	term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
	term.c_iflag = IGNPAR;
	term.c_oflag = 0;
	term.c_lflag = 0;/*ICANON;*/
 
	term.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
	term.c_cc[VQUIT]    = 0;     /* Ctrl-? */
	term.c_cc[VERASE]   = 0;     /* del */
	term.c_cc[VKILL]    = 0;     /* @ */
	term.c_cc[VEOF]     = 4;     /* Ctrl-d */
	term.c_cc[VTIME]    = 0;
	term.c_cc[VMIN]     = 0;
	term.c_cc[VSWTC]    = 0;     /* '?0' */
	term.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
	term.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	term.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	term.c_cc[VEOL]     = 0;     /* '?0' */
	term.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	term.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	term.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	term.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	term.c_cc[VEOL2]    = 0;     /* '?0' */

//	tcflush(fdc, TCIFLUSH);
	n = tcsetattr(fdc, TCSANOW, &term);
over :
	return (n);
	}

void GetWrenchTransformMatrix(rclcpp::Node::SharedPtr& node,
                              Eigen::Matrix3d& dst_rot_part,
                              Eigen::Matrix3d& dst_linear_part) {
  // initialize
  dst_rot_part = Eigen::Matrix3d::Identity();
  dst_linear_part = Eigen::Matrix3d::Zero();

  // get transform parameter
  const auto transform_vec = node->declare_parameter<std::vector<double>>("frame_to_sensor", {});
  if (transform_vec.empty()) {
    // 設定されていないだけなので，何もせずに抜ける
  } else if (transform_vec.size() == 3) {
    // x, y, z
    Eigen::Matrix3d translation;
    translation << 0.0, -transform_vec[2], transform_vec[1],
                   transform_vec[2], 0.0, -transform_vec[0],
                   -transform_vec[1], transform_vec[0], 0.0;
    dst_linear_part = translation * dst_rot_part;
  } else if (transform_vec.size() == 7) {
    // x, y, z, qx, qy, qz, qw
    dst_rot_part = Eigen::Quaterniond(transform_vec[6],
                                      transform_vec[3],
                                      transform_vec[4],
                                      transform_vec[5]).toRotationMatrix();
    Eigen::Matrix3d translation;
    translation << 0.0, -transform_vec[2], transform_vec[1],
                   transform_vec[2], 0.0, -transform_vec[0],
                   -transform_vec[1], transform_vec[0], 0.0;
    dst_linear_part = translation * dst_rot_part;
  } else {
    RCLCPP_WARN(node->get_logger(),
                "frame_to_sensor supports [x, y, z] or [x, y, z, qx, qy, qz, qw].");
  }
}

// センサ内の移動平均フィルタの点数を設定するコマンドを取得する
// WDF-6F ver2.22では点数を1, 2, 4, 8から選択可能
// 生値のノイズが大きめなのでデフォルトを8点とする
std::string GetMovingAverageSetting(rclcpp::Node::SharedPtr& node) {
  const auto num_of_points = node->declare_parameter<int32_t>("num_of_moving_average_points", 8);
  switch (num_of_points) {
    case 1:
      return std::string("1F");
    case 2:
      return std::string("2F");
    case 4:
      return std::string("4F");
    case 8:
    default:
      return std::string("8F");
  }
}

int main(int argc, char **argv) {
    int fdc;

    fdc = -1;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dynpick_driver_node");
    const auto devname = node->declare_parameter<std::string>("device", "/dev/ttyUSB0");
    const auto frame_id = node->declare_parameter<std::string>("frame_id", "/sensor");
    const auto rate = node->declare_parameter<double>("rate", 1000.0);

    // frame_to_sensor
    Eigen::Matrix3d linear_frame_to_sensor;
    Eigen::Matrix3d rot_frame_to_sensor;
    GetWrenchTransformMatrix(node, rot_frame_to_sensor, linear_frame_to_sensor);

    auto pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>("force", 1000);

    // Open COM port
    RCLCPP_INFO(node->get_logger(), "Open %s", devname.c_str());

    fdc = open(devname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fdc < 0) {
        RCLCPP_ERROR(node->get_logger(), "could not open %s\n", devname.c_str());
        return -1;
    }

    // Obtain sampling rate
    RCLCPP_INFO(node->get_logger(), "Sampling time = %f ms\n", 1.0/rate);

    // Set baud rate of COM port
    SetComAttr(fdc);

    // Configure moving average
    auto moving_average_setting = GetMovingAverageSetting(node);
    write(fdc, moving_average_setting.c_str(), moving_average_setting.size());

    // Request for initial single data
    write(fdc, "R", 1);

    rclcpp::Rate loop_rate(rate);
    while (rclcpp::ok()) {
        int c, len;
        char str[256];
        int tick;
        uint16_t data[6];

        // Request for initial data (2nd round)
        write(fdc, "R", 1);

        // Obtain single data
        len = 0;
        while ( len < kDataLength ) {
                c = read(fdc, str+len, kDataLength-len);
                if (c > 0) {
                    len += c;
                } else {
                    RCLCPP_DEBUG(node->get_logger(), "=== need to read more data ... n = %d (%d) ===", c, len);
                    continue;
                }
            }
        if ( len != kDataLength ) {
            RCLCPP_WARN(node->get_logger(), "=== error reciving data ... n = %d ===", c);
        }


        sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
               &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

        geometry_msgs::msg::WrenchStamped msg;
        msg.header.frame_id = frame_id;
        msg.header.stamp = node->get_clock()->now();

        Eigen::Vector3d force_vector((data[0] - 8192) / 24.0,
                                     (data[1] - 8192) / 24.0,
                                     (data[2] - 8192) / 24.0);
        Eigen::Vector3d torque_vector((data[3] - 8192) / 1638.0,
                                      (data[4] - 8192) / 1638.0,
                                      (data[5] - 8192) / 1638.0);
        torque_vector = linear_frame_to_sensor * force_vector
                      + rot_frame_to_sensor * torque_vector;
        force_vector = rot_frame_to_sensor * force_vector;

        msg.wrench.force.x = force_vector(0);
        msg.wrench.force.y = force_vector(1);
        msg.wrench.force.z = force_vector(2);
        msg.wrench.torque.x = torque_vector(0);
        msg.wrench.torque.y = torque_vector(1);
        msg.wrench.torque.z = torque_vector(2);

        pub->publish(msg);
        rclcpp::spin_some(node);

        loop_rate.sleep();
    }
    rclcpp::shutdown();

    return 0;
}
