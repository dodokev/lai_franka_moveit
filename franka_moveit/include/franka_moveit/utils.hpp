#include <visualization_msgs/msg/marker.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>

/**
 * File containing some useful functions
 */

/**
 * Draw in rviz2 the trajectory of a given link through a given publisher
 */
void publishTcpTrajectory(
    const robot_trajectory::RobotTrajectory &trajectory,
    const std::string &tcp_link,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world"; // base frame
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "tcp_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.005; // line thickness

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.orientation.w = 1.0;

    // Iterate over trajectory waypoints
    for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
    {
        const moveit::core::RobotState &state = trajectory.getWayPoint(i);

        // From state at a waypoint, get transform of tcp_link from panda_tool0
        const Eigen::Isometry3d &tcp_pose = state.getGlobalLinkTransform(tcp_link);

        // Create new point from the pose
        geometry_msgs::msg::Point p;
        p.x = tcp_pose.translation().x();
        p.y = tcp_pose.translation().y();
        p.z = tcp_pose.translation().z();

        marker.points.push_back(p);
    }

    pub->publish(marker);
}

/**
 * Compute the damping
 */
double cosRialzato(double sigma, double lambda, double threshold)
{
  double tmp, reg;

  sigma = abs(sigma);
  if (sigma < threshold)
  {

    tmp = (sigma / threshold) * M_PI;
    reg = lambda * (0.5 * cos(tmp) + 0.5);
    //		cout << "Damping! " << rand();
  }

  else
  {
    //	cout << "No Damping! " << rand();
    reg = 0.0;
  }
  //	cout << reg << "\t" << sigma << "\n";

  return reg;
}

/**
 * Compute the inverse of the Jacobian
 */
Eigen::MatrixXd eig_pinv(Eigen::MatrixXd J, double threshold, double lambda)
{

  int rowJ = J.rows();
  int colJ = J.cols();
  Eigen::MatrixXd pinvJ(colJ, rowJ);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::VectorXd eigValues(rowJ);
  std::complex<double> app;
  for (int i = 0; i < rowJ; i++)
  {
    app = svd.singularValues()(i);
    eigValues(i) = app.real();
  }

  Eigen::VectorXd p(rowJ);
  Eigen::MatrixXd Sinv(colJ, rowJ);
  Sinv.setZero();
  for (int i = 0; i < rowJ; i++)
  {
    // Add a damping if necessary 
    p(i) = cosRialzato(eigValues(i), lambda, threshold);

    Sinv(i, i) = eigValues(i) / (std::pow(eigValues(i), 2) + p(i));
  }

  if (J.rows() == 1)
  {
    // cout << "J: \n" << J << "\n\n";
    // cout << "V: \n" << svd.matrixV() << "\n\nSinv\n" << Sinv << "\n\nU trasposto: \n" << svd.matrixU().transpose() << "\n\n\n\n\n";
  }

  pinvJ = svd.matrixV() * Sinv * svd.matrixU().transpose();

  return pinvJ;
}