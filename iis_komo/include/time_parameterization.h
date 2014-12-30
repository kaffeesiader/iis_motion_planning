#ifndef TIME_PARAMETERIZATION_H
#define TIME_PARAMETERIZATION_H

#include <trajectory_msgs/JointTrajectory.h>
#include <iis_robot.h>

namespace iis_komo {
/// \brief This class modifies the timestamps of a trajectory to respect
/// velocity and acceleration constraints (taken from moveit core repository at
/// https://github.com/ros-planning/moveit_core/blob/indigo-devel/trajectory_processing/include/moveit/trajectory_processing/iterative_time_parameterization.h).

class TimeParameterization {

public:

	TimeParameterization(unsigned int max_iterations = 100, double max_time_change_per_it = .01);
	~TimeParameterization();

	// compute the time parameterization for given trajectory
	bool computeTimeStamps(trajectory_msgs::JointTrajectory &trajectory) const;
	// clear previously computed time parameterization of given trajectory
	void clearTimeParams(trajectory_msgs::JointTrajectory &trajectory);

	void set_velocity_factor(double factor) {
		CHECK(factor > 0 && factor <= 1.0, "Invalid velocity factor!");
		velocity_factor_ = factor;
	}
	double get_velocity_factor() {return velocity_factor_;}

private:

	unsigned int max_iterations_; /// @brief maximum number of iterations to find solution
	double max_time_change_per_it_; /// @brief maximum allowed time change per iteration in seconds
	double velocity_factor_; /// @brief percentage of maximum velocity

	void applyVelocityConstraints(trajectory_msgs::JointTrajectory &trajectory, std::vector<double> &time_diff) const;
	void applyAccelerationConstraints(trajectory_msgs::JointTrajectory &trajectory, std::vector<double> &time_diff) const;
	double findT1( const double d1, const double d2, double t1, const double t2, const double a_max) const;
	double findT2( const double d1, const double d2, const double t1, double t2, const double a_max) const;
	void updateTrajectory(trajectory_msgs::JointTrajectory& trajectory, const std::vector<double>& time_diff) const;
	void removeUselessPoints(trajectory_msgs::JointTrajectory& trajectory, const std::vector<double>& time_diff, double epsilon = 1e-4) const;
};

}

#endif // TIME_PARAMETERIZATION_H

