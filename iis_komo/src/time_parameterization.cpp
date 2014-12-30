#include <time_parameterization.h>


namespace iis_komo {


static const double DEFAULT_VEL_MAX = 1.0;
static const double DEFAULT_ACCEL_MAX = 1.0;
static const double ROUNDING_THRESHOLD = 0.01;

TimeParameterization::TimeParameterization(unsigned int max_iterations, double max_time_change_per_it)
	: max_iterations_(max_iterations),
	  max_time_change_per_it_(max_time_change_per_it),
	  velocity_factor_(0.5)
{}

TimeParameterization::~TimeParameterization()
{}

// Applies velocity
void TimeParameterization::applyVelocityConstraints(trajectory_msgs::JointTrajectory &trajectory, std::vector<double> &time_diff) const
{
	int num_points = trajectory.points.size();
	const arr limits = IISRobot::get_arm_velocity_limits();

	for (int i = 0 ; i < num_points-1 ; ++i)
	{
		trajectory_msgs::JointTrajectoryPoint &curr_waypoint = trajectory.points[i];
		trajectory_msgs::JointTrajectoryPoint &next_waypoint = trajectory.points[i+1];

		for (std::size_t j = 0 ; j < 7; ++j)
		{
			double v_max = limits(j) * velocity_factor_;

			double dq1 = curr_waypoint.positions[j];
			double dq2 = next_waypoint.positions[j];
			double t_min = std::abs(dq2-dq1) / v_max;

			if (t_min > time_diff[i]) {
				time_diff[i] = t_min;
			}
		}
	}
}

// Iteratively expand dt1 interval by a constant factor until within acceleration constraint
// In the future we may want to solve to quadratic equation to get the exact timing interval.
// To do this, use the CubicTrajectory::quadSolve() function in cubic_trajectory.h
double TimeParameterization::findT1(const double dq1, const double dq2,
									double dt1,	const double dt2,
									const double a_max) const {

	const double mult_factor = 1.01;
	double v1 = (dq1)/dt1;
	double v2 = (dq2)/dt2;
	double a = 2.0*(v2-v1)/(dt1+dt2);
	while( std::abs( a ) > a_max )
	{
		v1 = (dq1)/dt1;
		v2 = (dq2)/dt2;
		a = 2.0*(v2-v1)/(dt1+dt2);
		dt1 *= mult_factor;
	}
	return dt1;
}

double TimeParameterization::findT2(const double dq1, const double dq2,
									const double dt1, double dt2,
									const double a_max) const {

	const double mult_factor = 1.01;
	double v1 = (dq1)/dt1;
	double v2 = (dq2)/dt2;
	double a = 2.0*(v2-v1)/(dt1+dt2);
	while( std::abs( a ) > a_max )
	{
		v1 = (dq1)/dt1;
		v2 = (dq2)/dt2;
		a = 2.0*(v2-v1)/(dt1+dt2);
		dt2 *= mult_factor;
	}
	return dt2;
}

void TimeParameterization::updateTrajectory(trajectory_msgs::JointTrajectory &trajectory, const std::vector<double> &time_diff) const
{
	// Error check
	if (time_diff.empty())
		return;

	double time_sum = 0.0;
	trajectory_msgs::JointTrajectoryPoint *prev_waypoint;
	trajectory_msgs::JointTrajectoryPoint *curr_waypoint;
	trajectory_msgs::JointTrajectoryPoint *next_waypoint;

	int num_points = trajectory.points.size();
	trajectory.points[0].time_from_start = ros::Duration(time_sum);

	// Times
	for (int i = 1; i < num_points; ++i) {
		// Update the time between the waypoints in the robot_trajectory.
		time_sum += time_diff[i-1];
		trajectory.points[i].time_from_start = ros::Duration(time_sum);
	}

	// Return if there is only one point in the trajectory!
	if (num_points <= 1)
		return;

	// Accelerations
	for (int i = 0; i < num_points; ++i)
	{
		curr_waypoint = &trajectory.points[i];
		if (i > 0)
			prev_waypoint = &trajectory.points[i-1];
		if (i < num_points-1)
			next_waypoint = &trajectory.points[i+1];
		for (std::size_t j = 0; j < 7; ++j)
		{
			double q1;
			double q2;
			double q3;
			double dt1;
			double dt2;
			if (i == 0)
			{
				// First point
				q1 = next_waypoint->positions[j];
				q2 = curr_waypoint->positions[j];
				q3 = q1;
				dt1 = dt2 = time_diff[i];
			}
			else
				if (i < num_points-1)
				{
					// middle points
					q1 = prev_waypoint->positions[j];
					q2 = curr_waypoint->positions[j];
					q3 = next_waypoint->positions[j];
					dt1 = time_diff[i-1];
					dt2 = time_diff[i];
				}
				else
				{
					// last point
					q1 = prev_waypoint->positions[j];
					q2 = curr_waypoint->positions[j];
					q3 = q1;
					dt1 = dt2 = time_diff[i-1];
				}
			double v1, v2, a;
			bool start_velocity = false;
			if (dt1 == 0.0 || dt2 == 0.0)
			{
				v1 = 0.0;
				v2 = 0.0;
				a = 0.0;
			}
			else
			{
				if (i == 0)
				{
					if (curr_waypoint->velocities.size() == 7)
					{
						start_velocity = true;
						v1 = curr_waypoint->velocities[j];
					}
				}
				v1 = start_velocity ? v1 : (q2-q1)/dt1;
				//v2 = (q3-q2)/dt2;
				v2 = start_velocity ? v1 : (q3-q2)/dt2; // Needed to ensure continuous velocity for first point
				a = 2.0*(v2-v1)/(dt1+dt2);
			}
			// resize the vectors on initial update
			if(curr_waypoint->velocities.size() != 7) {
				curr_waypoint->velocities.resize(7);
				curr_waypoint->accelerations.resize(7);
			}

			curr_waypoint->velocities[j] = (v2+v1)/2.0;
			curr_waypoint->accelerations[j] = a;
		}
	}
}

// remove points where time_diff to previous point is smaller than epsilon
void TimeParameterization::removeUselessPoints(trajectory_msgs::JointTrajectory &trajectory, const std::vector<double> &time_diff, double epsilon) const
{
	for (int i = 0; i < time_diff.size(); ++i) {
		if(time_diff[i] < epsilon) {
			trajectory.points.erase(trajectory.points.begin() + i);
		}
	}
}

// Applies Acceleration constraints
void TimeParameterization::applyAccelerationConstraints(trajectory_msgs::JointTrajectory &trajectory,
														std::vector<double> & time_diff) const
{
	trajectory_msgs::JointTrajectoryPoint *prev_waypoint;
	trajectory_msgs::JointTrajectoryPoint *curr_waypoint;
	trajectory_msgs::JointTrajectoryPoint *next_waypoint;

	int num_points = trajectory.points.size();
	unsigned int num_joints = 7;
	int num_updates = 0;
	int iteration = 0;
	bool backwards = false;
	double q1, q2, q3;
	double dt1, dt2;
	double v1, v2;
	double a;

	do
	{
		num_updates = 0;
		iteration++;
		// In this case we iterate through the joints on the outer loop.
		// This is so that any time interval increases have a chance to get propogated through the trajectory
		for (unsigned int j = 0; j < num_joints ; ++j)
		{
			// Loop forwards, then backwards
			for (int count = 0; count < 2; ++count)
			{
				for (int i = 0 ; i < num_points-1; ++i)
				{
					int index = backwards ? (num_points-1)-i : i;
					curr_waypoint = &trajectory.points[index];
					if (index > 0)
						prev_waypoint = &trajectory.points[index-1];
					if (index < num_points-1)
						next_waypoint = &trajectory.points[index+1];

					// Get acceleration limits
					double a_max = 1.0;
//					const robot_model::VariableBounds &b = rmodel.getVariableBounds(vars[j]);
//					if (b.acceleration_bounded_)
//						a_max = std::min(fabs(b.max_acceleration_), fabs(b.min_acceleration_));

					if (index == 0) {
						// First point
						q1 = next_waypoint->positions[j];
						q2 = curr_waypoint->positions[j];
						q3 = next_waypoint->positions[j];
						dt1 = dt2 = time_diff[index];
						assert(!backwards);

					} else if (index < num_points-1) {
							// middle points
							q1 = prev_waypoint->positions[j];
							q2 = curr_waypoint->positions[j];
							q3 = next_waypoint->positions[j];
							dt1 = time_diff[index-1];
							dt2 = time_diff[index];
						}
						else
						{
							// last point - careful, there are only numpoints-1 time intervals
							q1 = prev_waypoint->positions[j];
							q2 = curr_waypoint->positions[j];
							q3 = prev_waypoint->positions[j];
							dt1 = dt2 = time_diff[index-1];
							assert(backwards);
						}
					if (dt1 == 0.0 || dt2 == 0.0)
					{
						v1 = 0.0;
						v2 = 0.0;
						a = 0.0;
					}
					else
					{
						bool start_velocity = false;
						if (index == 0)
						{
							if (curr_waypoint->velocities.size() == 7)
							{
								start_velocity = true;
								v1 = curr_waypoint->velocities[j];
							}
						}
						v1 = start_velocity ? v1 : (q2-q1)/dt1;
						v2 = (q3-q2)/dt2;
						a = 2.0*(v2-v1)/(dt1+dt2);
					}
					if (fabs(a) > a_max + ROUNDING_THRESHOLD)
					{
						if (!backwards)
						{
							dt2 = std::min(dt2 + max_time_change_per_it_, findT2(q2-q1, q3-q2, dt1, dt2, a_max));
							time_diff[index] = dt2;
						}
						else
						{
							dt1 = std::min(dt1 + max_time_change_per_it_, findT1(q2-q1, q3-q2, dt1, dt2, a_max));
							time_diff[index-1] = dt1;
						}
						num_updates++;
						if (dt1 == 0.0 || dt2 == 0.0)
						{
							v1 = 0.0;
							v2 = 0.0;
							a = 0.0;
						}
						else
						{
							v1 = (q2-q1)/dt1;
							v2 = (q3-q2)/dt2;
							a = 2*(v2-v1)/(dt1+dt2);
						}
					}
				}
				backwards = !backwards;
			}
		}
		//logDebug("applyAcceleration: num_updates=%i", num_updates);
	} while (num_updates > 0 && iteration < static_cast<int>(max_iterations_));
}

bool TimeParameterization::computeTimeStamps(trajectory_msgs::JointTrajectory& trajectory) const
{
	const int num_points = trajectory.points.size();

	if (num_points == 0) {
		return true;
	}

	std::vector<double> time_diff(num_points-1, 0.0); // the time difference between adjacent points
	applyVelocityConstraints(trajectory, time_diff);
	applyAccelerationConstraints(trajectory, time_diff);
	updateTrajectory(trajectory, time_diff);
	// remove points where time difference to previous point is smaller
	// than epsilon (i.e. not strictly increasing in time...)
	removeUselessPoints(trajectory, time_diff, 1e-4);

	return true;
}

void TimeParameterization::clearTimeParams(trajectory_msgs::JointTrajectory &trajectory) {
	// iterate over all waypoints and clear all time, velocity and accelleration related values.
	for (int i = 0; i < trajectory.points.size(); ++i) {
		trajectory_msgs::JointTrajectoryPoint &pt = trajectory.points[i];
		pt.velocities.clear();
		pt.accelerations.clear();
		pt.time_from_start = ros::Duration(0);
	}
}

} // namespace iis_komo
