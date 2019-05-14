/**
 * @file trajectory_ctrl_test.cpp
 * Trajectory controller unit test.
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "trajectory_ctrl_test.h"
#include "../controllers/TrajectoryCtrl.hpp"
#include "../controllers/GimbalCtrl.hpp"
#include "utils.h"
#include <unit_test/unit_test.h>
#include <matrix/math.hpp>
#include <uORB/topics/vehicle_local_position.h>

#define FLOAT_PRECISION 5

class TrajectoryCtrlTest : public UnitTest
{
public:
	TrajectoryCtrlTest();
	virtual ~TrajectoryCtrlTest();

	virtual bool run_tests();

private:
	bool goal_reached_test();
};

class MockWaypointNavigator : public WaypointNavigator
{
public:
	int waypoint_copy(int index, matrix::Vector3f *waypoint) override
	{
		*waypoint = WAYPOINTS[index % WAYPOINT_COUNT];
		return index;
	};

	int waypoint_count() override { return WAYPOINT_COUNT; };

	static const int WAYPOINT_COUNT = 3;
	const matrix::Vector3f WAYPOINTS[WAYPOINT_COUNT] = {
		matrix::Vector3f(10, 10, -10),
		matrix::Vector3f(20, 20, -20),
		matrix::Vector3f(30, 30, -30)
	};
};

TrajectoryCtrlTest::TrajectoryCtrlTest() {
}

TrajectoryCtrlTest::~TrajectoryCtrlTest() {
}

bool TrajectoryCtrlTest::goal_reached_test() {
	MockWaypointNavigator navigator;
	GimbalCtrl gimbal_ctrl;
	TrajectoryCtrl trajectory_ctrl(gimbal_ctrl, navigator);
	matrix::Vector3f goal_position;
	int update_attempt_count;
	bool warning_already_displayed = false; // avoid displaying duplicate warnings
	// Warning message to be displayed
	const char *warning_message = "\nWARNING: TrajectoryCtrl::update() is suboptimal.\n"
		"WARNING: This test force the position of the drone to be at the center of\n"
		"WARNING: the waypoint #0 by publishing a vehicle_local_position message on\n"
		"WARNING: uORB then call TrajectoryCtrl::update() and check that the goal\n"
		"WARNING: position is correctly set.\n"
		"WARNING: The test has detected that multiple call to TrajectoryCtrl::update()\n"
		"WARNING: are required in order to set the goal position.\n"
		"WARNING: To fix this warning, try to optimise TrajectoryCtrl::update()."
	;

	/* First waypoint */
	set_local_position_vector3f(matrix::Vector3f(0, 0, 0));
	for(update_attempt_count = 0; update_attempt_count < 101; update_attempt_count++) {
		trajectory_ctrl.update();
		if((trajectory_ctrl.get_goal_position() - navigator.WAYPOINTS[0]).length() < 0.001f) {
			break;
		}
	}

	goal_position = trajectory_ctrl.get_goal_position();
	compare_vector3f("goal_position", goal_position, navigator.WAYPOINTS[0], FLOAT_PRECISION);
	ut_assert_false(trajectory_ctrl.is_goal_reached());

	if(update_attempt_count > 0 && !warning_already_displayed) {
		PX4_WARN(warning_message);
		warning_already_displayed = true;
	}

	/* Second waypoint */
	set_local_position_vector3f(navigator.WAYPOINTS[0]);
	for(update_attempt_count = 0; update_attempt_count < 101; update_attempt_count++) {
		trajectory_ctrl.update();
		if((trajectory_ctrl.get_goal_position() - navigator.WAYPOINTS[1]).length() < FLOAT_PRECISION) {
			break;
		}
	}

	goal_position = trajectory_ctrl.get_goal_position();
	compare_vector3f("goal_position", goal_position, navigator.WAYPOINTS[1], FLOAT_PRECISION);
	ut_assert_false(trajectory_ctrl.is_goal_reached());

	if(update_attempt_count > 0 && !warning_already_displayed) {
		PX4_WARN(warning_message);
		warning_already_displayed = true;
	}

	/* Third waypoint */
	set_local_position_vector3f(navigator.WAYPOINTS[1]);
	for(update_attempt_count = 0; update_attempt_count < 101; update_attempt_count++) {
		trajectory_ctrl.update();
		if((trajectory_ctrl.get_goal_position() - navigator.WAYPOINTS[2]).length() < FLOAT_PRECISION) {
			break;
		}
	}

	goal_position = trajectory_ctrl.get_goal_position();
	compare_vector3f("goal_position", goal_position, navigator.WAYPOINTS[2], FLOAT_PRECISION);
	ut_assert_false(trajectory_ctrl.is_goal_reached());

	if(update_attempt_count > 0 && !warning_already_displayed) {
		PX4_WARN(warning_message);
		warning_already_displayed = true;
	}

	/* End of trajectory */
	set_local_position_vector3f(navigator.WAYPOINTS[2]);
	for(update_attempt_count = 0; update_attempt_count < 101; update_attempt_count++) {
		trajectory_ctrl.update();
		if(trajectory_ctrl.is_goal_reached()) {
			break;
		}
	}

	ut_assert_true(trajectory_ctrl.is_goal_reached());
	
	if(update_attempt_count > 0 && !warning_already_displayed) {
		PX4_WARN(warning_message);
		warning_already_displayed = true;
	}
	return true;
}

bool TrajectoryCtrlTest::run_tests()
{
	ut_run_test(goal_reached_test);
	return (_tests_failed == 0);
}

ut_declare_test(trajectory_ctrl_test, TrajectoryCtrlTest)
