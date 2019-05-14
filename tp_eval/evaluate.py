#!/usr/bin/env python2

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import os
import px4tools
import re



########


def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)


def find_string(pat, text):

    match = re.search(pat, text)
    if match:
        return match.group()
    else:
        return('not found')


def get_data_from_log(file = None, dev = False):
    # Folder of this script
    source_folder = os.path.dirname(os.path.abspath(__file__))

    if file is None:
        if dev:
            # os.chdir('../build_posix_sitl_dronecourse_dev/tmp/rootfs/fs/microsd/log')
            log_folder = source_folder + '/../build_posix_sitl_dronecourse_dev/tmp/rootfs/fs/microsd/log'
        else:
            # os.chdir('../build_posix_sitl_dronecourse/tmp/rootfs/fs/microsd/log')
            log_folder = source_folder + '/../build_posix_sitl_dronecourse/tmp/rootfs/fs/microsd/log'

        # List all logged sessions
        session_prop = []
        session_folders = os.listdir(log_folder)
        for session in session_folders:
            # append only if it is called 'sess#'
            if 'sess' in session:
                path_to_session = log_folder + '/' + session
                session_prop.append((path_to_session,
                                     os.stat(path_to_session).st_mtime))

        # Find newest session folder
        session_prop.sort(key=lambda tup: tup[1])
        newest_session_folder = session_prop[-1][0]

        # Find log files in this folder
        logfiles = [x for x in os.listdir(newest_session_folder) if '.ulg' in x]

        # Read log
        d = px4tools.read_ulog(newest_session_folder + '/' + logfiles[0])

    else:
        d = px4tools.read_ulog(file)

    data = d.concat(dt=0.01)

    # Clean dataset
    data = data[np.isfinite(data['t_vehicle_local_position_groundtruth_0__f_x'])]
    data = data[np.isfinite(data['t_vehicle_local_position_groundtruth_0__f_y'])]
    data = data[np.isfinite(data['t_vehicle_local_position_groundtruth_0__f_z'])]
    data = data[np.isfinite(data['timestamp'])]

    print('Data was analyzed from file ' + newest_session_folder + '/' + logfiles[0] + '\n\n')

    return data


def evaluation_waypoint_navigation(data_log):

    WAYPOINT_ACTIVATION_RANGE = 6

    if not len(data_log):
        print 'no data found for waypoint navigation'
        return

    px = data_log["t_vehicle_local_position_groundtruth_0__f_x"]
    py = data_log["t_vehicle_local_position_groundtruth_0__f_y"]
    pz = data_log["t_vehicle_local_position_groundtruth_0__f_z"]
    ts = data_log["timestamp"]

    # #

    pxarr = -px.values
    pyarr = py.values
    pzarr = -pz.values
    tarr = ts.values/1e6  # timestamp is in us

    data = {}

    data['x_drone'] = pxarr
    data['y_drone'] = pyarr
    data['z_drone'] = pzarr
    data['time'] = tarr

    # Get waypoints

    if 't_dronecourse_waypoint_0__f_x1' not in data_log:
        print 'no waypoint position logged. did you run "dronecourse waypoint_navigation"?'
        return

    data_task1_wp = data_log[np.isfinite(data_log['t_dronecourse_waypoint_0__f_x1'])]

    wp1_x = data_task1_wp["t_dronecourse_waypoint_0__f_x1"]
    wp1_y = data_task1_wp["t_dronecourse_waypoint_0__f_y1"]
    wp1_z = data_task1_wp["t_dronecourse_waypoint_0__f_z1"]

    wp2_x = data_task1_wp["t_dronecourse_waypoint_0__f_x2"]
    wp2_y = data_task1_wp["t_dronecourse_waypoint_0__f_y2"]
    wp2_z = data_task1_wp["t_dronecourse_waypoint_0__f_z2"]

    wp3_x = data_task1_wp["t_dronecourse_waypoint_0__f_x3"]
    wp3_y = data_task1_wp["t_dronecourse_waypoint_0__f_y3"]
    wp3_z = data_task1_wp["t_dronecourse_waypoint_0__f_z3"]

    pwx_t = [wp1_x.values[-1], wp2_x.values[-1], wp3_x.values[-1]]
    pwy = [wp1_y.values[-1], wp2_y.values[-1], wp3_y.values[-1]]
    pwz_t = [wp1_z.values[-1], wp2_z.values[-1], wp3_z.values[-1]]
    pwx = [x*-1 for x in pwx_t]
    pwz = [x*-1 for x in pwz_t]

    data['pwx'] = pwx
    data['pwy'] = pwy
    data['pwz'] = pwz

    ##

    minval = []
    minidx = []

    fig = plt.figure("Waypoint Navigation")

    data['cross_time_waypoints'] = []
    data['min_dist_waypoints'] = []

    for i in xrange(3):
        xdist = pxarr-pwx[i]
        ydist = pyarr-pwy[i]
        zdist = pzarr-pwz[i]

        dist = np.sqrt(xdist**2+ydist**2+zdist**2)

        minval.append(np.amin(dist))
        minidx.append(np.argmin(dist))

        t_cross = tarr[dist < WAYPOINT_ACTIVATION_RANGE]

        if len(t_cross) > 0:
            str1 = 'you crossed waypoint n. {0} after {1:.2f} seconds'.format(i+1, t_cross[0])
            str2 = 'minimum distance from waypoint n. {0} is {1:.2f} meters after {2:.2f} seconds'.format(i+1, minval[i], tarr[minidx[i]])

            data['cross_time_waypoints'].append(t_cross[0])
            data['min_dist_waypoints'].append(minval[i])

        else:
            data['cross_time_waypoints'].append(600)
            str1 = 'you did not cross waypoint n. {0} before timeout'.format(i+1)
            str2 = ''

        print str1
        print str2
        print '\n'

        ax = fig.add_subplot(1, 3, i+1)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('distance from waypoint n.{} [m]'.format(i+1))
        ax.grid()
        ax.plot(tarr, dist)

    #

    x = np.linspace(-10, 10, 100)
    y = np.linspace(-10, 10, 100)
    z = np.linspace(-10, 10, 100)

    #

    fig = plt.figure("Waypoint Navigation - Trajectory")

    ax = fig.add_subplot(1, 1, 1, projection='3d')

    ax.plot(pxarr, pyarr, zs=pzarr)


    x_m = mean([np.amin(pxarr), np.amax(pxarr)])
    y_m = mean([np.amin(pyarr), np.amax(pyarr)])
    z_m = mean([np.amin(pzarr), np.amax(pzarr)])

    x_range = max(np.abs(np.amin(pxarr) - x_m), np.abs(np.amax(pxarr) - x_m))
    y_range = max(np.abs(np.amin(pyarr) - y_m), np.abs(np.amax(pyarr) - y_m))
    z_range = max(np.abs(np.amin(pzarr) - z_m), np.abs(np.amax(pzarr) - z_m))

    range_fin = max([x_range, y_range, z_range])

    x_range = [x_m-range_fin/2, x_m+range_fin/2]
    y_range = [y_m-range_fin/2, y_m+range_fin/2]
    z_range = [z_m-range_fin/2, z_m+range_fin/2]

    # # plot spheres

    for i in xrange(3):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = pwx[i] + np.cos(u)*np.sin(v)
        y = pwy[i] + np.sin(u)*np.sin(v)
        z = pwz[i] + np.cos(v)
        ax.plot_wireframe(x, y, z, color="r")

    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    ax.set_zlim(z_range)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    return data


def evaluation_landing(data_log):

    EDGE_SIZE_Y = 2
    EDGE_SIZE_X = 1.05

    # Clean dataset

    data_log = data_log[np.isfinite(data_log['t_vehicle_local_position_groundtruth_0__f_x'])]
    data_log = data_log[np.isfinite(data_log['t_vehicle_local_position_groundtruth_0__f_y'])]
    data_log = data_log[np.isfinite(data_log['t_vehicle_local_position_groundtruth_0__f_z'])]
    data_log = data_log[np.isfinite(data_log['timestamp'])]

    data_log = data_log[np.isfinite(data_log["t_dronecourse_truck_position_0__f_x"])]
    data_log = data_log[np.isfinite(data_log["t_dronecourse_truck_position_0__f_y"])]
    data_log = data_log[np.isfinite(data_log["t_dronecourse_truck_position_0__f_z"])]

    data_log = data_log[100:]   # remove time needed for takeoff (a bit dirty!)

    # #

    px = data_log["t_vehicle_local_position_groundtruth_0__f_x"]
    py = data_log["t_vehicle_local_position_groundtruth_0__f_y"]
    pz = data_log["t_vehicle_local_position_groundtruth_0__f_z"]

    landed = data_log['t_vehicle_land_detected_0__f_landed']

    ts = data_log["timestamp"]

    tr_x = data_log["t_dronecourse_truck_position_0__f_x"]
    tr_y = data_log["t_dronecourse_truck_position_0__f_y"]
    tr_z = data_log["t_dronecourse_truck_position_0__f_z"]

    tr_vx = data_log["t_dronecourse_truck_position_0__f_vx"]
    tr_vy = data_log["t_dronecourse_truck_position_0__f_vy"]
    tr_vz = data_log["t_dronecourse_truck_position_0__f_vz"]


    if 't_target_position_ned_filtered_0__f_x' not in list(data_log) or 't_target_position_ned_0__f_x' not in list(data_log):
        print 'kalman filter not publishing - did you run "dronecourse target_following?" \n'
        return

    tr_x_eval = data_log["t_target_position_ned_filtered_0__f_x"]
    tr_y_eval = data_log["t_target_position_ned_filtered_0__f_y"]
    tr_z_eval = data_log["t_target_position_ned_filtered_0__f_z"]

    tr_vx_eval = data_log["t_target_position_ned_filtered_0__f_vx"]
    tr_vy_eval = data_log["t_target_position_ned_filtered_0__f_vy"]
    tr_vz_eval = data_log["t_target_position_ned_filtered_0__f_vz"]

    tr_x_meas = data_log["t_target_position_ned_0__f_x"]
    tr_y_meas = data_log["t_target_position_ned_0__f_y"]
    tr_z_meas = data_log["t_target_position_ned_0__f_z"]

    # #

    pxarr = -px.values
    pyarr = py.values
    pzarr = -pz.values

    data = {}

    data['x_drone'] = pxarr
    data['y_drone'] = pyarr
    data['z_drone'] = pzarr

    # fix axes
    tr_xarr = -tr_x.values
    tr_yarr = tr_y.values
    tr_zarr = -tr_z.values

    tr_xarr_eval = -tr_x_eval.values
    tr_yarr_eval = tr_y_eval.values
    tr_zarr_eval = -tr_z_eval.values

    tr_xarr_meas = -tr_x_meas.values
    tr_yarr_meas = tr_y_meas.values
    tr_zarr_meas = -tr_z_meas.values


    data['x_tr'] = tr_xarr
    data['y_tr'] = tr_yarr
    data['z_tr'] = tr_zarr

    data['x_tr_eval'] = tr_xarr_eval
    data['y_tr_eval'] = tr_yarr_eval
    data['z_tr_eval'] = tr_zarr_eval

    data['x_tr_meas'] = tr_xarr_meas
    data['y_tr_meas'] = tr_yarr_meas
    data['z_tr_meas'] = tr_zarr_meas

    #

    tr_vxarr = -tr_vx.values
    tr_vyarr = tr_vy.values
    tr_vzarr = -tr_vz.values

    tr_vxarr_eval = -tr_vx_eval.values
    tr_vyarr_eval = tr_vy_eval.values
    tr_vzarr_eval = -tr_vz_eval.values


    data['xv_tr'] = tr_vxarr
    data['yv_tr'] = tr_vyarr
    data['zv_tr'] = tr_vzarr

    data['vx_tr_eval'] = tr_vxarr_eval
    data['vy_tr_eval'] = tr_vyarr_eval
    data['vz_tr_eval'] = tr_vzarr_eval

    landedarr = landed.values
    tarr = ts.values/1e6  # timestamp is in us

    data['time'] = tarr

    if not sum(landedarr == 1):
        print 'no landing detected'
        return

    # #

    fig = plt.figure("Truck landing")

    ax = fig.add_subplot(1, 2, 1, projection='3d')

    ax.plot(pxarr, pyarr, zs=pzarr)

    x_m3 = mean([np.amin(pxarr), np.amax(pxarr)])
    y_m3 = mean([np.amin(pyarr), np.amax(pyarr)])
    z_m3 = mean([np.amin(pzarr), np.amax(pzarr)])

    x_range3 = max(np.abs(np.amin(pxarr) - x_m3), np.abs(np.amax(pxarr) - x_m3))
    y_range3 = max(np.abs(np.amin(pyarr) - y_m3), np.abs(np.amax(pyarr) - y_m3))
    z_range3 = max(np.abs(np.amin(pzarr) - z_m3), np.abs(np.amax(pzarr) - z_m3))

    range_fin3 = max([x_range3, y_range3, z_range3])

    x_range3 = [x_m3 - range_fin3 / 2, x_m3 + range_fin3 / 2]
    y_range3 = [y_m3 - range_fin3 / 2, y_m3 + range_fin3 / 2]
    z_range3 = [z_m3 - range_fin3 / 2, z_m3 + range_fin3 / 2]

    # # plot truck position

    ax.plot(tr_xarr, tr_yarr, zs=tr_zarr)

    ax.set_xlim(x_range3)
    ax.set_ylim(y_range3)
    ax.set_zlim(z_range3)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # # plot 2D platform + landing spot

    ax = fig.add_subplot(1, 2, 2)

    # # plot 2D truck surface

    ax.plot([- EDGE_SIZE_X, + EDGE_SIZE_X], [- EDGE_SIZE_Y, - EDGE_SIZE_Y], 'r', label='truck edge')
    ax.plot([- EDGE_SIZE_X, + EDGE_SIZE_X], [+ EDGE_SIZE_Y, + EDGE_SIZE_Y], 'r')
    ax.plot([+ EDGE_SIZE_X, + EDGE_SIZE_X], [- EDGE_SIZE_Y, + EDGE_SIZE_Y], 'r')
    ax.plot([- EDGE_SIZE_X, - EDGE_SIZE_X], [- EDGE_SIZE_Y, + EDGE_SIZE_Y], 'r')

    # landing spot

    is_landed = landedarr == 1

    t_land = tarr[is_landed][0]

    land_x = pxarr[is_landed][0]
    land_y = pyarr[is_landed][0]

    tr_x = tr_xarr[is_landed][0]
    tr_y = tr_yarr[is_landed][0]

    tr_vx = tr_vxarr[is_landed][0]
    tr_vy = tr_vyarr[is_landed][0]

    tr_vnorm = np.sqrt(tr_vx*tr_vx + tr_vy*tr_vy)

    tr_tilt_angle = np.arccos(tr_vy/tr_vnorm)

    dist_x = land_x - tr_x
    dist_y = land_y - tr_y

    dist = abs(max([dist_x, dist_y]))

    dist_x_rot = dist_x * np.cos(tr_tilt_angle) - dist_y * np.sin(tr_tilt_angle)
    dist_y_rot = dist_x * np.sin(tr_tilt_angle) + dist_y * np.cos(tr_tilt_angle)

    ax.scatter(dist_x_rot, dist_y_rot, s=100, c='b', label='landing position')

    dist = max([abs(dist_x), abs(dist_y)])

    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=1, mode="expand", borderaxespad=0.)
    ax.grid()

    if 1:  # managed
        str1 = 'you landed on the truck after after {0:.2f} seconds'.format(t_land)
        str2 = 'x distance from truck center is {0:.2f} meters'.format(dist_x_rot)
        str3 = 'y distance from truck center is {0:.2f} meters'.format(dist_y_rot)
    else:
        str1 = 'you did not manage to land on the plaform'
        str2 = ''
        str3 = ''

    data['time_landing'] = t_land
    data['distance_x'] = dist_x_rot
    data['distance_x'] = dist_x_rot
    data['distance'] = dist

    print '\n'
    print '\n'
    print str1
    print str2
    print str3
    print '\n'

    return data
