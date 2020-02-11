#!/usr/bin/env python3
#
# Copyright (C) 2018 Ewoud Smeur
# Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.


import os
import sys
import scipy as sp
from scipy import signal, optimize
import csv
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure, show

import control_effectiveness_utils as ut

GYRO_P = 0
GYRO_Q = 1
GYRO_R = 2
ACCEL_X = 0
ACCEL_Y = 1
ACCEL_Z = 2
CMD_THRUST = 0
CMD_ROLL = 1
CMD_PITCH = 2
CMD_YAW = 3


def cmd_model(c, p, freq, actuator_model, filt=None, order=1):
    '''
    apply actuator model, derivate and scale
    '''
    ca = actuator_model(c, p[0])
    if order == 1:
        dca = diff_signal(ca, freq, 1, filt)
        return (p[1] * dca[1] + p[2]*np.ones(np.shape(dca[1])))
    elif order == 2:
        dca = diff_signal(ca, freq, 2, filt)
        return ((p[3] * dca[2]) + p[1] * dca[1] + p[2]*np.ones(np.shape(dca[1])))

def cmd_model_full(c, tau, eff, freq, actuator_model, filt=None):
    nb_cmd = c.shape[1]
    ca = actuator_model(c, tau)
    dca = diff_signal(ca, freq, 1, filt)
    res = np.zeros(np.shape(c))
    for i in range(c.shape[0]):
        #print(np.reshape(dca[-1][i,:],(nb_cmd,1)))
        res[i,:] = np.dot(eff, np.reshape(dca[-1][i,:],(nb_cmd,1)))[0]
    return res


def process_data(conf, f_name, start, end, freq, opt="axis", fo_c=None, verbose=False):

    # Read data from log file
    data = genfromtxt(f_name, delimiter=',', skip_header=1)
    N = data.shape[0]

    # Get number of inputs and outputs
    nb_in = 0
    nb_out = 0
    for key in conf['data']:
        el = conf['data'][key]
        if el['type'] == 'input' and el['index'] >= 0:
            nb_in += 1
        if el['type'] == 'command' and el['index'] >= 0:
            nb_out += 1
    mixing = np.array(conf['mixing'])
    if (nb_in, nb_out) != np.shape(mixing):
        print("Mixing matrix dimensions not matching number of inputs and commands")
        sys.exit(1)
    if verbose:
        print("Nb of inputs:", nb_in)
        print("Nb of commands:", nb_out)
        print("Mixing matrix:")
        print(mixing)
    sys.exit(0)

    if end == -1:
        end = N

    #First order actuator dynamics constant (discrete, depending on sf)
    #for now fixed value, use autotune when None later
    if fo_c is None:
        fo_c = 0.08


    # Data structure
    t = np.arange(N) / freq
    gyro = data[:,1:4]
    accel = data[:,4:7]/pow(2,10)
    cmd = data[:,7:11]

    # Filtering
    filt = signal.butter(2, 3.2/(freq/2), 'low', analog=False)

    # Measurements derivates + filter
    gyro_df = diff_signal(gyro, freq, 2, filt)
    accel_df = diff_signal(accel, freq, 1, filt)
    #print("g", np.shape(gyro_df))

    if opt == "axis":
        # Optimization for each channels
        p_roll = optimize_axis(cmd[start:end,[CMD_ROLL]], gyro_df[-1][start:end,[GYRO_P]], freq, 1, filt)
        roll_cmd = cmd_model(cmd[:,[CMD_ROLL]], p_roll, freq, first_order_model, filt)

        p_pitch = optimize_axis(cmd[start:end,[CMD_PITCH]], gyro_df[-1][start:end,[GYRO_Q]], freq, 1, filt)
        pitch_cmd = cmd_model(cmd[:,[CMD_PITCH]], p_pitch, freq, first_order_model, filt)

        p_yaw = optimize_axis(cmd[start:end,[CMD_YAW]], gyro_df[-1][start:end,[GYRO_R]], freq, 2, filt)
        yaw_cmd = cmd_model(cmd[:,[CMD_YAW]], p_yaw, freq, first_order_model, filt, 2)

        p_thrust = optimize_axis(cmd[start:end,[CMD_THRUST]], accel_df[-1][start:end,[ACCEL_Z]], freq, 1, filt)
        thrust_cmd = cmd_model(cmd[:,[CMD_THRUST]], p_thrust, freq, first_order_model, filt)
        #thrust_cmd = cmd_model(cmd[:,[CMD_THRUST]], [0.01682614, -2.36359587/1000, 0], freq, first_order_model, filt)

        # Plot
        plot_results(roll_cmd, gyro_df[-1][:,[GYRO_P]], t, start, end, 'p dot dot [rad/s^3]')
        plot_results(pitch_cmd, gyro_df[-1][:,[GYRO_Q]], t, start, end, 'q dot dot [rad/s^3]')
        plot_results(yaw_cmd, gyro_df[-1][:,[GYRO_R]], t, start, end, 'r dot dot [rad/s^3]')
        plot_results(thrust_cmd, accel_df[-1][:,[ACCEL_Z]], t, start, end, 'az dot [m/s^3]')

    elif opt == "mixed":
        tau = optimize_mixed(cmd[start:end,:], np.hstack((gyro_df[-1][start:end,:], accel_df[-1][start:end,[ACCEL_Z]])), freq, filt)

    elif opt == "full":
        # Full optimization
        tau, eff = optimize_full(cmd[start:end,:], np.hstack((gyro_df[-1][start:end,:], accel_df[-1][start:end,[ACCEL_Z]])), freq, filt)
        res_cmd = cmd_model_full(cmd, tau, eff, freq, first_order_model, filt)
        #print(res_cmd[:,CMD_ROLL])

        # Plot
        plot_results(res_cmd[:,CMD_ROLL], gyro_df[-1][:,[GYRO_P]], t, start, end, 'p dot dot [rad/s^3]')
        plot_results(res_cmd[:,CMD_PITCH], gyro_df[-1][:,[GYRO_Q]], t, start, end, 'q dot dot [rad/s^3]')
        plot_results(res_cmd[:,CMD_YAW], gyro_df[-1][:,[GYRO_R]], t, start, end, 'r dot dot [rad/s^3]')
        plot_results(res_cmd[:,CMD_THRUST], accel_df[-1][:,[ACCEL_Z]], t, start, end, 'az dot [rad/s^3]')

    else:
        print("Unknown optimization type: ", opt)
        exit(1)

    # Show all plots
    plt.show()


def main():
    from argparse import ArgumentParser
    import json

    parser = ArgumentParser(description="Control effectiveness estimation tool")
    parser.add_argument("config", help="JSON configuration file")
    parser.add_argument("data", help="Log file for parameter estimation")
    parser.add_argument("-o", "--opt", dest="opt",
                      action="store", default="axis",
                      help="Optimization type (axis, mixed, full)")
    parser.add_argument("-f", "--freq", dest="freq",
                      action="store", default=512,
                      help="Sampling frequency")
    parser.add_argument("-d", "--dyn", dest="dyn",
                      action="store", default=0.08,
                      help="First order actuator dynamic (discrete time), 'None' for auto tuning")
    parser.add_argument("-s", "--start",
                      help="Start time",
                      action="store", dest="start", default="0")
    parser.add_argument("-e", "--end",
                      help="End time (-1 for unlimited time)",
                      action="store", dest="end", default=-1)
    parser.add_argument("-p", "--plot",
                      help="Show resulting plots",
                      action="store_true", dest="plot")
    parser.add_argument("-v", "--verbose",
                      action="store_true", dest="verbose")
    args = parser.parse_args()

    if not os.path.isfile(args.config) and not os.path.isfile(args.data):
        print("Config or data files are not valid")
        sys.exit(1)

    freq = int(args.freq)
    start = int(args.start) * freq
    end = int(args.end) * freq

    with open(args.config, 'r') as f:
        conf = json.load(f)
        #if args.verbose:
        #    print(json.dumps(conf))

        process_data(conf, args.data, start, end, freq, args.opt, float(args.dyn), args.verbose)


if __name__ == "__main__":
    main()

