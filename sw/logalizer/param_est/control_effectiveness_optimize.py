#!/usr/bin/env python3
#
# Copyright (C) 2018 Ewoud Smeur
# Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
import scipy as sp
from scipy import signal, optimize
import csv
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure, show
from optparse import OptionParser

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

def first_order_model(signal, tau):
    '''
    Apply a first order filter with (discrete) time constant tau
    '''
    return sp.signal.lfilter([tau], [1, tau-1], signal, axis=0)


def diff_signal(signal, freq, order=1, filt=None):
    '''
    compute the nth-order derivative of a signal of fixed freq
    and by applying a filter if necessary
    '''
    if filt is not None:
        signal = sp.signal.lfilter(filt[0], filt[1], signal, axis=0)

    res = [signal]
    nb = np.shape(signal)[1]
    for i in range(order):
        sigd = np.vstack((np.zeros((1,nb)), np.diff(res[-1], 1, axis=0))) * freq
        res.append(sigd)
    return res

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


def optimize_axis(x, y, freq, order=1, filt=None, p0=None, actuator_model=first_order_model):
    '''
    global optimization function
    '''

    def err_func(p, c, m):
        '''
        p vector of parameters to optimize
        c command vector
        m measurement vector

        computes the error between the scaled command derivate and the measurements
        '''
        mdl = cmd_model(c, p, freq, actuator_model, filt, order)
        #err = np.hstack((mdl,mdl,mdl)) - m
        err = mdl - m
        #print("e",np.shape(mdl),np.shape(m),np.shape(err))
        #print(p,np.shape(m),np.shape(mdl))
        #plt.figure()
        #plt.plot(m)
        #plt.plot(mdl)
        #plt.show()
        return err[:,0]

    if order <= 0:
        print("order should be > 0")
        exit(1)

    if p0 is None:
        p0 = np.hstack((np.array([.01, .01]), np.zeros(order))) # start from random non-zero value

    #ddy = diff_signal(y, freq, order+1, filt)
    #print(np.shape(ddy))
    #plt.figure()
    #plt.plot(y)
    #plt.plot(ddy[0])
    #plt.plot(ddy[1])
    #plt.plot(ddy[2])
    p1, cov, info, msg, success = optimize.leastsq(err_func, p0, args=(x, y), full_output=1)
    print(p1, success)
    return p1


def plot_results(x, y, t, start, end, label):
    '''
    plot two curves for comparison
    '''
    #print(np.shape(x), np.shape(y), np.shape(t))
    plt.figure()
    plt.plot(t, y)
    plt.plot(t, x)
    plt.xlabel('t [s]')
    plt.ylabel(label)
    plt.figure()
    plt.plot(x[start:end], y[start:end])
    plt.xlabel('command [pprz]')
    plt.ylabel(label)


def process_data(f_name, start, end, freq, fo_c=None):

    # Read data from log file
    data = genfromtxt(f_name, delimiter=',', skip_header=1)
    N = data.shape[0]

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

    # Show all plots
    plt.show()


def main():
    usage = "usage: %prog [options] log_filename.csv" + "\n" + "Run %prog --help to list the options."
    parser = OptionParser(usage)
    parser.add_option("-f", "--freq", dest="freq",
                      action="store", default=512,
                      help="Sampling frequency")
    parser.add_option("-d", "--dyn", dest="dyn",
                      action="store", default=0.08,
                      help="First order actuator dynamic (discrete time), 'None' for auto tuning")
    parser.add_option("-s", "--start",
                      help="Start time",
                      action="store", dest="start", default="0")
    parser.add_option("-e", "--end",
                      help="End time (-1 for unlimited time)",
                      action="store", dest="end", default=-1)
    parser.add_option("-p", "--plot",
                      help="Show resulting plots",
                      action="store_true", dest="plot")
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose")
    (options, args) = parser.parse_args()
    if len(args) != 1:
        parser.error("incorrect number of arguments")
    else:
        if os.path.isfile(args[0]):
            filename = args[0]
        else:
            print(args[0] + " not found")
            sys.exit(1)

    freq = int(options.freq)
    start = int(options.start) * freq
    end = int(options.end) * freq

    process_data(filename, start, end, freq, float(options.dyn))


if __name__ == "__main__":
    main()

