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

def cmd_model(c, p, freq, actuator_model, filt=None):
    '''
    apply actuator model, derivate and scale
    '''
    ca = actuator_model(c, p[0])
    dca = diff_signal(ca, freq, 1, filt)
    return (p[1] * dca[1] + p[2]*np.ones(np.shape(dca[1])))

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
        err = cmd_model(c, p, freq, actuator_model, filt) - m[0]
        print(p)
        return err[:,0]

    if order <= 0:
        print("order should be > 0")
        exit(1)

    if p0 is None:
        p0 = np.hstack((np.array([1., 1.]), np.zeros(order))) # start from random non-zero value

    ddy = diff_signal(y, freq, order, filt)
    p1, cov, info, msg, success = optimize.leastsq(err_func, p0, args=(x, ddy[-1]), full_output=1)
    print(p1, success)
    return p1


def plot_results(x, y, t, label):
    '''
    plot two curves for comparison
    '''
    print(np.shape(x), np.shape(y), np.shape(t))
    plt.figure()
    plt.plot(t, x)
    plt.plot(t, y)
    plt.xlabel('t [s]')
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

    # Optimization for each channels
    p_gyro = optimize_axis(cmd[start:end,[CMD_ROLL]], gyro[start:end,[GYRO_P]], freq, 1, filt)
    roll_cmd = cmd_model(cmd[:,[CMD_ROLL]], p_gyro, freq, first_order_model, filt)

    # Plot
    plot_results(roll_cmd, gyro_df[-1][:,[CMD_ROLL]], t, 'p dot dot [rad/s^3]')

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

## Do a linear least squares fit of the data
#def fit_axis(x,y,t, axis, start, end):
#    c = np.linalg.lstsq(x[start:end], y[start:end])[0]
#    print('Fit of axis ' + axis + ': ' + str(c*1000))
#    plt.figure()
#    plt.plot(t,y)
#    plt.plot(t,np.dot(x,c))
#    plt.ylabel(axis + ' dotdot[rad/s^3]')
#    plt.xlabel('t [s]')
#    return c

    #p1, cov, info, msg, success = optimize.leastsq(err_func, np.array([0.035, 0.018]), args=(cmd, meas), full_output=1, factor=0.1) #, bounds=([-np.inf, np.inf, 0.01],[np.inf,np.inf,1.]))

    #dcmd_af, ddmeas_f = deriv_signals(p1, x, y)
    #plt.figure()
    #c = p1[0]*dcmd_af + p1[1]*np.ones(np.shape(dcmd_af))
    ##print(np.shape(c), np.shape(t))
    #plt.plot(t,ddmeas_f)
    #plt.plot(t,c)
    #plt.ylabel(axis + ' dotdot[rad/s^3] (optim)')
    #plt.xlabel('t [s] (optim)')

    #def deriv_signals(p, cmd, meas):
    #    cmd_a = sp.signal.lfilter([p[2]], [1, -(1-p[2])], cmd, axis=0)
    #    cmd_af = sp.signal.lfilter(filt[0], filt[1], cmd_a, axis=0)
    #    dcmd_af = np.vstack((np.zeros((1,1)), np.diff(cmd_af,1,axis=0)))*freq
    #    meas_f = sp.signal.lfilter(filt[0], filt[1], meas, axis=0)
    #    dmeas_f = np.vstack((np.zeros((1,1)), np.diff(meas_f,1,axis=0)))*freq
    #    ddmeas_f = np.vstack((np.zeros((1,1)), np.diff(dmeas_f,1,axis=0)))*freq
    #    return (dcmd_af, ddmeas_f)

    ### Actuator dynamics
    ##cmd_a = sp.signal.lfilter([fo_c], [1, -(1-fo_c)], cmd, axis=0)
    ###servotest_a = sp.signal.lfilter([fo_c], [1, -(1-fo_c)], servotest, axis=0)

    ### b, a = signal.butter(2, 7/(sf/2), 'low', analog=False)
    ### servotest_a = sp.signal.lfilter(b, a, servotest, axis=0)
    ### servotest_a = actuator_dyn_2nd(servotest, 0.8, 70, 1/sf, 60000, 10000000)

    ##gyro_f = sp.signal.lfilter(b, a, gyro, axis=0)
    ##cmd_af = sp.signal.lfilter(b, a, cmd_a, axis=0)
    ##accel_f = sp.signal.lfilter(b, a, accel, axis=0)

    ### derivatives
    ##dgyro_f = np.vstack((np.zeros((1,3)), np.diff(gyro_f,1,axis=0)))*freq
    ##ddgyro_f = np.vstack((np.zeros((1,3)), np.diff(dgyro_f,1,axis=0)))*freq
    ##daccel_f = np.vstack((np.zeros((1,3)), np.diff(accel_f,1,axis=0)))*freq
    ##dcmd_af = np.vstack((np.zeros((1,4)), np.diff(cmd_af,1,axis=0)))*freq
    ##ddcmd_af = np.vstack((np.zeros((1,4)), np.diff(dcmd_af,1,axis=0)))*freq

    ### Estimation of the control effectiveness
    ###c = fit_axis(dcmd_af[:,0:4], ddgyro_f[:,[0]], t, 'p', start, end) # 0, N)
    ###c = fit_axis(dcmd_af[:,0:4], ddgyro_f[:,[1]], t, 'q', start, end) # 0, N)
    ###c = fit_axis(dcmd_af[:,0:4], daccel_f[:,[2]], t, 'accel', start, end)
    ### Use all commands, to see if there is crosstalk
    ###c = fit_axis(np.hstack((dcmd_af[:,0:4], ddcmd_af[:,0:4]/freq)), ddgyro_f[:,[2]], t, 'r', start, end) # 0, end)

    #plt.figure()
    #plt.plot(t,cmd[:,1:2])
    #plt.plot(t,cmd_af[:,1:2])
    #plt.plot(t,dcmd_af[:,1:2])
    #plt.plot(t,dcmd_af[:,1:2]*0.035)
    #plt.plot(t,ddgyro_f[:,0:1])
    #plt.xlabel('t [s]')
    #plt.ylabel('command [PPRZ]')

