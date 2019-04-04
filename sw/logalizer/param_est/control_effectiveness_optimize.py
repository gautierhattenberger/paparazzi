#!/usr/bin/env python3

# Copyright (C) 2018 Ewoud Smeur

import os
import scipy as sp
from scipy import signal, optimize
import csv
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure, show
from optparse import OptionParser

def optimize_axis(x,y,t,axis,start,end,filt,freq):

    def deriv_signals(p, cmd, meas):
        cmd_a = sp.signal.lfilter([p[2]], [1, -(1-p[2])], cmd, axis=0)
        cmd_af = sp.signal.lfilter(filt[0], filt[1], cmd_a, axis=0)
        dcmd_af = np.vstack((np.zeros((1,1)), np.diff(cmd_af,1,axis=0)))*freq
        meas_f = sp.signal.lfilter(filt[0], filt[1], meas, axis=0)
        dmeas_f = np.vstack((np.zeros((1,1)), np.diff(meas_f,1,axis=0)))*freq
        ddmeas_f = np.vstack((np.zeros((1,1)), np.diff(dmeas_f,1,axis=0)))*freq
        return (dcmd_af, ddmeas_f)

    def err_func(p, cmd, meas):
        #cmd_a = sp.signal.lfilter([0.018], [1, -(1-0.018)], cmd, axis=0)
        #cmd_a = sp.signal.lfilter([p[2]], [1, -(1-p[2])], cmd, axis=0)
        #cmd_af = sp.signal.lfilter(filt[0], filt[1], cmd_a, axis=0)
        #dcmd_af = np.vstack((np.zeros((1,1)), np.diff(cmd_af,1,axis=0)))*freq
        #meas_f = sp.signal.lfilter(filt[0], filt[1], meas, axis=0)
        #dmeas_f = np.vstack((np.zeros((1,1)), np.diff(meas_f,1,axis=0)))*freq
        #ddmeas_f = np.vstack((np.zeros((1,1)), np.diff(dmeas_f,1,axis=0)))*freq
        dcmd_af, ddmeas_f = deriv_signals(p, cmd, meas)
        #err = dcmd_af - p[0] * dmeas_f
        err = (p[0] * dcmd_af + p[1]*np.ones(np.shape(dcmd_af))) - ddmeas_f
        #err = (p[0] * ddmeas_f + p[1]*np.ones(np.shape(ddmeas_f))) - dcmd_af
        print(p)
        #plt.figure()
        #plt.plot(cmd)
        #plt.plot(cmd_a)
        #plt.plot(cmd_af)
        #plt.plot(p[0] * dcmd_af + p[1]*np.ones(np.shape(dcmd_af)))
        #plt.plot(ddmeas_f)
        #plt.show()
        return err[:,0]

    cmd = x[start:end]
    meas = y[start:end]
    p1, cov, info, msg, success = optimize.leastsq(err_func, np.array([0.1, 0., 0.1]), args=(cmd, meas), full_output=1, factor=0.1) #, bounds=([-np.inf, np.inf, 0.01],[np.inf,np.inf,1.]))
    #p1, cov, info, msg, success = optimize.leastsq(err_func, np.array([0.035, 0.018]), args=(cmd, meas), full_output=1, factor=0.1) #, bounds=([-np.inf, np.inf, 0.01],[np.inf,np.inf,1.]))
    print(p1, success)
    dcmd_af, ddmeas_f = deriv_signals(p1, x, y)
    plt.figure()
    c = p1[0]*dcmd_af + p1[1]*np.ones(np.shape(dcmd_af))
    #print(np.shape(c), np.shape(t))
    plt.plot(t,ddmeas_f)
    plt.plot(t,c)
    plt.ylabel(axis + ' dotdot[rad/s^3] (optim)')
    plt.xlabel('t [s] (optim)')


# Do a linear least squares fit of the data
def fit_axis(x,y,t, axis, start, end):
    c = np.linalg.lstsq(x[start:end], y[start:end])[0]
    print('Fit of axis ' + axis + ': ' + str(c*1000))
    plt.figure()
    plt.plot(t,y)
    plt.plot(t,np.dot(x,c))
    plt.ylabel(axis + ' dotdot[rad/s^3]')
    plt.xlabel('t [s]')
    return c

# Effort to make a function that accurately models the actuator, but this is still not sufficient for the bebop2
def actuator_dyn_2nd(x, zeta, omega, dt, maxrate, maxaccel):
    y = np.zeros(x.shape)
    y[0] = x[0]
    yd = 0
    ydd = 0
    for i in range(x.shape[0]-1):
        ydd = -2*zeta*omega*yd + (x[i]-y[i])*omega*omega
        if(abs(ydd) > maxaccel):
            ydd = maxaccel*np.sign(ydd)
        yd = yd + ydd*dt
        if(abs(yd) > maxrate):
            yd = maxrate*np.sign(yd)
        y[i+1] = y[i] + yd*dt
    return y


def process_data(f_name, start, end, freq, fo_c=None):

    # Read data from log file
    data = genfromtxt(f_name, delimiter=',', skip_header=1)
    N = data.shape[0]

    if end == -1:
        end = N

    ## Sample frequency
    #sf = 512;
    #First order actuator dynamics constant (discrete, depending on sf)
    #for now fixed value, use autotune when None later
    if fo_c is None:
        fo_c = 0.08


    # Data structure
    t = np.arange(N) / freq
    gyro = data[:,1:4]
    accel = data[:,4:7]/pow(2,10)
    cmd = data[:,7:11]
    # If actuator feedback is available:
    #obs = data[:,11:15]
    # If testing the actuator response:
    #servotest = data[:,15]

    # Actuator dynamics
    cmd_a = sp.signal.lfilter([fo_c], [1, -(1-fo_c)], cmd, axis=0)
    #servotest_a = sp.signal.lfilter([fo_c], [1, -(1-fo_c)], servotest, axis=0)

    # b, a = signal.butter(2, 7/(sf/2), 'low', analog=False)
    # servotest_a = sp.signal.lfilter(b, a, servotest, axis=0)
    # servotest_a = actuator_dyn_2nd(servotest, 0.8, 70, 1/sf, 60000, 10000000)

    # Filtering
    b, a = signal.butter(2, 3.2/(freq/2), 'low', analog=False)
    gyro_f = sp.signal.lfilter(b, a, gyro, axis=0)
    cmd_af = sp.signal.lfilter(b, a, cmd_a, axis=0)
    accel_f = sp.signal.lfilter(b, a, accel, axis=0)

    # derivatives
    dgyro_f = np.vstack((np.zeros((1,3)), np.diff(gyro_f,1,axis=0)))*freq
    ddgyro_f = np.vstack((np.zeros((1,3)), np.diff(dgyro_f,1,axis=0)))*freq
    daccel_f = np.vstack((np.zeros((1,3)), np.diff(accel_f,1,axis=0)))*freq
    dcmd_af = np.vstack((np.zeros((1,4)), np.diff(cmd_af,1,axis=0)))*freq
    ddcmd_af = np.vstack((np.zeros((1,4)), np.diff(dcmd_af,1,axis=0)))*freq

    # Estimation of the control effectiveness
    c = fit_axis(dcmd_af[:,0:4], ddgyro_f[:,[0]], t, 'p', start, end) # 0, N)
    #c = fit_axis(dcmd_af[:,0:4], ddgyro_f[:,[1]], t, 'q', start, end) # 0, N)
    #c = fit_axis(dcmd_af[:,0:4], daccel_f[:,[2]], t, 'accel', start, end)
    # Use all commands, to see if there is crosstalk
    #c = fit_axis(np.hstack((dcmd_af[:,0:4], ddcmd_af[:,0:4]/freq)), ddgyro_f[:,[2]], t, 'r', start, end) # 0, end)

    optimize_axis(cmd[:,[1]], gyro[:,[0]], t, 'p', start, end, (b,a), freq)
    #optimize_axis(dcmd_af[:,[1]], ddgyro_f[:,[0]], t, 'p', start, end)

    #plt.figure()
    #plt.plot(t,cmd[:,1:2])
    #plt.plot(t,cmd_af[:,1:2])
    #plt.plot(t,dcmd_af[:,1:2])
    #plt.plot(t,dcmd_af[:,1:2]*0.035)
    #plt.plot(t,ddgyro_f[:,0:1])
    #plt.xlabel('t [s]')
    #plt.ylabel('command [PPRZ]')

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

