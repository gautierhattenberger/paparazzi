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

'''
Utility functions for control effectiveness estimation
'''

import numpy as np
import scipy as sp
from scipy import optimize
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure, show

#
# functions for actuators model
#

def first_order_model(signal, tau):
    '''
    Apply a first order filter with (discrete) time constant tau
    '''
    return sp.signal.lfilter([tau], [1, tau-1], signal, axis=0)

def rate_limit_model(signal, max_rate):
    '''
    Apply rate limiter of signal
    '''
    return signal # TODO

#
# Utility functions
#

def diff_signal(signal, freq, order=1, filt=None):
    '''
    compute the nth-order derivative of a signal of fixed freq
    and by applying a filter if necessary
    '''
    #if filt is not None:
    #    signal = sp.signal.lfilter(filt[0], filt[1], signal, axis=0)

    #res = [signal]
    #print(np.shape(signal))
    #print(np.shape(res))
    #try:
    #    nb = np.shape(signal)[1]
    #except:
    #    nb = 1
    #for i in range(order):
    #    print(nb, np.shape(res[-1]))
    #    diff = np.diff(res[-1], 1, axis=0)
    #    print(diff, np.shape(diff))
    #    sigd = np.hstack((np.zeros((1,nb)), diff.reshape(1,np.shape(signal)[0]-1))) * freq
    #    res.append(sigd)
    #return res
    diff = np.diff(signal, order)
    res = np.hstack((np.zeros((1,order)), diff.reshape(1,len(diff)))) * pow(freq, order)
    #print(np.shape(res), res)
    return res


def apply_filter(filt_name, params, signal, freq, fo_c=None):
    '''
    apply a filter to an input signal based on the config (name + params)
    '''
    if filt_name == '1st_order':
        '''
        params = [tau]
        '''
        if fo_c is None:
            # use param from conf file
            return first_order_model(signal, params[0])
        else:
            # use function parameter
            return first_order_model(signal, fo_c)
    elif filt_name == 'rate_limit':
        '''
        params = [max_rate]
        '''
        return rate_limit_model(signal, params[0])
    elif filt_name == 'diff_signal':
        '''
        params = [order]
        '''
        return diff_signal(signal, freq, params[0])
    elif filt_name == 'butter':
        '''
        params = [order, Wn]
        '''
        b, a = sp.signal.butter(params[0], params[1]/(freq/2))
        return sp.signal.lfilter(b, a, signal, axis=0)
    else:
        print("Unknown filter type", filt_name)


def get_name_by_index(conf, type_, index):
    for key in conf['data']:
        el = conf['data'][key]
        if el['index'] == index and el['type'] == type_:
            return el['name']
    return None

def apply_format(conf, data):
    try:
        format_ = conf['format']
        if format_ == "float":
            return data
        if format_ == "bfp":
            res = conf['resolution']
            return data / pow(2, res)
        if format_ == "pprz":
            return data
        if format_ == "int":
            scale = conf['scale']
            return data * scale
        else:
            print("Unknown format:", format_)
            return data
    except:
        print("Format error:", conf)
        return data

#
# Display functions
#

def plot_results(x, y, t, start, end, label, show=False):
    '''
    plot two curves for comparison
    '''
    #print(np.shape(x), np.shape(y), np.shape(t))
    plt.figure()
    plt.plot(t, y)
    plt.plot(t, x)
    plt.xlabel('t [s]')
    plt.ylabel(label)
    #plt.figure()
    #plt.plot(x[start:end], y[start:end])
    #plt.xlabel('command [pprz]')
    #plt.ylabel(label)
    if show:
        plt.show()

def print_results():
    pass


#
# Optimization functions
#

def fit_axis(x, y, axis, start, end):
    c = np.linalg.lstsq(x[start:end], y[start:end])#, rcond=None)
    print(axis)
    print(c[0]*1000)
    return c[0]

