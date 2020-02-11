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
from scipy import signal, optimize
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure, show

#
# functions for actuators model
#

def first_order_model(sig, tau):
    '''
    Apply a first order filter with (discrete) time constant tau
    '''
    return sp.signal.lfilter([tau], [1, tau-1], sig, axis=0)

def rate_limit_model(sig, max_rate):
    '''
    Apply rate limiter of signal
    '''
    return sig # TODO

#
# Utility functions
#

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
    plt.figure()
    plt.plot(x[start:end], y[start:end])
    plt.xlabel('command [pprz]')
    plt.ylabel(label)
    if show:
        plt.show()

def print_results():
    pass


