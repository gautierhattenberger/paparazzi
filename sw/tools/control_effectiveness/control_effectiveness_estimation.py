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
import csv
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt

import control_effectiveness_utils as ut

def process_data(conf, f_name, start, end, freq, fo_c=None, verbose=False):

    # Read data from log file
    data = genfromtxt(f_name, delimiter=',', skip_header=1)

    N = data.shape[0]
    if end < 0:
        end = N

    # Get number of inputs and outputs
    mixing = np.array(conf['mixing'])
    (nb_in, nb_out) = np.shape(mixing)
    if verbose:
        print("Nb of inputs:", nb_in)
        print("Nb of commands:", nb_out)
        print("Mixing matrix:")
        print(mixing)
    time = np.arange(end-start) / freq # default time vector if not in data
    inputs = np.zeros((end-start, nb_in))
    commands = np.zeros((end-start, nb_out))
    for key in conf['data']:
        el = conf['data'][key]
        t = el['type']
        idx = el['index']
        if t == 'input' and idx >= 0:
            if idx >= nb_in:
                print("Invalid input index for {}".format(el['name']))
                exit(1)
            inputs[:, idx] = ut.apply_format(el, data[start:end, int(key)])
            for (filt_name, params) in el['filters']:
                inputs[:,idx] = ut.apply_filter(filt_name, params, inputs[:, idx].copy(), freq)
        if t == 'command' and idx >= 0:
            if idx >= nb_out:
                print("Invalid command index for {}".format(el['name']))
                exit(1)
            commands[:, idx] = ut.apply_format(el, data[start:end, int(key)])
            for (filt_name, params) in el['filters']:
                commands[:,idx] = ut.apply_filter(filt_name, params, commands[:, idx].copy(), freq, fo_c)
        if t == 'timestamp':
            time = ut.apply_format(el, data[start:end, int(key)])

    for i in range(nb_in):
        name = ut.get_name_by_index(conf, 'input', i)
        cmd = np.multiply(commands, mixing[[i],:])
        fit = ut.fit_axis(cmd, inputs[:,[i]], name, 0, N)
        cmd_fit = np.dot(cmd, fit)
        ut.plot_results(cmd_fit, inputs[:,[i]], time, 0, N, name)

    plt.show()


def main():
    from argparse import ArgumentParser
    import json

    parser = ArgumentParser(description="Control effectiveness estimation tool")
    parser.add_argument("config", help="JSON configuration file")
    parser.add_argument("data", help="Log file for parameter estimation")
    parser.add_argument("-f", "--freq", dest="freq",
                      action="store", default=512,
                      help="Sampling frequency")
    parser.add_argument("-d", "--dyn", dest="dyn",
                      #action="store", default=0.08,
                      help="First order actuator dynamic (discrete time), 'None' for config file default")
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
    dyn = args.dyn
    if dyn is not None:
        dyn = float(dyn)

    with open(args.config, 'r') as f:
        conf = json.load(f)
        process_data(conf, args.data, start, end, freq, dyn, args.verbose)


if __name__ == "__main__":
    main()

