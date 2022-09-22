#!/usr/bin/python3

'''
Tools to analyse perf_profil module output
Provides performance profiling information on the AP functions
'''

import sys
import numpy as np
import matplotlib.pyplot as plt

sysclk = 216 # MHz

def rtc2us(t):
    return float(((int(t) - 1) / sysclk) + 1)

def rtc2s(t):
    return float(((int(t) - 1) / sysclk) + 1)*1e-6

if len(sys.argv) == 2:
    data_file = sys.argv[1]
else:
    print("Please provide data file")
    exit(1)

time_vector = {
        "periodic_start":   (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "main":             (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "ap_gen":           (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "ap_static":        (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "sensors":          (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "estimation":       (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "sensors":          (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "control":          (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "default":          (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "core":             (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "attitude":         (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "modules_sync":     (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "modules":          (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "radio":            (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "failsafe":         (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "monitor":          (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "electrical":       (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "telemetry":        (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "baro":             (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "periodic_end":     (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "event_start":      (np.zeros(0), np.zeros(0), np.zeros(0), None),
        "event_end":        (np.zeros(0), np.zeros(0), np.zeros(0), None)
        }
last_time = None
last_data = None

event_vector = {
        "event":        None,
        "periodic":     None,
        "sensors":      None,
        "radio":        None,
        "gnc":          None,
        "core":         None,
        "telemetry":    None
        }

with open(data_file) as f:
    for line in f:
        data = line.split()
        if len(data) == 3 and data[0] == 'PPT':
            if last_data is None:
                last_data = data
                continue
            try:
                name = last_data[1]
                time = int(last_data[2])
                new_time = int(data[2])

                if not (name in ['periodic_start', 'periodic_end', 'event_start', 'event_end', 'event']):
                    (period, duty, tv, previous) = time_vector[name]
                    if previous is None:
                        time_vector[name] = (period, duty, rtc2us(new_time), time)
                    else:
                        p = time - previous
                        dt = new_time - time
                        if p < 0:
                            p = p + 2**32
                        time_vector[name] = (
                                np.append(period, rtc2us(p)),
                                np.append(duty, rtc2us(dt)),
                                np.append(tv, rtc2us(new_time)),
                                time)
                elif not (name in ['periodic_start', 'periodic_end', 'event']):
                    split = name.split('_', 1)
                    base = split[0]
                    tail = split[1]
                    if tail == "end":
                        (period, duty, tv, previous) = time_vector[name]
                        (_, _, _, start) = time_vector["{}_start".format(base)]
                        #print(name, base, tail, previous, start)
                        if previous is None:
                            time_vector[name] = (period, duty, rtc2us(new_time), time)
                        elif start is not None:
                            p = time - previous
                            dt = new_time - start
                            #print(name, dt, df, time, new_time, previous)
                            if p < 0:
                                p = p + 2**32
                            time_vector[name] = (
                                    np.append(period, rtc2us(p)),
                                    np.append(duty, rtc2us(dt)),
                                    np.append(tv, rtc2us(new_time)),
                                    time)
                    else: # tail = start
                        time_vector[name] = (np.zeros(0), np.zeros(0), np.zeros(0), time)
                else:
                    pass

                last_data = data
                #if data[1] == 'main':
                #    time_vector[data[1]] = np.append(time_vector[data[1]], [int(data[2])])
                #    #print(data[1], data[2], time_vector[data[1]])
            except KeyboardInterrupt:
                print("stop loop by hand")
                break
            except:
                print("invalid time or name at line", line)
        elif len(data) == 8 and data[0] == 'PPTE':
            try:
                name = data[1]
                nb_sample = int(data[2])
                nb_over = int(data[3])
                dt = rtc2us(data[4])
                duty = float(data[7])
                dt_min = rtc2us(data[5])
                dt_max = rtc2us(data[6])
                if event_vector[name] is None:
                    event_vector[name] = (
                            [np.array(nb_sample)], np.array(nb_over),
                            np.array(dt / float(nb_sample)),
                            np.array(duty / float(nb_sample)),
                            np.array(dt_min), np.array(dt_max))
                else:
                    event_vector[name] = (
                            np.append(event_vector[name][0], nb_sample),
                            np.append(event_vector[name][1], nb_over),
                            np.append(event_vector[name][2], dt / float(nb_sample)),
                            np.append(event_vector[name][3], duty / float(nb_sample)),
                            np.append(event_vector[name][4], dt_min),
                            np.append(event_vector[name][5], dt_max)
                            )
            except KeyboardInterrupt:
                print("stop loop by hand")
                break
            #except:
            #    print("invalid event at line", line)
        else:
            print("invalid data at line:", line)

# print results
print("name                (samples)\t| period (us) \t[std     ] | freq (Hz)\t| duty (us) \t[std     ] [min     ] [max     ] [nb over ] [%       ]")
for key in time_vector:
    if not (key in ['periodic_start', 'periodic_end', 'event_start']):
        (p,d,_,t) = time_vector[key]
        if t is not None and len(p) > 1:
            print("{:<20}({})\t| {:.2f} \t[{:<8.3f}] | {:<8.3f}\t| {:.3f} \t[{:<8.3f}] [{:<8.3f}] [{:<8.3f}] [{:<8d}] [{:<8.4f}]".format(key, len(p),
                np.mean(p), np.std(p), 1e6/np.mean(p),
                np.mean(d), np.std(d), np.min(d), np.max(d),
                (d > np.mean(p)).sum(), 100.*np.mean(d)/np.mean(p)))

        if False and len(d) > 1:
            i = np.arange(0, len(d))
            plt.figure()
            plt.plot(i, p)
            plt.xlabel('sample')
            plt.ylabel('usec')
            plt.title('{} period'.format(key))
            plt.figure()
            plt.plot(i, d)
            plt.xlabel('sample')
            plt.ylabel('usec')
            plt.title('{} duty'.format(key))
            plt.show()

for key in event_vector:
    data = event_vector[key]
    if data is not None:
        print("{:<20}({}*{})\t| {:.2f} \t[{:<8.3f}] | {:<8.3f}\t| {:.3f} \t[{:<8}] [{:<8.3f}] [{:<8.3f}] [{:<8d}] [{:<8}]".format(key,
            len(data[0]), data[0][0],
            np.mean(data[2]), np.std(data[2]), 1e6/np.mean(data[2]),
            np.mean(data[3]), 'N/A', np.min(data[4]), np.max(data[5]),
            np.sum(data[1]), 'N/A'))

        if True:
            t_end = len(data[0])*data[0][0]*np.mean(data[2])/1e6
            i = np.linspace(0, t_end, len(data[0]))
            plt.figure()
            #if key == "gnc":
            #    # only for Niquad_voliere_2.LOG
            #    plt.axvspan(0,120, facecolor='red', alpha=0.1)
            #    plt.axvspan(120,210, facecolor='orange', alpha=0.1)
            #    plt.axvspan(130,140, facecolor='orange', alpha=0.1)
            #    plt.axvspan(153,195, facecolor='orange', alpha=0.1)
            #    plt.axvspan(210,320, facecolor='green', alpha=0.1)
            #    plt.axvspan(228,300, facecolor='green', alpha=0.1)
            #    plt.axvspan(320,335, facecolor='red', alpha=0.1)
            #print(len(i),t_end,len(data[0]),len(data[4]), len(data[5]))
            plt.fill_between(i, data[4], data[5], alpha=0.5, linewidth=0)
            plt.plot(i, data[3])
            plt.xlabel('time (sec)')
            plt.ylabel('min, max, average (usec)')
            plt.title('{} duty'.format(key))
            plt.show()

        if False:
            i = np.arange(0, len(data[0]))
            plt.figure()
            plt.plot(i, data[2])
            plt.xlabel('sample')
            plt.ylabel('usec')
            plt.title('{} period'.format(key))
            plt.figure()
            plt.plot(i, data[3])
            plt.xlabel('sample')
            plt.ylabel('usec')
            plt.title('{} duty'.format(key))
            plt.figure()
            plt.plot(i, data[4])
            plt.xlabel('sample')
            plt.ylabel('usec')
            plt.title('{} min'.format(key))
            plt.figure()
            plt.plot(i, data[5])
            plt.xlabel('sample')
            plt.ylabel('usec')
            plt.title('{} max'.format(key))
            plt.show()

# DT
time_sensor = time_vector['sensors'][2]
time_estimation = time_vector['estimation'][2]
time_control = time_vector['control'][2]
#print(len(time_sensor), len(time_estimation), len(time_control))
#print((time_sensor), (time_estimation), time_control)
#print(time_estimation - time_sensor)
#print(time_control - time_sensor)
#size = np.min(np.array([len(time_sensor), len(time_estimation), len(time_control)]))
if False:
    i = np.arange(0, size)
    plt.figure()
    plt.plot(i, time_estimation[:size] - time_sensor[:size])
    plt.xlabel('sample')
    plt.ylabel('usec')
    plt.title('dt sensor - estimation')
    plt.figure()
    plt.plot(i, time_control[:size] - time_sensor[:size])
    plt.xlabel('sample')
    plt.ylabel('usec')
    plt.title('dt sensor - control')
    plt.show()

