#!/usr/bin/python3

import os
import sys
import time
import subprocess
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as pl
import matplotlib.legend_handler as lh
from scipy import stats
from scipy.stats.stats import pearsonr
import numpy as np

from parameters import *

#
# This calculates and plots these metrics:
# - End-to-end PDR (delivery ratio)
# - Link-layer PAR (ACK ratio, between a node and all of its neighbors)
# - Radio Duty Cycle
#
# The radio duty cycle is based on Energest results.
# For Cooja motes, those are quite approximate
# (as e.g. clock drift is not simulated, SPI communication time is not simulated).
#

matplotlib.style.use('seaborn')
matplotlib.rcParams['pdf.fonttype'] = 42


OUT_DIR = "../plots"

DATA_DIRECTORY="../simulations"

DATA_FILE = "cached_data.json"

START_TIME_MINUTES = 1
END_TIME_MINUTES = 10

SEND_INTERVAL = 10

ROOT_ID = 1

ONLY_MEDIAN = True

EXPERIMENTS = ["sim"]

USE_CACHED_DATA = False

###########################################

MARKERS = ["o", "s", "X", "X", "X", "X"]
BASIC_MARKERS = ["o", "s", "X", "X", "X", "X"]

###########################################

def graph_ci(data, ylabel, filename):
    pl.figure(figsize=(6, 3.5))

    width = 0.15

    for i, a in enumerate(ALGORITHMS):
        algo_data = data[i]

        x = np.arange(len(algo_data)) - width / 2 + width * i
        pl.bar(x, algo_data, width, label=ALGONAMES[a], color=COLORS[a])

    pl.ylim(bottom=0)
    pl.xlabel("Slotframe size")
    pl.ylabel(ylabel)

    pl.xticks(range(len(SLOTFRAME_SIZES) + 1), [str(u) for u in SLOTFRAME_SIZES] + [""])

    legend = pl.legend()

    pl.savefig(OUT_DIR + "/" + filename, format='pdf',
               bbox_extra_artists=(legend,),
               bbox_inches='tight')

###########################################

def get_seqnums(send_interval, start_time = START_TIME_MINUTES):
    duration_seconds = (END_TIME_MINUTES - START_TIME_MINUTES) * 60
    num_packets = duration_seconds // send_interval
    if start_time <= START_TIME_MINUTES:
        first_packet = 1
    else:
        skipped_seqnums = (start_time - START_TIME_MINUTES) * 60 // send_interval
        first_packet = 1 + skipped_seqnums
    return (first_packet, num_packets)

###########################################

class MoteStats:
    def __init__(self, id):
        self.id = id
        self.seqnums = set()
        self.associated_at_minutes = None
        self.packets_tx = 0
        self.packets_ack = 0
        self.radio_on = 0
        self.radio_total = 0
        self.is_valid = False

    def calc(self, send_interval, first_seqnum, last_seqnum):
        if self.associated_at_minutes is None:
            print("node {} never associated".format(self.id))
            #self.is_valid = False
            #return

        if self.associated_at_minutes >= 30:
            print("node {} associated too late: {}, seqnums={}".format(
                self.id, self.associated_at_minutes, self.seqnums))
            #self.is_valid = False
            #return

        self.is_valid = True

        if self.packets_tx:
            self.par = 100.0 * self.packets_ack / self.packets_tx
        else:
            self.par = 0.0

        if self.associated_at_minutes >= 30:
            first_seqnum, last_seqnum = get_seqnums(send_interval)

        expected = (last_seqnum - first_seqnum) + 1
        actual = len(self.seqnums)

        if expected:
            self.pdr = 100.0 * actual / expected
        else:
            self.pdr = 0.0

        if self.radio_total:
            self.rdc = 100.0 * self.radio_on / self.radio_total
        else:
            print("warning: no radio duty cycle for {}".format(self.id))
            self.rdc = 0.0

###########################################

def process_file(filename, experiment, send_interval, is_testbed=False):
    motes = {}
    has_assoc = set()
    print(filename)

    in_initialization = True

    first_seqnum, last_seqnum = get_seqnums(send_interval)
    start_ts_unix = None

    with open(filename, "r") as f:
        for line in f:
            if is_testbed:
                fields1 = line.strip().split(";")
                fields2 = fields1[2].split()
                fields = fields1[:2] + fields2
            else:
                fields = line.strip().split()

            try:
                # in milliseconds
                if is_testbed:
                    ts_unix = float(fields[0])
                    if start_ts_unix is None:
                        start_ts_unix = ts_unix
                    ts_unix -= start_ts_unix
                    ts = int(float(ts_unix) * 1000)
                    node = int(fields[1][3:])
                else:
                    ts = int(fields[0]) // 1000
                    node = int(fields[1]) 
            except:
                # failed to extract timestamp
                continue

            if node not in motes:
                motes[node] = MoteStats(node)

            if "association done (1" in line:
                #print(line)
                has_assoc.add(node)
                motes[node].seqnums = set()
                motes[node].associated_at_minutes = (ts // 1000 + 59) // 60
                continue

            if in_initialization:
                # ignore the first N minutes of the test, while the network is being built
                if ts > START_TIME_MINUTES * 60 * 1000:
                    in_initialization = False
                else:
                    continue

            if node == ROOT_ID:
                # 314937392 1 [INFO: Node      ] seqnum=6 from=fd00::205:5:5:5
                if "seqnum=" in line:
                    #print(line)
                    sn = int(fields[5].split("=")[1])
                    if not (first_seqnum <= sn <= last_seqnum):
                        continue
                    if "=" not in fields[6]:
                        continue
                    fromtext, fromaddr = fields[6].split("=")
                    # this is needed to distinguish between "from" and "to" in the query example
                    if fromtext == "from":
                        fromnode = int(fromaddr.split(":")[-1], 16)
                        if is_testbed:
                            fromnode = node_id_to_mote_id.get(fromnode, 0)
                        if fromnode in has_assoc:
                            # account for the seqnum
                            motes[fromnode].seqnums.add(sn)
                            # print("add sn={} fromnode={}".format(sn, fromnode))

                # ignore the root, except for PDR
                continue

            if node not in has_assoc:
                continue

            # 600142000 28 [INFO: Link Stats] num packets: tx=0 ack=0 rx=0 to=0014.0014.0014.0014
            if "num packets" in line:
                tx = int(fields[7].split("=")[1])
                ack = int(fields[8].split("=")[1])
                rx = int(fields[9].split("=")[1])
                motes[node].packets_tx += tx
                motes[node].packets_ack += ack
                continue
          
            # 600073000:8 [INFO: Energest  ] Radio total :    1669748/  60000000 (27 permil)
            if "Radio total" in line:
                # only account for the period when data packets are sent
                if ts > START_TIME_MINUTES * 60 * 1000:
                    on = int(fields[8][:-1])
                    total = int(fields[9])
                    motes[node].radio_on += on
                    motes[node].radio_total += total
                    continue

            if "add packet failed" in line:
                # TODO: account for queue drops!
                continue

    r = []
    for k in motes:
        m = motes[k]
        if m.id == ROOT_ID:
            continue
        m.calc(send_interval, first_seqnum, last_seqnum)
        if m.is_valid:
            #print(" ", m.id, m.pdr, m.par)
            r.append((m.pdr, m.par, m.rdc))
#        else:
#            print("mote {} does not have valid PDR: packets={}".format(m.id, m.seqnums))
    return r

###########################################

def load_all(data_directory):
    si = SEND_INTERVAL
    data = {}
    for a in ALGORITHMS:
        data[a] = {}
        for sf in SLOTFRAME_SIZES:
            data[a][str(sf)] = {}
            for exp in EXPERIMENTS:
                data[a][str(sf)][exp] = {}

                t_pdr_results = []
                t_par_results = []
                t_rdc_results = []

                a_pdr_results = []
                a_par_results = []
                a_rdc_results = []

                path = os.path.join(data_directory,
                                    a,
                                    "sf_{}".format(sf),
                                    exp)

                for dirname in subprocess.check_output("ls -d " + path, shell=True).split():
                    resultsfile = os.path.join(dirname.decode("ascii"), "COOJA.testlog")

                    if not os.access(resultsfile, os.R_OK):
                        continue

                    r = process_file(resultsfile, exp, si)
                    pdr = [x[0] for x in r]
                    par = [x[1] for x in r]
                    rdc = [x[2] for x in r]
                    t_pdr_results += pdr
                    t_par_results += par
                    t_rdc_results += rdc
                    a_pdr_results.append(np.mean(pdr))
                    a_par_results.append(np.mean(par))
                    a_rdc_results.append(np.mean(rdc))


                    if ONLY_MEDIAN:
                        if len(a_pdr_results):
                            midpoint = len(a_pdr_results) // 2
                            print("pdr=", sorted(a_pdr_results))
                            pdr_metric = sorted(a_pdr_results)[midpoint]
                            par_metric = sorted(a_par_results)[midpoint]
                            rdc_metric = sorted(a_rdc_results)[midpoint]
                        else:
                            pdr_metric = 0
                            par_metric = 0
                            rdc_metric = 0
                    else:
                        pdr_metric = np.mean(t_pdr_results)
                        par_metric = np.mean(t_par_results)
                        rdc_metric = np.mean(t_rdc_results)

                    data[a][str(sf)][exp]["pdr"] = pdr_metric
                    data[a][str(sf)][exp]["par"] = par_metric
                    data[a][str(sf)][exp]["rdc"] = rdc_metric
    return data

###########################################

def access(data, a, sf, exp, metric):
    return data[a][str(sf)][exp][metric]

###########################################

def plot_all(data, exp):
    pdr_results = [[] for _ in ALGORITHMS]
    par_results = [[] for _ in ALGORITHMS]
    rdc_results = [[] for _ in ALGORITHMS]

    for sf in SLOTFRAME_SIZES:
        for i, a in enumerate(ALGORITHMS):
            print("Algorithm {}".format(ALGONAMES[a]))
            for exp in EXPERIMENTS:
                rdc_results[i].append(access(data, a, sf, exp, "rdc"))
                pdr_results[i].append(access(data, a, sf, exp, "pdr"))
                par_results[i].append(access(data, a, sf, exp, "par"))


    # plot the results
    graph_ci(pdr_results, "End-to-end PDR, %", "sim_pdr.pdf")
    graph_ci(par_results, "Link-layer PAR, %", "sim_par.pdf")
    graph_ci(rdc_results, "Radio duty cycle, %", "sim_rdc.pdf")


###########################################

def ensure_loaded(data_file, data_directory):
    if USE_CACHED_DATA and os.access(data_file, os.R_OK):
        print("Cached file found, using it directly")
        with open(data_file, "r") as f:
            data = json.load(f)
    else:
        if USE_CACHED_DATA:
            print("Cached file not found, parsing log files...")
        data = load_all(data_directory)
        with open(data_file, "w") as f:
            json.dump(data, f)
    return data

###########################################

def main():
    try:
        os.mkdir(OUT_DIR)
    except:
        pass

    data = ensure_loaded(DATA_FILE, DATA_DIRECTORY)

    for exp in EXPERIMENTS:
        plot_all(data, exp)

###########################################

if __name__ == '__main__':
    main()
    print("all done!")
