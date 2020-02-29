#!/usr/bin/python

import sys, os, copy, errno
import multiprocessing
import subprocess

from parameters import *

OUT_DIRECTORY = os.path.join(SELF_PATH, "simulations")

# tailor the workload depending on the number of cores;
# but leave some cores free for other things
NUM_CORES = multiprocessing.cpu_count() * 7 // 8

ENV = {
    "FIRMWARE_TYPE" : "1",
    "TSCH_SCHEDULE_CONF_DEFAULT_LENGTH" : "7",
    "ORCHESTRA_CONF_UNICAST_PERIOD" : "11",
}

########################################

def create_out_dir(name):
    try:
        os.mkdir(name)
    except IOError as e:
        if e.errno != errno.EEXIST:
           print("Failed to create " + name)
           print(e)
        pass
    except Exception as ex:
        pass

########################################

def generate_simulations(dirname, env, wildcards):
    makefile = open("Makefile.tmpl", "r").read()

    # replace the template symbols with their values
    for key in env:
        makefile = makefile.replace("@" + key + "@", str(env[key]))

    print("dirname=", dirname)

    filenames = []
    for fs in wildcards:
        fs = os.path.join(SELF_PATH, fs)
        try:
            filenames += subprocess.check_output("ls " + fs, shell=True).split()
        except Exception as ex:
            print(ex)

    print("filenames=", filenames)

    all_directories = []
    for filename in filenames:
        sim_name = os.path.basename(os.path.splitext(filename)[0])
        sim_dirname = os.path.join(dirname, sim_name)
        create_out_dir(sim_dirname)

        all_directories.append(sim_dirname)

        subprocess.call("cp " + filename + " " + sim_dirname + "/sim.csc", shell=True)
        with open(sim_dirname + "/Makefile", "w") as f:
            f.write(makefile)

        subprocess.call("cp ../project-conf.h " + sim_dirname, shell=True)
        subprocess.call("cp ../node.c " + sim_dirname, shell=True)

    return all_directories

########################################

def generate_runner(description, all_directories, do_overwrite):
    open_as = "w" if do_overwrite else "a+"
    with open("run-" + description + ".sh", open_as) as f:
        if do_overwrite:
            f.write("#!/bin/bash\n")

        for i, dirname in enumerate(all_directories):
            f.write("./run_cooja.py " + dirname + " &\n")
            if i % NUM_CORES == NUM_CORES - 1:
                f.write("wait\n\n")
        f.write("wait\n")

    os.chmod("run-" + description + ".sh", 0o755)

########################################
def main():
    # sparse, medium, and dense networks - depending on the neighbor count
    all_directories = []
    dirname1 = OUT_DIRECTORY
    create_out_dir(dirname1)
    for a in ALGORITHMS:
        firmware_type = FIRMWARE_TYPES[a]
        dirname2 = os.path.join(dirname1, a)
        create_out_dir(dirname2)
        for ss in SLOTFRAME_SIZES:
            dirname3 = os.path.join(dirname2, "sf_{}".format(ss))
            create_out_dir(dirname3)
            cenv = copy.copy(ENV)
            cenv["FIRMWARE_TYPE"] = str(firmware_type)
            cenv["ORCHESTRA_CONF_UNICAST_PERIOD"] = str(ss)
            cenv["TSCH_SCHEDULE_CONF_DEFAULT_LENGTH"] = str(ss)
            cenv["SEND_INTERVAL_SEC"] = str(SEND_INTERVAL_SEC[0])
            all_directories += generate_simulations(dirname3, cenv, ["sim.csc"])
    generate_runner("all", all_directories, True)


########################################
if __name__ == '__main__':
    main()
    print("all done!")

