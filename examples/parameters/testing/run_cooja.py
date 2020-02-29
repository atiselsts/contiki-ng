#!/usr/bin/env python

import sys
import os
from subprocess import Popen, PIPE, STDOUT

# of the "autonomous" example
SELF_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# move two levels up
CONTIKI_PATH = os.path.dirname(os.path.dirname(SELF_PATH))

cooja = 'java -jar ' + os.path.normpath(os.path.join(SELF_PATH, '../../tools/cooja/dist/cooja.jar'))
cooja_input = './sim.csc'
cooja_output = 'COOJA.testlog'

#######################################################
def run_subprocess(args, input_string):
    retcode = -1
    stdoutdata = ''
    try:
        proc = Popen(args, stdout = PIPE, stderr = STDOUT, stdin = PIPE, shell = True)
        (stdoutdata, stderrdata) = proc.communicate(input_string)
        if not stdoutdata:
            stdoutdata = ''
        if stderrdata:
            stdoutdata += stderrdata
        retcode = proc.returncode
    except OSError as e:
        sys.stderr.write("runSubprocess OSError:" + str(e))
    except CalledProcessError as e:
        sys.stderr.write("runSubprocess CalledProcessError:" + str(e))
        retcode = e.returncode
    except Exception as e:
        sys.stderr.write("runSubprocess exception:" + str(e))
    finally:
        return (retcode, stdoutdata)


#######################################################
def execute_test(directory, cooja_file):
    os.chdir(directory)

    # cleanup
    try:
        os.rm(cooja_output)
    except:
        pass

    f = os.path.join(directory, cooja_file)
    args = " ".join([cooja, "-nogui=" + f, "-contiki=" + CONTIKI_PATH])
    sys.stdout.write("  Running Cooja, args={}\n".format(args))

    (retcode, output) = run_subprocess(args, '')
    if retcode != 0:
        sys.stderr.write("Failed, retcode=" + str(retcode) + ", output:")
        sys.stderr.write(output)
        return False

    sys.stdout.write("  Checking for output...")

    is_done = False
    with open(cooja_output, "r") as f:
        for line in f.readlines():
            line = line.strip()
            if line == "TEST OK":
                sys.stdout.write(" done.\n")
                is_done = True
                continue

    if not is_done:
        sys.stdout.write("  test failed.\n")
        return False

    sys.stdout.write(" test done\n")
    return True

#######################################################

def execute_test_run(directory):
    execute_test(directory, "sim.csc")

#######################################################

def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "./"
    execute_test_run(arg)
    print("all done")

#######################################################

if __name__ == '__main__':
    main()
