#!/usr/bin/env python

import sys, subprocess

NETWORK_ID = 0

keys = [
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE01FE0000004B120001FE00F8E6A0A2", #1
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE02FE0000004B120002FE00F8E6A0A0", #2
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE03FE0000004B120003FE00F8E6A09E", #3
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE04FE0000004B120004FE00F8E6A09C", #4
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE05FE0000004B120005FE00F8E6A09A", #5
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE06FE0000004B120006FE00F8E6A098", #6
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE07FE0000004B120007FE00F8E6A096", #7
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE08FE0000004B120008FE00F8E6A094", #8
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE09FE0000004B120009FE00F8E6A092", #9
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFE0AFE0000004B12000AFE00F8E6A090", #10
]

spw_keys = [
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEC0FE0000004B1200C0FE00F8E6A024", # 192 (0xC0)
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEC1FE0000004B1200C1FE00F8E6A022", # 193 (0xC1)
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEC2FE0000004B1200C2FE00F8E6A020", # 194 (0xC2)
    ":1EFF8800FEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEC3FE0000004B1200C3FE00F8E6A01E" # 195 (0xC3)
]


def get_key(node_id):
    if node_id < 192:
        key = keys[node_id - 1]
        print("gateway", node_id, key)
    else:
        key = spw_keys[node_id - 192]
        print("wearable", node_id, key)
    return key

def set_key(filename, outfilename, id):
    print("set key: using ID {}".format(id))

    key = get_key(id)

    with open(filename, "r") as inf:
        with open(outfilename, "w") as outf:
            for line in inf.readlines():
                if line[:9] == ":1EFF8800":
                    outf.write(key + "\n")
                else:
                    outf.write(line)

def usage():
    print("Usage: key.py filename node_id")
    print("The node ID or the filename was not supplied in the correct form, aborting")
    sys.exit(-1)
                    
def main():
    if len(sys.argv) < 3:
        usage()

    filename = sys.argv[1]
    try:
        id = int(sys.argv[2])
    except:
        usage()

    set_key(filename, filename + ".out", id)

    subprocess.call(["mv", filename + ".out", filename])

if __name__ == '__main__':
    main()
