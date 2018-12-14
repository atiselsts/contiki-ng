# Overview

Instant is a TSCH schedule for data collection from mobile nodes. It allows efficient and flexible commucation communication between wearable nodes (that have bulk or high-rate data they wish to upload) and gateways (that collect the data; "access points" in the paper).

If you use this code for your research projects, you are encouraged to cite the following paper:

> A. Elsts, J. Pope, X. Fafoutis, R. Piechocki, G. Oikonomou. *Instant: A TSCH Schedule for Data Collection from Mobile Nodes*. International Conference on Embedded Wireless Systems and Networks (EWSN 2019), Beijing, February 2019.


# Implementation

The Instant scheduling mechanism is implemented in these files

* `os/net/mac/tsch/instant.h`
* `os/net/mac/tsch/instant.c`
* `os/net/mac/tsch/tsch-slot-operation.c`

There are also changes in other files under `os/net/mac/tsch` and `os/net/mac/framer`.

The files `os/net/mac/tsch/tcsh-schedule.{h,c}` have been removed.


# Building and uploading the software

Instant can function in a network with up to `INSTANT_NUM_GATEWAYS` gateways and up to `INSTANT_NUM_WEARABLES` wearables. By default `INSTANT_NUM_GATEWAYS=4` and `INSTANT_NUM_GATEWAYS=4`.

Let's assume you want to create the simplest network with 1 gateway and 1 wearable.

First build and upload the gateway:

    $ cd examples/instant/gw
    $ make node.upload A=1 PORT=<gateway USB port name>

Then the wearable:

    $ cd examples/instant/wearable
    $ make node.upload A=192 PORT=<wearable USB port name>

With the parameter `A` you pass the node ID of the gateway / wearable. The gateway node IDs are in range from 1 to `INSTANT_NUM_GATEWAYS`. The wearable IDs are in the range 192 (0xC0) to `192 + INSTANT_NUM_WEARABLES - 1`. The node ID is equal to the last byte of the node's IEEE address; the next to the last byte is equal to zero.


# Some interesting parameters

`INSTANT_CONF_IS_CONNECTION_MODE` - enable Instant Connection mode.

`INSTANT_CONF_INERTIA` - the larger, the less reluctant the gateways are to break the transient connections with the wearables. Default value 1 (other values are not discussed in the paper).

`INSTANT_CONF_ORCHESTRA_GREEDY` - emulates Orchestra. Also / only needs makefile changes (to enable RPL).

`INSTANT_SLOTFRAME_SIZE` - size of an Instant slotframe, default 50 slots.

`INSTANT_NUM_GATEWAYS`, `INSTANT_NUM_WEARABLES` - the max number of nodes.

`INSTANT_NUM_ACK_SUBSLOTS` - the max number of ACKs per a probing slot; $N_a$ in the paper. Default value 3.

`INSTANT_PACKET_SIZE` - the payload of a single unicast packet. Default 104 bytes.

`INSTANT_DATA_SIZE` - the size of the "data" to collect. Default 100000 bytes.

`INSTANT_FRESH_ROUNDS` - $T_{fresh}$ from the paper.

See `os/net/mac/tsch/instant.h` for these and other settings.
