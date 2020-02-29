import os

# the path of "examples/autonomous"
SELF_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ALGORITHMS = [
    "6tisch_min", # 1
    "orchestra_rb", # 2
]

ALGONAMES = {
    "6tisch_min" : "6tisch minimal",
    "orchestra_rb" : "Orchestra RB",
}

COLORS = {
    "6tisch_min": "red",
    "orchestra_sb": "green",
    "orchestra_rb": "slateblue",
}

FIRMWARE_TYPES = {
    "6tisch_min" : 1,
    "orchestra_rb" : 2,
}

SLOTFRAME_SIZES =[
    7,
    19,
]

SEND_INTERVAL_SEC =[
    10
]
