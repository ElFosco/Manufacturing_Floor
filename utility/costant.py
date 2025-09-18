
KIT, ASM, PACK = 1, 2, 3
K1, K2, B1, B2, P , KB, BP, B12 = "K1", "K2", "B1", "B2", "P", "KB", "BP", "B12"

FLASHLIGHT_CLIPPED = 0
FLASHLIGHT_SCREWS = 1


map_type_stage = { 0:0,
                   1:1,
                   2:1,
                   3:2
}


HUMAN_JOBS_TIME = {
    "K1":  20,
    "K2":  10,
    "B1":  30,
    "B2":  20,
    "P":   5,
    "T":   5
}

WS_KITTING = {
    'type': KIT,
    'subtype': None,
    'jobs':
        {"K1": 10},
}

WS_BUILDING_1 = {
    'type': ASM,
    'subtype': FLASHLIGHT_CLIPPED,
    'jobs':
        {"B1": 10},
}

WS_BUILDING_2 = {
    'type': ASM,
    'subtype': FLASHLIGHT_SCREWS,
    'jobs':
        {"B1": 20,
         "B2": 15}
}

WS_PALLETTING = {
    'type': PACK,
    'subtype': None,
    'jobs':
        {"P": 15}
}



TYPE_TO_SUBTYPES = {
        "KIT": [],
        "ASSEMBLY": ["GRIP", "GRIP & SCREW"],
        "PALLETING": [],
        "HUMAN" : []
    }
