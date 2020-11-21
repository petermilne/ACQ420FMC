#!/usr/bin/python
import epics
import time

hn="acq2106_201"

fmt="{}:{}:AI:CAL:ESLO"

old_state = "000"

def onChange(pvname=None, value=None, char_value=None, **kw):
    global old_state
    print("{}  {} = {}".format(pvname, old_state, value))
    old_state = value
    
state = epics.PV("{}:MODE:TRANS_ACT:STATE".format(hn))
state.add_callback(onChange)

while True:
    time.sleep(100)

for site in range(1,7):
    print(site)
