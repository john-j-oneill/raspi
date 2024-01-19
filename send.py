import os
import can

with can.interface.Bus(channel = 'can0', bustype = 'socketcan') as can0:
	msg = can.Message(is_extended_id=False, arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7])
	can0.send(msg)
