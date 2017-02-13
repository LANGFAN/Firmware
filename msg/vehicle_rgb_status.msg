# microseconds since system boot, needed to integrate
#uint64 timestamp
uint8 rgb_status    # rgb_status, 0~11, indicates apm state, and will be sent to RGBLed on ESC
#bool rgb_status_changed # flag, if true, send rgb_status to uavcan. try orb_check() first
