from mobiflight_variable_requests import MobiFlightVariableRequests
import time
import logging, logging.handlers
import ctypes
from ctypes import wintypes
from SimConnect import SimConnect
from SimConnect.Enum import SIMCONNECT_CLIENT_DATA_ID, SIMCONNECT_RECV_ID, SIMCONNECT_RECV_CLIENT_DATA


class SimConnectMobiFlight(SimConnect):

    def __init__(self, auto_connect=True, library_path=None):
        self.client_data_handlers = []
        if library_path:
            super().__init__(auto_connect, library_path)
        else:
            super().__init__(auto_connect)
        # Fix missing types
        self.dll.MapClientDataNameToID.argtypes = [wintypes.HANDLE, ctypes.c_char_p, SIMCONNECT_CLIENT_DATA_ID]


    def register_client_data_handler(self, handler):
        if not handler in self.client_data_handlers:
            logging.info("Register new client data handler")
            self.client_data_handlers.append(handler)


    def unregister_client_data_handler(self, handler):
        if handler in self.client_data_handlers:
            logging.info("Unregister client data handler")
            self.client_data_handlers.remove(handler)


    def my_dispatch_proc(self, pData, cbData, pContext):
        dwID = pData.contents.dwID
        if dwID == SIMCONNECT_RECV_ID.SIMCONNECT_RECV_ID_CLIENT_DATA:
            client_data = ctypes.cast(pData, ctypes.POINTER(SIMCONNECT_RECV_CLIENT_DATA)).contents
            for handler in self.client_data_handlers:
                handler(client_data)
        else:
            super().my_dispatch_proc(pData, cbData, pContext)


sm = SimConnectMobiFlight()
mf = MobiFlightVariableRequests(sm)

print("Starting high‑speed telemetry loop…")

# High‑resolution timer
t0 = time.perf_counter()
cycles = 0

while True:
    # Fetch as fast as possible
    pitch = mf.get("(A:PLANE PITCH DEGREES, Degrees)")
    roll  = mf.get("(A:PLANE BANK DEGREES, Degrees)")
    yaw   = mf.get("(A:PLANE HEADING DEGREES TRUE, Degrees)")
    air   = mf.get("(A:AIRSPEED INDICATED, Knots)")

    cycles += 1

    # Print every 10th cycle
    if cycles % 10 == 0:
        dt = time.perf_counter() - t0
        hz = cycles / dt if dt > 0 else 0

        print(
            f"[{hz:6.1f} Hz]  "
            f"pitch={pitch:7.2f}  "
            f"roll={roll:7.2f}  "
            f"yaw={yaw:7.2f}  "
            f"airspeed={air:7.2f}"
        )
