import hid
import threading
import time

class FlightStick:
    VENDOR_ID  = 0x1B4F   # SparkFun
    PRODUCT_ID = 0x9206   # Pro Micro inside MaxFlightStick

    # Button → (byte_index, bit_index)
    BUTTON_MAP = {
        5:  (1, 4),
        6:  (1, 5),
        7:  (1, 6),
        8:  (1, 7),
        9:  (2, 0),
        10: (2, 1),
        13: (2, 4),
    }

    def __init__(self):
        self.device = hid.device()
        self.device.open(self.VENDOR_ID, self.PRODUCT_ID)
        self.device.set_nonblocking(True)

        # Track previous button states
        self.state = {btn: False for btn in self.BUTTON_MAP}

        # Callbacks for rising edge events
        self.callbacks = {btn: None for btn in self.BUTTON_MAP}

        # Start background thread
        self.running = True
        self.thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.thread.start()

    def close(self):
        self.running = False
        self.thread.join()
        self.device.close()

    def on_press(self, button, callback):
        """Register a callback for button press (low → high)."""
        if button in self.callbacks:
            self.callbacks[button] = callback

    def get_button(self, button):
        """Return current state of a button (True/False)."""
        return self.state.get(button, False)

    def _poll_loop(self):
        while self.running:
            data = self.device.read(64)
            if data:
                self._process_report(data)
            time.sleep(0.001)  # 1 kHz polling

    def _process_report(self, data):
        for button, (byte_idx, bit_idx) in self.BUTTON_MAP.items():
            byte_val = data[byte_idx]
            pressed = (byte_val & (1 << bit_idx)) != 0

            # Detect rising edge
            if pressed and not self.state[button]:
                if self.callbacks[button]:
                    self.callbacks[button]()  # trigger event

            self.state[button] = pressed
