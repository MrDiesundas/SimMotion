# src/key_helper.py

import ctypes
import time

KEYEVENTF_KEYDOWN = 0x0000
KEYEVENTF_KEYUP   = 0x0002
VK_CONTROL = 0x11
VK_DELETE  = 0x2E
VK_INSERT  = 0x2D

class KeyHelper:
    """Utility class for sending keypresses to Windows."""

    @staticmethod
    def press_key(code):
        ctypes.windll.user32.keybd_event(code, 0, KEYEVENTF_KEYDOWN, 0)

    @staticmethod
    def release_key(code):
        ctypes.windll.user32.keybd_event(code, 0, KEYEVENTF_KEYUP, 0)

    @staticmethod
    def tap_key(code, delay=0.05):
        """Press and release a key with a small delay."""
        KeyHelper.press_key(code)
        time.sleep(delay)
        KeyHelper.release_key(code)

    @staticmethod
    def tap_combo(modifier, key, delay=0.05):
        """Press CTRL+DEL or similar combos."""
        KeyHelper.press_key(modifier)
        KeyHelper.press_key(key)
        time.sleep(delay)
        KeyHelper.release_key(key)
        KeyHelper.release_key(modifier)
