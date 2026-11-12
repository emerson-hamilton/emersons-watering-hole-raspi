#!/usr/bin/env python3
"""
Raspberry Pi Camera RTSP Server using MediaMTX (built-in rpiCamera source)
+ PIR motion-triggered still capture via GPIO, confirmed by video motion
detection (VMD) during daytime, with a burned-in overlay.

For Raspberry Pi OS Trixie
Requires: MediaMTX, ffmpeg, gpiozero, lgpio, Pillow (PIL), OpenCV, numpy, astral
Install:
  sudo apt-get install ffmpeg python3-gpiozero python3-lgpio python3-pil python3-opencv python3-numpy
  # astral is used for sunrise/sunset calculation. If not packaged for your
  # release, install into a venv, or: pip3 install --break-system-packages astral
  sudo apt-get install python3-astral
"""

import subprocess
import time
import sys
import signal
import socket
import os
import threading
import shutil
from datetime import datetime
from collections import deque
from zoneinfo import ZoneInfo

try:
    from gpiozero import DigitalInputDevice
    GPIOZERO_AVAILABLE = True
except ImportError:
    GPIOZERO_AVAILABLE = False

try:
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from astral import LocationInfo
    from astral.sun import sun
    ASTRAL_AVAILABLE = True
except ImportError:
    ASTRAL_AVAILABLE = False


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
RTSP_PORT = 8554
RTSP_PATH = "cam"  # No leading slash for MediaMTX
VIDEO_WIDTH = 1920
VIDEO_HEIGHT = 1080
VIDEO_FPS = 10
VIDEO_BITRATE = 3000000  # 3 Mbps

# Authentication settings
RTSP_USERNAME = "admin"
RTSP_PASSWORD = os.environ["RTSP_PASSWORD"]

# PIR / motion capture settings
PIR_GPIO_PIN = 27                       # BCM pin number
PHOTO_DIR = "/mnt/usbdrive/img"         # Where snapshots are saved
USB_MOUNT_POINT = "/mnt/usbdrive"       # The mount point that must be active before PHOTO_DIR is used
CAPTURE_COOLDOWN = 5                    # Minimum seconds between captures
SNAPSHOT_TIMEOUT = 10                   # Max seconds to wait for on-demand ffmpeg snapshot (fallback path)

# Compensates for end-to-end pipeline latency (camera encode -> RTSP -> network
# -> ffmpeg decode) so the saved frame reflects the moment of the trigger
# rather than a moment slightly *before* it. Measure empirically - tune as needed.
CAPTURE_DELAY_SECONDS = 2.2

# Wireless radios
DISABLE_WIRELESS_RADIOS = True  # blocks wifi + bluetooth via rfkill at startup
RFKILL_PATH = "/usr/sbin/rfkill"  # run `which rfkill` to confirm on your system

# Keyframe interval for the camera source, in frames. Lower = faster access to
# a decodable frame for any RTSP client, at the cost of slightly higher bitrate.
IDR_PERIOD = VIDEO_FPS  # roughly one keyframe per second

# Persistent frame-buffer settings (this is what makes captures near-instant)
FRAME_BUFFER_PATH = "/dev/shm/latest_frame.jpg"  # tmpfs (RAM), avoids flash wear from constant overwrites
FRAME_BUFFER_FPS = 1         # how often the buffer file is refreshed
FRAME_BUFFER_MAX_AGE = 3     # seconds - if the buffer is older than this, fall back to on-demand capture

# ffmpeg decode tuning - reduces CPU load from continuously decoding the
# buffer stream. Set FFMPEG_DECODER to None to fall back to ffmpeg's default
# software decoder if hardware decode causes issues on your system.
FFMPEG_DECODER = "h264_v4l2m2m"  # Pi 4 hardware-accelerated H.264 decoder
FFMPEG_SKIP_NONKEYFRAMES = True  # only decode I-frames (matches IDR_PERIOD)

# The VMD reader is a second, simultaneous decode consumer of the same stream.
# Many Pi boards only support a single concurrent hardware (v4l2m2m) decode
# session - running two at once can cause both processes to fight over the
# decoder and crash-restart in a loop. VMD's resolution/fps are tiny, so
# software decode here costs very little and avoids the conflict entirely.
VMD_FFMPEG_DECODER = None  # keep as None (software) unless you've confirmed concurrent HW decode works

# Overlay settings - burned into the bottom of each saved snapshot
OVERLAY_ENABLED = True
OVERLAY_BAR_HEIGHT = 60                  # px, height of the semi-transparent bar
OVERLAY_BAR_COLOR = (0, 0, 0)            # RGB, black
OVERLAY_BAR_OPACITY = 128                # 0-255, 128 = ~50% opacity
OVERLAY_ICON_PATH = "/home/<pi-user>/Naturebytes/Scripts/img/pawprints_60x60.png"
OVERLAY_ICON_SIZE = 60                   # px, expected to match OVERLAY_BAR_HEIGHT
OVERLAY_ICON_LEFT_MARGIN = 10            # px, gap between frame edge and icon
OVERLAY_TEXT_LEFT = "Emerson's Wildlife Cam"
OVERLAY_TEXT_GAP_AFTER_ICON = 15         # px, gap between icon and left text
OVERLAY_TEXT_RIGHT_MARGIN = 15           # px, gap between right text and frame edge
OVERLAY_TEXT_COLOR = (255, 255, 255, 255)  # white
OVERLAY_FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
OVERLAY_FONT_SIZE_RATIO = 0.55           # fraction of bar height used to size the font
OVERLAY_TIMESTAMP_FORMAT = "%Y-%m-%d %-I:%M %p"  # 12-hour, no seconds, no leading zero on hour

# Video motion detection (VMD) - confirms daytime PIR triggers using motion
# detected in a small/low-fps analysis stream pulled from the same RTSP feed.
# This never runs at night - PIR alone is trusted then.
VMD_ENABLED = True
VMD_WIDTH = 320
VMD_HEIGHT = int(round(VMD_WIDTH * VIDEO_HEIGHT / VIDEO_WIDTH / 2) * 2)  # proportional, kept even
VMD_FPS = 3
VMD_DIFF_THRESHOLD = 25          # 0-255, per-pixel grayscale difference to count as "changed"
VMD_MIN_CONTOUR_AREA = 60        # px^2 in the downscaled frame; rejects small noise blobs
VMD_EVENT_BUFFER_SECONDS = 5     # how long detected-motion timestamps are retained
CORRELATION_WINDOW_SECONDS = 1.5 # +/- window around a PIR trigger to look for a corroborating VMD event

# Day/night boundary via sunrise/sunset (VMD confirmation only applies during the day)
LATITUDE = 0.0                       # <-- set to your location
LONGITUDE = 0.0                      # <-- set to your location
TIMEZONE_NAME = "America/New_York"   # <-- set to your local IANA timezone name
DAY_NIGHT_MARGIN_MINUTES = 30        # extends "daytime" this many minutes past sunset / before sunrise,
                                      # since thermal effects (your original false-trigger cause) can linger


class RTSPServer:
    def __init__(self):
        self.mediamtx_proc = None
        self.running = False
        self.config_path = '/tmp/mediamtx.yml'
        self.motion_sensor = None
        self.last_capture_time = 0
        self.capture_lock = threading.Lock()
        self.frame_buffer_proc = None

        # VMD state
        self.vmd_proc = None
        self.vmd_reader_running = False
        self.vmd_thread = None
        self.vmd_events = deque()
        self.vmd_lock = threading.Lock()
        self.vmd_functional = False  # only True once the VMD reader is confirmed running

        # Sunrise/sunset cache (recomputed once per calendar day)
        self._sun_cache_date = None
        self._sun_cache = None

    # -----------------------------------------------------------------
    # MediaMTX configuration / lifecycle
    # -----------------------------------------------------------------
    def create_config(self):
        """Create MediaMTX configuration file"""
        config_content = f"""# MediaMTX Configuration

# RTSP server settings
rtspAddress: :{RTSP_PORT}
protocols: [tcp]

# API settings
api: yes
apiAddress: :9997

# Authentication
authMethod: internal

# Logging
logLevel: warn
logDestinations: [stdout]

# Path settings
paths:
  {RTSP_PATH}:
    # This path accepts streams published via RTSP
    source: rpiCamera
    rpiCameraWidth: {VIDEO_WIDTH}
    rpiCameraHeight: {VIDEO_HEIGHT}
    rpiCameraFPS: {VIDEO_FPS}
    rpiCameraBitrate: {VIDEO_BITRATE}
    rpiCameraIDRPeriod: {IDR_PERIOD}
    rpiCameraAfMode: manual
    rpiCameraLensPosition: 0.600000

authInternalUsers:
  # Username. 'any' means any user, including anonymous ones.
  - user: any
    # Password. Not used in case of 'any' user.
    pass:
    # IPs or networks allowed to use this user. An empty list means any IP.
    ips: []
    # List of permissions.
    permissions:
      # Available actions are: publish, read, playback, api, metrics, pprof.
      - action: publish
        path:
      - action: read
        path:
      - action: playback
        path:
      - action: api
        path:
"""

        with open(self.config_path, 'w') as f:
            f.write(config_content)

        print(f"Created configuration file at {self.config_path}")

    def find_rtsp_server(self):
        """Find mediamtx or rtsp-simple-server binary"""
        bins = ['mediamtx', 'rtsp-simple-server']
        for bin_name in bins:
            try:
                result = subprocess.run(['which', bin_name], capture_output=True, text=True)
                if result.returncode == 0:
                    return bin_name
            except:
                pass
        return None

    def _decoder_input_args(self, decoder=FFMPEG_DECODER):
        """Build the ffmpeg input-side args that control how the RTSP stream
        is decoded. These must appear BEFORE -i on the command line."""
        args = []
        if FFMPEG_SKIP_NONKEYFRAMES:
            args += ['-skip_frame', 'nokey']
        if decoder:
            args += ['-c:v', decoder]
        return args

    # -----------------------------------------------------------------
    # Persistent frame buffer (keeps a constantly-refreshed "latest frame"
    # file so motion captures don't pay RTSP connection/keyframe latency)
    # -----------------------------------------------------------------
    def start_frame_buffer(self):
        """Start a background ffmpeg process that continuously overwrites
        FRAME_BUFFER_PATH with the most recent frame from the stream."""
        rtsp_url = f"rtsp://127.0.0.1:{RTSP_PORT}/{RTSP_PATH}"

        ffmpeg_cmd = [
            'ffmpeg',
            '-rtsp_transport', 'tcp',
            '-fflags', 'nobuffer',
            '-flags', 'low_delay',
        ] + self._decoder_input_args() + [
            '-i', rtsp_url,
            '-vf', f'fps={FRAME_BUFFER_FPS}',
            '-update', '1',
            '-q:v', '2',
            '-y', FRAME_BUFFER_PATH
        ]

        self.frame_buffer_proc = subprocess.Popen(
            ffmpeg_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print(f"Started persistent frame buffer -> {FRAME_BUFFER_PATH} ({FRAME_BUFFER_FPS} fps)")

    def restart_frame_buffer_if_needed(self):
        """Called periodically from the monitor loop."""
        if self.frame_buffer_proc and self.frame_buffer_proc.poll() is not None:
            print("Frame buffer process terminated, restarting...")
            self.start_frame_buffer()

    # -----------------------------------------------------------------
    # Video motion detection (VMD) - daytime confirmation layer
    # -----------------------------------------------------------------
    def start_vmd_reader(self):
        """Start a background ffmpeg process that decodes a small, low-fps
        copy of the stream to raw grayscale frames on stdout, and a Python
        thread that analyzes those frames for motion."""
        if not VMD_ENABLED:
            return

        if not CV2_AVAILABLE:
            print("WARNING: OpenCV/numpy not installed - VMD confirmation disabled.")
            print("Install with: sudo apt-get install python3-opencv python3-numpy")
            self.vmd_functional = False
            return

        rtsp_url = f"rtsp://127.0.0.1:{RTSP_PORT}/{RTSP_PATH}"

        ffmpeg_cmd = [
            'ffmpeg',
            '-rtsp_transport', 'tcp',
            '-fflags', 'nobuffer',
            '-flags', 'low_delay',
        ] + self._decoder_input_args(VMD_FFMPEG_DECODER) + [
            '-i', rtsp_url,
            '-vf', f'fps={VMD_FPS},scale={VMD_WIDTH}:{VMD_HEIGHT}',
            '-pix_fmt', 'gray',
            '-f', 'rawvideo',
            'pipe:1'
        ]

        try:
            self.vmd_proc = subprocess.Popen(
                ffmpeg_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            print(f"WARNING: failed to start VMD reader: {e}")
            self.vmd_functional = False
            return

        self.vmd_reader_running = True
        self.vmd_thread = threading.Thread(target=self._vmd_reader_loop, daemon=True)
        self.vmd_thread.start()
        self.vmd_functional = True
        print(f"Started VMD reader ({VMD_WIDTH}x{VMD_HEIGHT} @ {VMD_FPS}fps)")

    def restart_vmd_reader_if_needed(self):
        """Called periodically from the monitor loop."""
        if VMD_ENABLED and CV2_AVAILABLE and self.vmd_proc and self.vmd_proc.poll() is not None:
            print("VMD reader process terminated, restarting...")
            self.vmd_reader_running = False
            self.start_vmd_reader()

    def _vmd_reader_loop(self):
        """Runs in its own thread. Reads raw grayscale frames from the VMD
        ffmpeg process, compares consecutive frames (after compensating for
        global brightness/exposure shifts), and records a timestamp whenever
        a large-enough contiguous change is found."""
        frame_size = VMD_WIDTH * VMD_HEIGHT
        buf = b""
        prev_frame = None

        while self.vmd_reader_running:
            try:
                chunk = self.vmd_proc.stdout.read(frame_size - len(buf))
            except Exception:
                break

            if not chunk:
                break  # process ended

            buf += chunk
            if len(buf) < frame_size:
                continue

            frame = np.frombuffer(buf, dtype=np.uint8).reshape((VMD_HEIGHT, VMD_WIDTH))
            buf = b""

            if prev_frame is not None:
                try:
                    # Compensate for a global brightness/exposure shift between
                    # frames (e.g. auto-exposure reacting to a cloud) before
                    # differencing, so that shift alone isn't read as motion.
                    mean_shift = float(frame.mean()) - float(prev_frame.mean())
                    adjusted = frame.astype(np.int16) - int(round(mean_shift))
                    adjusted = np.clip(adjusted, 0, 255).astype(np.uint8)

                    diff = cv2.absdiff(adjusted, prev_frame)
                    _, thresh = cv2.threshold(diff, VMD_DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
                    thresh = cv2.dilate(thresh, None, iterations=2)
                    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    if any(cv2.contourArea(c) >= VMD_MIN_CONTOUR_AREA for c in contours):
                        now = time.time()
                        with self.vmd_lock:
                            self.vmd_events.append(now)
                            cutoff = now - VMD_EVENT_BUFFER_SECONDS
                            while self.vmd_events and self.vmd_events[0] < cutoff:
                                self.vmd_events.popleft()
                except Exception as e:
                    print(f"WARNING: VMD frame analysis error: {e}")

            prev_frame = frame

    def _vmd_confirms(self, trigger_time):
        """True if a VMD motion event was recorded within CORRELATION_WINDOW_SECONDS
        of trigger_time (in either direction)."""
        with self.vmd_lock:
            events = list(self.vmd_events)
        return any(abs(t - trigger_time) <= CORRELATION_WINDOW_SECONDS for t in events)

    # -----------------------------------------------------------------
    # Day / night determination (sunrise/sunset)
    # -----------------------------------------------------------------
    def _get_sun_times(self, for_date):
        if self._sun_cache_date == for_date and self._sun_cache is not None:
            return self._sun_cache

        loc = LocationInfo(latitude=LATITUDE, longitude=LONGITUDE, timezone=TIMEZONE_NAME)
        s = sun(loc.observer, date=for_date, tzinfo=ZoneInfo(TIMEZONE_NAME))

        self._sun_cache_date = for_date
        self._sun_cache = s
        return s

    def is_daytime(self):
        """Returns True if it's currently daytime (with DAY_NIGHT_MARGIN_MINUTES
        of padding around sunrise/sunset). Returns False (i.e. "trust PIR alone")
        if this can't be determined, since that's the safer failure mode - it
        means we never block a real capture due to a sunrise/sunset calc error."""
        if not ASTRAL_AVAILABLE:
            return False

        try:
            tz = ZoneInfo(TIMEZONE_NAME)
            now = datetime.now(tz)
            s = self._get_sun_times(now.date())

            from datetime import timedelta
            margin = timedelta(minutes=DAY_NIGHT_MARGIN_MINUTES)
            day_start = s['sunrise'] - margin
            day_end = s['sunset'] + margin

            return day_start <= now <= day_end
        except Exception as e:
            print(f"WARNING: sunrise/sunset calculation failed ({e}), defaulting to nighttime behavior.")
            return False

    # -----------------------------------------------------------------
    # PIR motion capture
    # -----------------------------------------------------------------
    def setup_gpio(self):
        """Set up the PIR sensor on GPIO_PIN and register the rising-edge callback"""
        if not GPIOZERO_AVAILABLE:
            print("\nWARNING: gpiozero is not installed - motion capture disabled.")
            print("Install it with: sudo apt-get install python3-gpiozero python3-lgpio\n")
            return False

        os.makedirs(PHOTO_DIR, exist_ok=True)

        try:
            # PIR output is logic high when triggered -> rising edge = motion detected
            self.motion_sensor = DigitalInputDevice(PIR_GPIO_PIN, pull_up=False)
            self.motion_sensor.when_activated = self.on_motion_detected
            print(f"PIR sensor configured on GPIO {PIR_GPIO_PIN}")
            print(f"Snapshots will be saved to: {PHOTO_DIR}")
            return True
        except Exception as e:
            print(f"Failed to set up GPIO pin {PIR_GPIO_PIN}: {e}")
            return False

    def on_motion_detected(self):
        """Called by gpiozero on the rising edge of the PIR pin. Runs in its own
        thread so it never blocks further GPIO events."""
        trigger_time = time.time()
        if trigger_time - self.last_capture_time < CAPTURE_COOLDOWN:
            # Still in cooldown period, ignore this trigger
            return

        self.last_capture_time = trigger_time
        print(f"\n[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Motion detected on GPIO {PIR_GPIO_PIN}!")

        handler_thread = threading.Thread(target=self._handle_trigger, args=(trigger_time,), daemon=True)
        handler_thread.start()

    def _handle_trigger(self, trigger_time):
        """Decides whether to actually capture: at night (or if VMD is
        disabled/unavailable), PIR alone is trusted. During the day, requires
        a corroborating video-motion event within CORRELATION_WINDOW_SECONDS."""
        daytime = self.is_daytime()

        if VMD_ENABLED and daytime:
            if not self.vmd_functional:
                print("WARNING: daytime trigger, but VMD is not functional - capturing without confirmation.")
            else:
                print(f"Daytime trigger - waiting {CORRELATION_WINDOW_SECONDS}s for VMD confirmation...")
                time.sleep(CORRELATION_WINDOW_SECONDS)
                if not self._vmd_confirms(trigger_time):
                    print("No corroborating video motion found - treating as a false trigger, skipping capture.")
                    return
                print("VMD confirmed motion - proceeding with capture.")
        else:
            print("Nighttime trigger (or VMD disabled) - trusting PIR alone.")

        self._delayed_capture(trigger_time)

    def _delayed_capture(self, trigger_time):
        """Waits until CAPTURE_DELAY_SECONDS have elapsed since trigger_time
        (accounting for any time already spent, e.g. in VMD confirmation),
        then grabs the frame. This keeps total trigger-to-capture timing
        consistent regardless of which path was taken above."""
        elapsed = time.time() - trigger_time
        remaining = CAPTURE_DELAY_SECONDS - elapsed
        if remaining > 0:
            time.sleep(remaining)
        self.capture_photo()

    def capture_photo(self):
        """Save a snapshot as a JPEG. Preferentially copies the persistent
        frame-buffer file (near-instant); falls back to an on-demand ffmpeg
        pull from the RTSP stream if the buffer is missing or stale."""
        if not self.capture_lock.acquire(blocking=False):
            print("A capture is already in progress, skipping this trigger.")
            return

        try:
            capture_time = datetime.now()
            date_folder = os.path.join(PHOTO_DIR, capture_time.strftime('%Y-%m-%d'))
            os.makedirs(date_folder, exist_ok=True)
            timestamp = capture_time.strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(date_folder, f"motion_{timestamp}.jpg")
            capture_succeeded = False

            buffer_age = None
            if os.path.exists(FRAME_BUFFER_PATH):
                buffer_age = time.time() - os.path.getmtime(FRAME_BUFFER_PATH)

            if buffer_age is not None and buffer_age <= FRAME_BUFFER_MAX_AGE:
                # Fast path: just copy the already-current frame
                shutil.copy2(FRAME_BUFFER_PATH, filename)
                print(f"Snapshot saved (buffer, age {buffer_age:.2f}s): {filename}")
                capture_succeeded = True
            else:
                # Fallback: pull a fresh frame directly (slower - pays
                # connection/keyframe latency)
                print("Frame buffer unavailable or stale, falling back to on-demand capture...")
                rtsp_url = f"rtsp://127.0.0.1:{RTSP_PORT}/{RTSP_PATH}"

                ffmpeg_cmd = [
                    'ffmpeg',
                    '-y',
                    '-rtsp_transport', 'tcp',
                    '-fflags', 'nobuffer',
                    '-flags', 'low_delay',
                    '-probesize', '32',
                    '-analyzeduration', '0',
                ] + self._decoder_input_args() + [
                    '-i', rtsp_url,
                    '-frames:v', '1',
                    '-q:v', '2',
                    filename
                ]

                result = subprocess.run(
                    ffmpeg_cmd,
                    capture_output=True,
                    timeout=SNAPSHOT_TIMEOUT
                )

                if result.returncode == 0 and os.path.exists(filename):
                    print(f"Snapshot saved: {filename}")
                    capture_succeeded = True
                else:
                    print("Snapshot capture failed.")
                    stderr = result.stderr.decode(errors='ignore')
                    if stderr:
                        print("ffmpeg output (tail):")
                        print("\n".join(stderr.strip().splitlines()[-5:]))

            if capture_succeeded:
                self.add_overlay(filename, capture_time)

        except subprocess.TimeoutExpired:
            print("Snapshot capture timed out.")
        except Exception as e:
            print(f"Error capturing snapshot: {e}")
        finally:
            self.capture_lock.release()

    def add_overlay(self, image_path, timestamp_dt):
        """Burn a semi-transparent bar with an icon, a label, and a timestamp
        into the bottom of the saved snapshot. Failures here are logged but
        never raised - a missing overlay is far preferable to losing the
        underlying capture."""
        if not OVERLAY_ENABLED:
            return

        if not PIL_AVAILABLE:
            print("WARNING: Pillow (PIL) is not installed - skipping overlay.")
            print("Install it with: sudo apt-get install python3-pil")
            return

        try:
            base = Image.open(image_path).convert("RGBA")
            width, height = base.size
            bar_top = height - OVERLAY_BAR_HEIGHT

            # Semi-transparent bar, drawn on its own layer then composited
            # (drawing directly with an RGBA fill on the base image would
            # not blend correctly since the base has no real alpha to blend
            # against once it came from a JPEG)
            overlay_layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
            bar_draw = ImageDraw.Draw(overlay_layer)
            bar_draw.rectangle(
                [0, bar_top, width, height],
                fill=(*OVERLAY_BAR_COLOR, OVERLAY_BAR_OPACITY)
            )
            composited = Image.alpha_composite(base, overlay_layer)
            draw = ImageDraw.Draw(composited)

            bar_center_y = bar_top + OVERLAY_BAR_HEIGHT / 2
            text_left_x = OVERLAY_ICON_LEFT_MARGIN

            # Icon
            if os.path.exists(OVERLAY_ICON_PATH):
                try:
                    icon = Image.open(OVERLAY_ICON_PATH).convert("RGBA")
                    if icon.size != (OVERLAY_ICON_SIZE, OVERLAY_ICON_SIZE):
                        icon = icon.resize((OVERLAY_ICON_SIZE, OVERLAY_ICON_SIZE))
                    icon_y = bar_top + (OVERLAY_BAR_HEIGHT - OVERLAY_ICON_SIZE) // 2
                    composited.paste(icon, (OVERLAY_ICON_LEFT_MARGIN, icon_y), icon)
                    text_left_x = OVERLAY_ICON_LEFT_MARGIN + OVERLAY_ICON_SIZE + OVERLAY_TEXT_GAP_AFTER_ICON
                except Exception as e:
                    print(f"WARNING: failed to load overlay icon ({OVERLAY_ICON_PATH}): {e}")
            else:
                print(f"WARNING: overlay icon not found at {OVERLAY_ICON_PATH}, skipping icon")

            # Font, sized relative to the bar height
            font_size = max(8, int(OVERLAY_BAR_HEIGHT * OVERLAY_FONT_SIZE_RATIO))
            try:
                font = ImageFont.truetype(OVERLAY_FONT_PATH, font_size)
            except Exception as e:
                print(f"WARNING: failed to load font ({OVERLAY_FONT_PATH}): {e}, using default")
                font = ImageFont.load_default()

            def vcentered_y(text, fnt):
                bbox = draw.textbbox((0, 0), text, font=fnt)
                text_h = bbox[3] - bbox[1]
                return bar_center_y - text_h / 2 - bbox[1]

            # Left-justified label
            draw.text(
                (text_left_x, vcentered_y(OVERLAY_TEXT_LEFT, font)),
                OVERLAY_TEXT_LEFT,
                font=font,
                fill=OVERLAY_TEXT_COLOR
            )

            # Right-justified timestamp
            timestamp_str = timestamp_dt.strftime(OVERLAY_TIMESTAMP_FORMAT)
            bbox_r = draw.textbbox((0, 0), timestamp_str, font=font)
            text_r_w = bbox_r[2] - bbox_r[0]
            text_r_x = width - OVERLAY_TEXT_RIGHT_MARGIN - text_r_w
            draw.text(
                (text_r_x, vcentered_y(timestamp_str, font)),
                timestamp_str,
                font=font,
                fill=OVERLAY_TEXT_COLOR
            )

            # Flatten back to RGB (JPEG has no alpha channel) and overwrite
            composited.convert("RGB").save(image_path, quality=90)

        except Exception as e:
            print(f"WARNING: failed to add overlay to {image_path}: {e}")

    # -----------------------------------------------------------------
    # Main server loop
    # -----------------------------------------------------------------
    def start_server(self):
        """Start the RTSP server"""

        print("=" * 60)
        print("Raspberry Pi Camera RTSP Server + PIR/VMD Motion Capture")
        print("=" * 60)

        try:
            pi_ip = self.get_ip_address()
            print(f"\nRaspberry Pi IP: {pi_ip}")
            print(f"Resolution: {VIDEO_WIDTH}x{VIDEO_HEIGHT} @ {VIDEO_FPS}fps")
            print(f"Bitrate: {VIDEO_BITRATE/1000000}Mbps")
            print(f"\nStream URL: rtsp://{pi_ip}:{RTSP_PORT}/{RTSP_PATH}")
            print(f"Username: {RTSP_USERNAME}")
            print(f"Password: <*********>")
            print("\nPress Ctrl+C to stop...\n")

            # Create config file
            self.create_config()

            # Check for mediamtx/rtsp-simple-server
            mediamtx_bin = self.find_rtsp_server()
            if not mediamtx_bin:
                print("\nError: MediaMTX/rtsp-simple-server not found!")
                print("\nPlease install MediaMTX:")
                print("  wget https://github.com/bluenviron/mediamtx/releases/download/v1.8.3/mediamtx_v1.8.3_linux_arm64v8.tar.gz")
                print("  tar -xzf mediamtx_v1.8.3_linux_arm64v8.tar.gz")
                print("  sudo mv mediamtx /usr/local/bin/")
                print("  sudo chmod +x /usr/local/bin/mediamtx")
                return

            print(f"Starting RTSP server using {mediamtx_bin}...")

            # Start MediaMTX
            self.mediamtx_proc = subprocess.Popen(
                [mediamtx_bin, self.config_path],
                stdout=None,
                stderr=None
            )

            # Wait for server to start
            time.sleep(3)

            # Check if server started successfully
            if self.mediamtx_proc.poll() is not None:
                print("Error: RTSP server failed to start")
                return

            print("RTSP server started successfully")

            # Start the persistent frame buffer now that the stream is available
            self.start_frame_buffer()
            time.sleep(1)  # let it grab at least one frame before we're "ready"

            # Start the VMD reader (daytime confirmation layer)
            self.start_vmd_reader()

            # Set up the PIR sensor now that the stream is available to capture from
            self.setup_gpio()

            self.running = True

            print("\n" + "=" * 60)
            print("RTSP server is running!")
            print("=" * 60)
            print(f"\nConnect from Synology to:")
            print(f"  URL: rtsp://{pi_ip}:{RTSP_PORT}/{RTSP_PATH}")
            print(f"  Username: {RTSP_USERNAME}")
            print(f"  Password: <*********>")
            if self.motion_sensor:
                print(f"\nWatching for motion on GPIO {PIR_GPIO_PIN}...")
            if VMD_ENABLED and self.vmd_functional:
                print(f"VMD daytime confirmation active ({VMD_WIDTH}x{VMD_HEIGHT} @ {VMD_FPS}fps)")
            print("\nMonitoring streams...\n")

            # Monitor MediaMTX and the frame buffer (GPIO handling runs in the
            # background via gpiozero)
            while self.running:
                time.sleep(1)

                if self.mediamtx_proc.poll() is not None:
                    print("RTSP server process terminated, restarting...")
                    time.sleep(2)
                    # Restart MediaMTX
                    self.mediamtx_proc = subprocess.Popen(
                        [mediamtx_bin, self.config_path],
                        stdout=None,
                        stderr=None
                    )
                    time.sleep(2)
                    self.start_frame_buffer()  # camera source restarted, buffer must reconnect too
                    self.start_vmd_reader()
                else:
                    self.restart_frame_buffer_if_needed()
                    self.restart_vmd_reader_if_needed()

        except KeyboardInterrupt:
            print("\n\nShutting down...")
        except FileNotFoundError as e:
            print(f"\nError: Required program not found: {e}")
            print("\nMake sure ffmpeg is installed:")
            print("  sudo apt-get install ffmpeg")
        except Exception as e:
            print(f"\nError: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")

        if self.motion_sensor:
            try:
                self.motion_sensor.close()
            except:
                pass

        self.vmd_reader_running = False
        if self.vmd_proc:
            try:
                self.vmd_proc.terminate()
                self.vmd_proc.wait(timeout=5)
            except:
                try:
                    self.vmd_proc.kill()
                except:
                    pass
        if self.vmd_thread:
            try:
                self.vmd_thread.join(timeout=3)
            except:
                pass

        if self.frame_buffer_proc:
            try:
                self.frame_buffer_proc.terminate()
                self.frame_buffer_proc.wait(timeout=5)
            except:
                try:
                    self.frame_buffer_proc.kill()
                except:
                    pass

        if self.mediamtx_proc:
            try:
                self.mediamtx_proc.terminate()
                self.mediamtx_proc.wait(timeout=5)
            except:
                try:
                    self.mediamtx_proc.kill()
                except:
                    pass

        print("Server stopped.")

    def get_ip_address(self):
        """Get the Pi's IP address"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "0.0.0.0"


def disable_wireless_radios():
    """Block wifi and bluetooth via rfkill. Requires a passwordless sudo rule
    for these exact commands (see /etc/sudoers.d/picam-rfkill setup notes).
    Exits the program if either command fails, since this is treated as a
    hard requirement."""
    if not DISABLE_WIRELESS_RADIOS:
        return

    for radio in ("wifi", "bluetooth"):
        cmd = ['sudo', RFKILL_PATH, 'block', radio]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        except FileNotFoundError:
            print(f"ERROR: rfkill not found at {RFKILL_PATH}. Run 'which rfkill' and update RFKILL_PATH.")
            sys.exit(1)
        except subprocess.TimeoutExpired:
            print(f"ERROR: 'rfkill block {radio}' timed out.")
            sys.exit(1)

        if result.returncode != 0:
            print(f"ERROR: failed to block {radio} radio.")
            print(f"Command: {' '.join(cmd)}")
            if result.stderr:
                print(f"stderr: {result.stderr.strip()}")
            print("\nMake sure a passwordless sudo rule exists, e.g. in")
            print("/etc/sudoers.d/picam-rfkill:")
            print(f"  pi ALL=(ALL) NOPASSWD: {RFKILL_PATH} block wifi, {RFKILL_PATH} block bluetooth")
            sys.exit(1)

        print(f"Blocked {radio} radio.")


def check_usb_mounted():
    """Verify USB_MOUNT_POINT is an active mount point (not just an empty
    directory on the root filesystem). Exits the program with an error if
    it isn't, since otherwise photos would silently get written to the SD
    card instead of the USB drive."""
    if not os.path.ismount(USB_MOUNT_POINT):
        print("=" * 60)
        print("ERROR: USB drive is not mounted")
        print("=" * 60)
        print(f"\nExpected an active mount at: {USB_MOUNT_POINT}")
        print("but it does not appear to be mounted.")
        print("\nCheck that the drive is connected and mounted, e.g.:")
        print(f"  lsblk")
        print(f"  sudo mount /dev/sdXN {USB_MOUNT_POINT}")
        print("\nIf it should mount automatically at boot, verify the")
        print("entry in /etc/fstab is correct.")
        print("=" * 60)
        sys.exit(1)

    print(f"Verified USB drive is mounted at {USB_MOUNT_POINT}")


def print_setup_instructions(pi_ip):
    """Print setup instructions for Synology"""
    print("\n" + "=" * 60)
    print("SYNOLOGY SURVEILLANCE STATION SETUP")
    print("=" * 60)
    print("\n1. In Synology Surveillance Station:")
    print("   - Go to IP Camera > Add > Add Camera")
    print("   - Select brand: 'User-defined' or 'Custom'")
    print("\n2. Enter camera details:")
    print(f"   - Name: Raspberry Pi Camera")
    print(f"   - IP address: {pi_ip}")
    print(f"   - Port: {RTSP_PORT}")
    print(f"   - Model: Select any RTSP-compatible model")
    print(f"   - RTSP URL: rtsp://{pi_ip}:{RTSP_PORT}/{RTSP_PATH}")
    print(f"   - Username: {RTSP_USERNAME}")
    print(f"   - Password: <********>")
    print("\n3. Click 'Test Connection' to verify")
    print("4. Complete the setup and start recording")
    print("\nNOTE: The first connection may take a few seconds")
    print("\nMOTION CAPTURE:")
    print(f"   - PIR sensor expected on GPIO {PIR_GPIO_PIN} (BCM numbering)")
    print(f"   - Snapshots saved to {PHOTO_DIR}")
    print(f"   - Minimum {CAPTURE_COOLDOWN}s between captures")
    print(f"   - Daytime PIR triggers require VMD confirmation (+/-{CORRELATION_WINDOW_SECONDS}s window)")
    print(f"   - Nighttime PIR triggers are trusted alone")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    # Block wifi/bluetooth before anything else starts
    disable_wireless_radios()

    # Fail fast if the USB drive isn't mounted, before starting the RTSP
    # server or camera at all
    check_usb_mounted()

    server = RTSPServer()

    # Display setup instructions
    pi_ip = server.get_ip_address()
    print_setup_instructions(pi_ip)

    # Only wait for a keypress when running interactively (e.g. from a
    # terminal). Under systemd (or any non-TTY context) there's no one to
    # press Enter, so skip straight to starting the server.
    if sys.stdin.isatty():
        input("Press Enter to start the RTSP server...")
    else:
        print("Non-interactive session detected (e.g. running under systemd) - starting immediately.")

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        server.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    server.start_server()
