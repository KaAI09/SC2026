from datetime import datetime, timezone
import threading
import time


class MonitorState:
    """Thread-safe snapshot of the three monitored signals: camera image,
    battery status, and storage usage."""

    def __init__(self, stale_timeout_sec, image_source_width, image_source_height):
        self._lock = threading.Lock()
        self._stale_timeout_sec = stale_timeout_sec

        self._battery_status = None
        self._battery_updated_at = None
        self._battery_updated_monotonic = None

        self._image_frame = None
        self._image_width = image_source_width
        self._image_height = image_source_height
        self._image_updated_at = None
        self._image_updated_monotonic = None

        self._storage_used_percentage = None
        self._storage_used_bytes = None
        self._storage_total_bytes = None
        self._storage_updated_at = None
        self._storage_updated_monotonic = None

    def _is_stale(self, updated_monotonic):
        if updated_monotonic is None:
            return True

        return (time.monotonic() - updated_monotonic) > self._stale_timeout_sec

    def _format_gb(self, size_bytes):
        if size_bytes is None:
            return '--'

        return f'{size_bytes / (1024 ** 3):.1f} GB'

    def update_battery(self, battery_status):
        clamped_value = max(0.0, min(100.0, float(battery_status)))

        with self._lock:
            self._battery_status = clamped_value
            self._battery_updated_at = datetime.now(timezone.utc)
            self._battery_updated_monotonic = time.monotonic()

    def update_image(self, frame_bytes, source_width, source_height):
        with self._lock:
            self._image_frame = frame_bytes
            self._image_width = int(source_width)
            self._image_height = int(source_height)
            self._image_updated_at = datetime.now(timezone.utc)
            self._image_updated_monotonic = time.monotonic()

    def update_storage(self, used_bytes, total_bytes):
        if total_bytes <= 0:
            return

        used_percentage = (float(used_bytes) / float(total_bytes)) * 100.0

        with self._lock:
            self._storage_used_percentage = max(0.0, min(100.0, used_percentage))
            self._storage_used_bytes = int(used_bytes)
            self._storage_total_bytes = int(total_bytes)
            self._storage_updated_at = datetime.now(timezone.utc)
            self._storage_updated_monotonic = time.monotonic()

    def get_latest_frame(self):
        with self._lock:
            return self._image_frame

    def snapshot(self):
        with self._lock:
            battery_status = self._battery_status
            battery_updated_at = self._battery_updated_at
            battery_updated_monotonic = self._battery_updated_monotonic

            image_width = self._image_width
            image_height = self._image_height
            image_updated_at = self._image_updated_at
            image_updated_monotonic = self._image_updated_monotonic

            storage_used_percentage = self._storage_used_percentage
            storage_used_bytes = self._storage_used_bytes
            storage_total_bytes = self._storage_total_bytes
            storage_updated_at = self._storage_updated_at
            storage_updated_monotonic = self._storage_updated_monotonic

        battery_has_data = battery_status is not None
        image_has_data = image_updated_at is not None
        storage_has_data = (
            storage_used_percentage is not None
            and storage_used_bytes is not None
            and storage_total_bytes is not None
        )

        return {
            'battery': {
                'has_data': battery_has_data,
                'battery_status': None if battery_status is None else round(battery_status, 1),
                'battery_display': '--.-%' if battery_status is None else f'{battery_status:.1f}%',
                'updated_at': None if battery_updated_at is None else battery_updated_at.isoformat(),
                'is_stale': self._is_stale(battery_updated_monotonic),
            },
            'image': {
                'has_data': image_has_data,
                'updated_at': None if image_updated_at is None else image_updated_at.isoformat(),
                'is_stale': self._is_stale(image_updated_monotonic),
                'resolution_display': f'{image_width}x{image_height}',
            },
            'storage': {
                'has_data': storage_has_data,
                'updated_at': None if storage_updated_at is None else storage_updated_at.isoformat(),
                'is_stale': self._is_stale(storage_updated_monotonic),
                'used_percentage': (
                    None if storage_used_percentage is None else round(storage_used_percentage, 1)
                ),
                'used_display': (
                    '--.-%' if storage_used_percentage is None
                    else f'{storage_used_percentage:.1f}%'
                ),
                'used_space_display': self._format_gb(storage_used_bytes),
                'total_space_display': self._format_gb(storage_total_bytes),
            },
        }
