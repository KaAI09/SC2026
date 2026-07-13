const BATTERY_THRESHOLDS = {
  good: 70,
  caution: 35,
};

const STORAGE_THRESHOLDS = {
  good: 70,
  caution: 85,
};

const CARD_CLASSES = [
  'is-good',
  'is-live',
  'is-caution',
  'is-low',
  'is-waiting',
  'is-stale',
  'is-offline',
];

const config = window.MONITOR_CONFIG || {
  statusEndpoint: '/api/status',
  frameEndpoint: '/api/frame',
  placeholderUrl: '/api/frame/placeholder',
  refreshIntervalMs: 1000,
  imageRefreshIntervalMs: 150,
};

const elements = {
  batteryCard: document.getElementById('battery-card'),
  batteryChip: document.getElementById('battery-chip'),
  batteryValue: document.getElementById('battery-value'),
  batteryUpdated: document.getElementById('battery-updated'),
  batteryMeterFill: document.getElementById('battery-meter-fill'),
  imageCard: document.getElementById('image-card'),
  imageChip: document.getElementById('image-chip'),
  imageUpdated: document.getElementById('image-updated'),
  imageResolution: document.getElementById('image-resolution'),
  cameraFrame: document.getElementById('camera-frame'),
  storageCard: document.getElementById('storage-card'),
  storageChip: document.getElementById('storage-chip'),
  storageValue: document.getElementById('storage-value'),
  storageUpdated: document.getElementById('storage-updated'),
  storageMeterFill: document.getElementById('storage-meter-fill'),
  storageDetail: document.getElementById('storage-detail'),
};

let imageRequestInFlight = false;

function clampPercent(value) {
  return Math.max(0, Math.min(100, Number(value) || 0));
}

function formatUpdatedAt(updatedAt) {
  if (!updatedAt) {
    return 'Waiting for message';
  }

  const date = new Date(updatedAt);

  if (Number.isNaN(date.getTime())) {
    return 'Invalid timestamp';
  }

  return date.toLocaleString();
}

function setCardState(cardElement, state) {
  cardElement.classList.remove(...CARD_CLASSES);
  cardElement.classList.add(`is-${state}`);
}

function getBatteryState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  const battery = clampPercent(data.battery_status);

  if (battery >= BATTERY_THRESHOLDS.good) {
    return 'good';
  }

  if (battery >= BATTERY_THRESHOLDS.caution) {
    return 'caution';
  }

  return 'low';
}

function getBatteryLabel(state) {
  switch (state) {
    case 'good':
      return 'GOOD';
    case 'caution':
      return 'CAUTION';
    case 'low':
      return 'LOW';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function getImageState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  return 'live';
}

function getImageLabel(state) {
  switch (state) {
    case 'live':
      return 'LIVE';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function getStorageState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  const storageUsed = clampPercent(data.used_percentage);

  if (storageUsed >= STORAGE_THRESHOLDS.caution) {
    return 'low';
  }

  if (storageUsed >= STORAGE_THRESHOLDS.good) {
    return 'caution';
  }

  return 'good';
}

function getStorageLabel(state) {
  switch (state) {
    case 'good':
      return 'GOOD';
    case 'caution':
      return 'CAUTION';
    case 'low':
      return 'HIGH';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function renderBattery(data) {
  const state = getBatteryState(data);
  const batteryPercent = data.has_data ? clampPercent(data.battery_status) : 0;

  setCardState(elements.batteryCard, state);
  elements.batteryChip.textContent = getBatteryLabel(state);
  elements.batteryValue.textContent = data.battery_display || '--.-%';
  elements.batteryUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.batteryMeterFill.style.width = `${batteryPercent}%`;
}

function renderImage(data) {
  const state = getImageState(data);

  setCardState(elements.imageCard, state);
  elements.imageChip.textContent = getImageLabel(state);
  elements.imageUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.imageResolution.textContent = data.resolution_display || 'Waiting for frame';
}

function renderStorage(data) {
  const state = getStorageState(data);
  const usedPercent = data.has_data ? clampPercent(data.used_percentage) : 0;

  setCardState(elements.storageCard, state);
  elements.storageChip.textContent = getStorageLabel(state);
  elements.storageValue.textContent = data.used_display || '--.-%';
  elements.storageUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.storageMeterFill.style.width = `${usedPercent}%`;
  elements.storageDetail.textContent = `${data.used_space_display || '--'} / ${data.total_space_display || '--'} used`;
}

function renderOffline() {
  setCardState(elements.batteryCard, 'offline');
  setCardState(elements.imageCard, 'offline');
  setCardState(elements.storageCard, 'offline');
  elements.batteryChip.textContent = 'OFFLINE';
  elements.imageChip.textContent = 'OFFLINE';
  elements.storageChip.textContent = 'OFFLINE';
  elements.batteryUpdated.textContent = 'Unable to reach monitor server';
  elements.imageUpdated.textContent = 'Unable to reach monitor server';
  elements.storageUpdated.textContent = 'Unable to reach monitor server';
}

async function fetchStatus() {
  try {
    const response = await fetch(config.statusEndpoint, { cache: 'no-store' });

    if (!response.ok) {
      throw new Error(`Unexpected response: ${response.status}`);
    }

    const payload = await response.json();
    renderBattery(payload.battery || {});
    renderImage(payload.image || {});
    renderStorage(payload.storage || {});
  } catch (error) {
    console.error('Failed to fetch monitor status', error);
    renderOffline();
  }
}

function refreshCameraFrame() {
  if (imageRequestInFlight) {
    return;
  }

  imageRequestInFlight = true;

  const image = new Image();
  image.onload = () => {
    elements.cameraFrame.src = image.src;
    imageRequestInFlight = false;
  };
  image.onerror = () => {
    elements.cameraFrame.src = config.placeholderUrl;
    imageRequestInFlight = false;
  };
  image.src = `${config.frameEndpoint}?t=${Date.now()}`;
}

function startPolling() {
  fetchStatus();
  // Low-latency MJPEG stream: feed the <img> directly; the browser renders each
  // pushed frame. Falls back to the placeholder if the stream cannot start.
  const cam = elements.cameraFrame;
  const placeholder = config.placeholderUrl || '/api/frame/placeholder';
  cam.onerror = () => { cam.onerror = null; cam.src = placeholder; };
  cam.src = config.streamEndpoint || '/api/stream';
  window.setInterval(fetchStatus, config.refreshIntervalMs);
}

document.addEventListener('DOMContentLoaded', startPolling);
