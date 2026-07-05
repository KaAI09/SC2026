from pathlib import Path

from monitor.flask_app_factory import create_app
from monitor.monitor_state import MonitorState


def read_monitor_static_asset(*path_parts):
    package_root = Path(create_app.__code__.co_filename).resolve().parent
    candidates = [package_root / 'static' / Path(*path_parts)]
    for parent in Path(__file__).resolve().parents:
        candidates.append(
            parent / 'src' / 'monitor' / 'monitor' / 'static' / Path(*path_parts)
        )

    for candidate in candidates:
        if candidate.exists():
            return candidate.read_text(encoding='utf-8')

    raise FileNotFoundError(Path('static') / Path(*path_parts))


def make_test_app():
    resource_path = Path(__file__).resolve()
    return create_app(
        MonitorState(3.0, 160, 120),
        'Test Monitor',
        'battery_status',
        '/camera/image/compressed',
        '/',
        1000,
        100,
        resource_path,
        resource_path,
        160,
        120,
    )


def test_status_snapshot_reports_only_image_battery_storage():
    snapshot = MonitorState(3.0, 160, 120).snapshot()

    assert set(snapshot.keys()) == {'battery', 'image', 'storage'}
    # Removed panels must not leak back into the payload.
    assert 'control' not in snapshot
    assert 'recording' not in snapshot
    assert 'debug_image' not in snapshot


def test_status_endpoint_returns_three_sections():
    app = make_test_app()

    with app.test_request_context('/api/status'):
        payload = app.view_functions['api_status']().get_json()

    assert set(payload.keys()) == {'battery', 'image', 'storage'}


def test_frame_endpoint_serves_placeholder_when_no_frame():
    app = make_test_app()

    with app.test_request_context('/api/frame'):
        response = app.view_functions['api_frame']()

    assert response.mimetype == 'image/svg+xml'


def test_graph_endpoint_is_removed():
    app = make_test_app()

    assert 'api_graph' not in app.view_functions
    assert 'api_frame_grayscale' not in app.view_functions


def test_dashboard_has_no_control_graph_or_debug_assets():
    app = make_test_app()

    with app.test_request_context('/'):
        template = app.view_functions['index']()

    script = read_monitor_static_asset('js', 'app.js')

    assert 'image-card' in template
    assert 'battery-card' in template
    assert 'storage-card' in template
    assert 'control-card' not in template
    assert 'graph-card' not in template
    assert 'record-badge' not in template
    assert 'debug-grid' not in template
    assert 'fetchGraph' not in script
    assert 'renderControl' not in script
