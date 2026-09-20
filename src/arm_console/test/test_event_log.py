from arm_console.event_log import EventLogWriter, ros_log_directory


def test_ros_log_directory_honors_ros_log_dir(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    monkeypatch.setenv('ROS_HOME', '/should/not/be/used')

    assert ros_log_directory() == tmp_path


def test_event_log_is_rotated(tmp_path):
    writer = EventLogWriter(tmp_path, max_bytes=100, backup_count=2)
    assert writer.available

    for index in range(20):
        assert writer.write('info', f'event {index}: ' + ('x' * 30)) is None
    writer.close()

    assert (tmp_path / 'arm_console_events.log').exists()
    assert (tmp_path / 'arm_console_events.log.1').exists()


def test_event_log_open_failure_is_non_fatal(tmp_path):
    not_a_directory = tmp_path / 'file'
    not_a_directory.write_text('occupied', encoding='utf-8')

    writer = EventLogWriter(not_a_directory)

    assert not writer.available
    assert writer.error
    assert writer.write('warning', 'still safe to show in the UI') is None


def test_event_log_rotation_failure_degrades_to_no_op(monkeypatch, tmp_path):
    writer = EventLogWriter(tmp_path, max_bytes=1, backup_count=1)

    def fail_rotation():
        raise OSError('simulated rotation failure')

    monkeypatch.setattr(writer._handler, 'doRollover', fail_rotation)

    assert writer.write('error', 'must not escape into the Qt event loop') is None
    assert not writer.available
    assert 'simulated rotation failure' in writer.error
