# Security and physical safety

This repository controls real actuators. Read [hardware safety](docs/safety.md)
before connecting a controller. CI and offline tests do not authorize motion or
establish a safe physical installation. ROS 2 topics and services are not an
authentication boundary; use an isolated, trusted robot network.

## Reporting a problem

For a public report, provide the software revision, ROS version, a minimal
offline reproduction and a redacted log. Remove camera serials, local paths,
network addresses, calibration captures, credentials and images of people or
private spaces. Never attach a complete home directory, rosbag or serial dump
without reviewing its contents.

The project repository is
[ros2-visual-manipulator](https://github.com/chao142857/ros2-visual-manipulator). A private security
reporting channel has not yet been configured. Use GitHub's private vulnerability
reporting if it is enabled on that repository. Otherwise first
request a private contact channel without including exploit details or secrets.
The confirmed public maintainer contact is recorded in
[RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md); a dedicated security channel is still pending.

If a credential was published, revoke it at its issuer before cleaning history.
Deleting a file in the latest commit does not remove earlier copies.

## Local publication checks

```bash
python3 tools/release/audit.py . --history --json /tmp/arm-source-audit.json
python3 tools/release/audit.py ../arm_ws-publication-20260920 --strict-public
```

The scanner reports locations and categories without printing matched values.
It scans reachable Git objects only when `--history` is supplied. It uses
patterns, not a complete credential database; binary files and archives receive
metadata inspection, not content decoding. Human review remains necessary.

Publish only the reviewed export described in [PUBLICATION_AUDIT.md](PUBLICATION_AUDIT.md).
The hardware workspace intentionally retains local assets, calibration and old
history. `.gitignore` does not remove tracked files or historical objects.
