# Third-party material and excluded assets

The maintainer authorizes Apache-2.0 for their original software. See
[LICENSE](LICENSE). This statement does not relicense third-party material,
hardware designs, CAD or meshes.

The maintainer clarified that their original hardware contribution is the
adapter/mount only. This does not claim authorship of the arm body, gripper,
motors or cameras. The adapter cannot yet be identified as a separately reviewed
file set, and its hardware license has not been selected. All model/design
assets therefore remain excluded.

## Notices retained in source

ROS package test templates carry Open Source Robotics Foundation copyright and
Apache-2.0 notices. Those notices are retained. Any future imported code must
retain its applicable license and attribution.

The following nine unchanged generated templates keep their Open Source Robotics
Foundation copyright (2015 or 2017) and Apache-2.0 notice:

- `src/arm_driver/test/test_copyright.py`
- `src/arm_driver/test/test_flake8.py`
- `src/arm_driver/test/test_pep257.py`
- `src/gripper_driver/test/test_copyright.py`
- `src/gripper_driver/test/test_flake8.py`
- `src/gripper_driver/test/test_pep257.py`
- `src/camera_perception/test/test_copyright.py`
- `src/camera_perception/test/test_flake8.py`
- `src/camera_perception/test/test_pep257.py`

These files match the corresponding `ament_python` templates from
[ros2/ros2cli, tag 0.18.18](https://github.com/ros2/ros2cli/tree/0.18.18/ros2pkg/ros2pkg/resource/ament_python)
after EmPy decorator-marker expansion.

Three additional files are adaptations of those same templates under Apache-2.0:

- `src/wrist_camera_bringup/test/test_copyright.py`:
  [upstream template](https://github.com/ros2/ros2cli/blob/0.18.18/ros2pkg/ros2pkg/resource/ament_python/test_copyright.py.em),
  Copyright 2015 Open Source Robotics Foundation, Inc.
- `src/wrist_camera_bringup/test/test_flake8.py`:
  [upstream template](https://github.com/ros2/ros2cli/blob/0.18.18/ros2pkg/ros2pkg/resource/ament_python/test_flake8.py.em),
  Copyright 2017 Open Source Robotics Foundation, Inc.
- `src/wrist_camera_bringup/test/test_pep257.py`:
  [upstream template](https://github.com/ros2/ros2cli/blob/0.18.18/ros2pkg/ros2pkg/resource/ament_python/test_pep257.py.em),
  Copyright 2015 Open Source Robotics Foundation, Inc.

Their upstream copyright and Apache-2.0 notices are retained alongside
`Copyright 2026 chao (project modifications)`. That additional line credits the
project adaptations; it does not replace the upstream attribution. Existing
adaptations add module docstrings, format the flake8 assertion without changing
its expression, and remove the generated copyright test's default skip so that
this project's copyright test remains enabled. The attribution correction
changes comments only; test bodies, markers and existing skip behavior are
unchanged.

The three tagged upstream templates were verified byte-for-byte against the
installed ROS 2 Humble `ros2pkg 0.18.18` templates before EmPy marker expansion.
All twelve template files are covered by the complete Apache-2.0 text in the
root [LICENSE](LICENSE).

For other software in the export, Apache-2.0 coverage follows the maintainer's
statement about their original work. This review does not infer ownership of an
unidentified third-party contribution from the absence of a copyright header.

The project depends on ROS 2 Humble, MoveIt 2, RealSense ROS/librealsense,
OpenCV, NumPy, PyYAML, SciPy, PyQt5 and other packages declared in package
manifests and installation documentation. They are installed separately and are
subject to their own licenses. They are not relicensed by this repository.
The controller firmware (not bundled) and vendor camera/CAD materials are not
bundled as an implied open-source dependency.

## RealSense inverse-Brown deprojection adaptation

`src/tabletop_grasp/tabletop_grasp/rgbd.py::_inverse_brown_rays` identifies itself
as a vectorized adaptation of the inverse-Brown branch of
`rs2_deproject_pixel_to_point` in librealsense **v2.56.2**, `src/rs.cpp`.
The checked local SDK source carries this notice:

> License: Apache 2.0 See LICENSE file in root directory.
> Copyright(c) 2024 Intel Corporation. All Rights Reserved.

The adaptation changes the C++ scalar routine into Python/NumPy array operations
and caches calibration rays. It retains the ten-iteration deprojection algorithm.
It is third-party-derived software under Apache-2.0, not an original hardware
design or an independent physical calibration result.

Upstream references: [source](https://github.com/IntelRealSense/librealsense/blob/v2.56.2/src/rs.cpp),
[license](https://github.com/IntelRealSense/librealsense/blob/v2.56.2/LICENSE),
[SDK third-party notices](https://github.com/IntelRealSense/librealsense/blob/v2.56.2/NOTICE.md).
Other components listed in the SDK notice are not copied into this repository
by that mathematical adaptation; the separately installed SDK retains its full
upstream notices.
Verbatim copies of the tagged upstream [LICENSE](third_party/librealsense/LICENSE)
and [NOTICE](third_party/librealsense/NOTICE) are included. Their source commit
and SHA-256 checksums are recorded in [the provenance file](third_party/librealsense/README.md).
Copyright/contact emails in those notices are preserved under an exact-file-hash
exception to the publication privacy scan; this exception does not suppress
credential findings.

## Model and design status

| Local material | Evidence available | Publication decision |
| --- | --- | --- |
| `arm_description/meshes/dummy_v2` | Local reference archive and converter provenance; no verified upstream distribution grant | Excluded |
| Original arm link meshes | Existing binary STL files; upstream author/license unverified | Excluded |
| `arm_description/urdf` | CAD-derived geometry; source rights unresolved | Excluded |
| Tabletop gripper meshes and CAD geometry JSON | Derived from a local gripper assembly; source rights unresolved | Excluded |
| Model-specific SRDF | Derived from the excluded robot description | Excluded |
| Adapter/mount CAD claimed as original | Exact file set not established; no hardware license selected | Excluded |
| Camera captures, bags and screenshots | Local hardware/scene records; publication review incomplete | Excluded |

Generating a new STL or simplifying a mesh does not establish a redistribution
right. Excluded files remain in the private hardware workspace. They must not be
added to a public release until their provenance and license are documented.

The software-only candidate therefore lacks the complete robot model and
site-specific calibration. Model-dependent RViz, MoveIt, grasp and geometry tests
cannot be reproduced from this candidate alone. See [models/README.md](models/README.md).

## Before adding assets

Record the source URL or original author, upstream revision, exact license,
modifications, redistribution permission and required attribution. Keep proof
with the release review. For original CAD, the maintainer must choose its license
separately. Do not replace missing designs with guessed robot dimensions.

## Phase 2 license review (2026-09-21)

The existing root LICENSE is the complete Apache-2.0 text; it was not replaced.
All nine exported package manifests declare Apache-2.0. For `arm_description`,
the declaration covers only the seven retained software/metadata/documentation
files: `CMakeLists.txt`, `package.xml`, `launch/robot_state.launch.py`,
`launch/view_arm.launch.py`, `rviz/arm.rviz`, `urdf/README.md` and `meshes/README.md`.
The ordinary CMake scaffold follows the Apache-2.0 ROS 2 `ros2pkg` generator;
project additions follow the root original-software declaration. See
[ros2pkg CMake template, 0.18.18](https://github.com/ros2/ros2cli/blob/0.18.18/ros2pkg/ros2pkg/resource/ament_cmake/CMakeLists.txt.em).
The exporter pins and changes only the copied package license/description;
this does not license the excluded local models, CAD or converter. No vendor SDK or controller firmware is bundled. The
serial protocol implementation is present, but a protocol string or missing
copyright header does not prove authorship of copied code: retain any upstream
attribution and confirm provenance before adding vendor material. No positive
evidence of an additional copied controller SDK was found in the inspected source.

| Software license | Practical obligations | Relevant distinction |
| --- | --- | --- |
| MIT | Retain copyright and permission notice | Short permissive license; no express Apache-style contributor patent grant |
| Apache-2.0 (existing) | Retain license/notices, identify modifications, preserve applicable NOTICE | Express contribution patent grant and patent-litigation termination; no trademark grant |
| BSD-3-Clause | Retain notices/disclaimer in source and binary documentation | Explicit no-endorsement clause; no express Apache-style patent grant |

All permit commercial/proprietary use and disclaim warranties; none clears
unknown CAD, meshes, calibration media or firmware. There is no reason to change
the existing software choice for this release. Original adapter CAD still needs
an exact file set and a separate hardware-license decision. Newly located chassis
CAD and simulation assets remain outside the export and require their own review.

Separately installed dependencies keep their licenses. In particular, a future
bundled GUI/container/binary distribution must review PyQt5/Qt GPL or commercial
terms and other redistributed components; the root Apache notice alone does not
cover that bundle. This candidate distributes project source and dependency
instructions, not those vendor binaries.

## Test-data exclusions (2026-09-21)

Internal CAD-derived geometry validation is excluded from the public software release.
The allowlist excludes `src/tabletop_grasp/test/test_gripper_motion.py`,
`src/tabletop_grasp/test/test_cad_config.py`, and the model/scene-specific
`src/dual_camera_foundation/test/test_locked_j6_ik.py`. The last file contains
a target/joint snapshot whose provenance is unconfirmed. Original private files
are retained unchanged; no substitute measurements are published. Other checks
requiring excluded CAD, SRDF or site calibration explicitly skip in the public
candidate. This removes validation coverage, not a runtime safety gate.
