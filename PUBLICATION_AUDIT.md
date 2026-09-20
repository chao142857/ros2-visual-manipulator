# GitHub publication audit

This is the 2026-09-08 decision/evidence snapshot. Current Phase 2 defaults,
validation and release status are in [RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md)
and the [2026-09-20 audit addendum](docs/audit_2026-09-20.md#phase-2-2026-09-21).
The historical numbers below are preserved.

Review date: 2026-09-08. This review distinguishes the private hardware workspace
from the exported software-only publication candidate. The original workspace
must not be pushed wholesale to a public repository.

## Decisions and current limits

- The maintainer authorized Apache-2.0 for their original software.
- The maintainer's original hardware contribution is the adapter/mount only,
  not the arm body, gripper, motors or cameras. No separately identifiable
  adapter file set or hardware license is approved; all model assets stay excluded.
- Third-party notices remain in the source. CAD/STL, CAD-derived metadata,
  unresolved robot descriptions and SRDF, original CAD awaiting a license,
  site calibration and captures are excluded from publication.
- The verified public GitHub identity is `bbabbr123-sketch` (display name
  `Chao142857`). The public maintainer email was explicitly confirmed on
  2026-09-21 and is recorded in [RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md).
  The repository URL was also explicitly supplied on 2026-09-21:
  [https://github.com/chao142857/ros2-visual-manipulator](https://github.com/chao142857/ros2-visual-manipulator).
- The local driver mappings, directions, joint limits, zero offsets, speed,
  acceleration, PWM bounds and emergency behavior are not changed by export.
  The existing TCP lookup table embedded in `gripper_driver/gripper_geometry.py`
  is retained as part of that software behavior. It is historical measurement
  data for the author's hardware; another installation must verify it physically.
- Real camera identity defaults are replaced **only in exported copies** with
  nonmatching one-digit sentinels. The export records every changed line in
  `PUBLICATION_TRANSFORMATIONS.json`. Supply an explicitly selected real serial
  before camera use. Unpinned auto-selection is not substituted silently.
- The gripper's unique USB by-id path is also replaced only in exported copies
  with `/dev/serial/by-id/CONFIGURE_GRIPPER_PORT`. Configure the actual controller
  port explicitly. The scanner examines adjacent Python string literals so a
  multiline device identity cannot bypass the release gate.

This candidate supports reading the software, installing its dependencies and
running the documented subset of offline checks. It does not include a legally
cleared complete robot model or transferable calibration, and is not a complete
hardware reproduction kit.

## Scope and evidence

The initial inventory contained 70 tracked files, substantial pre-existing local
edits, and untracked perception/grasp packages. Git had two commits on `main`
and no configured remote. Both commits and every reachable object were reviewed
by the release scanner: 74 blob objects, 14 of them binary model files. Commit
identities and source metadata contained personal author information.

The first current-tree scan covered 597 non-build files, including ignored local
material, archives and captures. It found local absolute paths, personal email
  addresses, real camera serials and ASIC identifiers. It did not detect a private
key, recognized access token, literal password/API credential, or private IPv4
address under its patterns. This is a bounded pattern-scan result, not proof of
the absence of all secrets. Archives and binary payloads were inventoried but not
decoded, so they are excluded from the public candidate.

A supplementary AST scan identified four unique-device USB path bindings in
the gripper node and composing launch files; these are transformed in the
candidate and included in the per-location transformation report.

Examples requiring exclusion or explicit review are recorded by file/category,
without reproducing private values:

| Location in the private workspace | Category / action |
| --- | --- |
| Package manifests and `setup.py` | Public identity and maintainer-approved public contact; final values tracked in RELEASE_CHECKLIST.md |
| Wrist/dual-camera/tabletop launch defaults and wrist tests | Device serials replaced in export copies only |
| Gripper node and three composing launches | Unique USB by-id controller path replaced in export copies only |
| Existing wrist README and Chinese deployment guide | Device and ASIC identifiers, old firmware/location observations; excluded |
| Existing tabletop README and operation/CAD documents | Home paths and private field evidence; excluded |
| `tabletop_grasp/config/cad_measurements.json`, `cad_collision_proxy.json` | Source CAD paths and unlicensed derived geometry; excluded |
| `camera_perception/config/camera_extrinsics.yaml` | Real fixed-camera calibration; excluded |
| Wrist `d405_*` calibration YAML | Site-specific hand-eye and device internal TF; excluded |
| `tools/windows`, `tools/handoff`, historical root notes | Local workflow/history information; excluded |
| `artifacts`, recorded bags, source ZIP and restore directory | Unreviewed media, diagnostics and duplicate historical content; excluded |

## Cleanup performed

Exactly 47 AppleDouble `._*` metadata files (192,512 bytes) were confirmed by
their binary format signature, backed up and then removed. Their contents are
filesystem metadata, not ROS source or hardware functionality. The backup was
verified by reading every archived member and comparing SHA-256 hashes before
removal. The private sibling backup directory contains
`appledouble-20260908.tar.gz` and its per-file/hash manifest.

No hardware code, old robot model, rosbag, calibration result, source ZIP or
field evidence was deleted. Ignore rules now cover local data and generated
output. Ignore rules do not untrack files or sanitize existing history.

## Five publication file classes

1. **Reviewed original-software candidates.** The exact `files` list in
   `tools/release/publication_manifest.json` is the release boundary. It includes
   the Python implementation/launch/test and ROS metadata of `arm_driver`,
   `gripper_driver`, `arm_console`, `camera_perception`, `wrist_camera_bringup`,
   `dual_camera_foundation`, `tabletop_grasp`, and `tabletop_grasp_interfaces`;
   the source-only launch/metadata of `arm_description`; explicitly selected
   software configurations; and the new root documentation, scripts and CI.
   The maintainer's declaration authorizes their original software. The list
   does not prove authorship of an unidentified third-party component. Known
   upstream templates are classified separately below. The JSON file supplies
   every individual path; directory names here do not authorize extra files.
2. **Identified third-party material retained.** The twelve files enumerated in
   `THIRD_PARTY_NOTICES.md` are ROS test templates under Apache-2.0 with their
   original notices. The explicitly identified librealsense v2.56.2 inverse-Brown
   adaptation in `tabletop_grasp/rgbd.py` retains Intel's Apache-2.0 attribution
   in that notice file. ROS/MoveIt/RealSense/OpenCV and other external dependencies
   are installed separately and retain their own licenses. No vendor model
   is approved merely because a package declares Apache-2.0.
3. **Unknown-source or unlicensed design material excluded.** Exact local sets:
   `src/arm_description/meshes/*.stl`,
   `src/arm_description/meshes/dummy_v2/*.stl`,
   `src/arm_description/urdf/*`,
   `src/arm_description/tools/generate_dummy_v2_meshes.py`,
   `src/tabletop_grasp/meshes/**`,
   `src/tabletop_grasp/config/cad_*.json`,
   `src/tabletop_grasp/config/gripper_motion*.json`, and
   `src/tabletop_grasp/config/dummy_v2.srdf`.
   The CAD processing scripts under `src/tabletop_grasp/scripts/cad_*` and the
   model-preview builder/template are also conservatively omitted from this
   software candidate; omission does not assert that those scripts infringe.
4. **Local material that must not be uploaded.** `.git/**`, build/install/log and
   virtual-environment trees, `.chao_restore_backup_*/`, `artifacts/**`,
   `d405_id3_teach_bag*/`, `src.zip`, old root field/workflow notes,
   `tools/handoff/**`, `tools/windows/**`, `tools/wsl/**`, all pre-existing
   package README/docs, `src/camera_perception/config/camera_extrinsics.yaml`,
   `src/wrist_camera_bringup/config/d405_*.yaml`,
   `src/tabletop_grasp/config/tabletop.yaml`, and
   `src/tabletop_grasp/config/camera_white_cuboid_roi.yaml` are outside the
   allowlist. Four package READMEs needed by install rules are generated as
   concise public descriptions; their private historical contents are not copied.
5. **Privacy-risk locations requiring review.** The location/category table
   above covers author identity, home paths, camera/ASIC serials, controller USB
   paths, calibration and scene records. The redacted scanner report supplies
   exact lines. No private value is placed in the public report or transformation
   log. Any newly found personal data blocks export until explicitly resolved.

## Reproduce the candidate

The exact reviewed source list is
[`tools/release/publication_manifest.json`](tools/release/publication_manifest.json).
It contains individual file paths, not directory globs. New files are excluded
until the list is deliberately reviewed and updated. A listed missing file,
symlink, binary/oversized input, prohibited model/archive, or remaining private
data makes export fail. The candidate has no `.git` directory.
Files being edited by the parallel perception task also have explicitly reviewed
source hashes in the allowlist; a later revision requires review before export.

From the private workspace root:

```bash
python3 tools/release/audit.py . --history --json /tmp/arm-private-audit.json
python3 tools/release/export_publication.py ../arm_ws-publication
python3 tools/release/audit.py ../arm_ws-publication --strict-public \
  --json /tmp/arm-public-audit.json
```

To regenerate an already generated candidate, add `--refresh`. It refuses an
ordinary directory or a directory containing `.git`, and retains the previous
candidate in a dated sibling directory before replacing it. The source workspace
is never reset, cleaned, checked out, or overwritten.

`PUBLICATION_MANIFEST.json` inventories exported files with SHA-256; generated
reports are not self-hashed. It also records each source input's SHA-256 without
its private contents. Every source input and the allowlist are read again after
writing the staging tree and after its audit; concurrent edits cause a closed
failure instead of replacing the candidate with a mixed snapshot.
`PUBLICATION_TRANSFORMATIONS.json` records the
path, original source line and replacement category for each change without recording the
original identity. Explicit README placeholders describe excluded model folders;
they are not substitute URDFs or meshes.

The new inverse-Brown regression test has one additional exact-source-hash
transformation: recorded device intrinsics are replaced with explicit synthetic
test intrinsics, and the final test of two excluded private scene profiles plus
its unused imports is omitted. Mathematical/SDK comparison tests remain. The
original test is retained unchanged in the hardware workspace, and all four
copy-only edits are recorded. This is not a replacement physical calibration.

## History and eventual publication

Keep the current Git history private. Start a separate public history only from
the reviewed candidate after reviewing its notices, limitations and manifest.
Do not add a public remote to the hardware workspace and push `--all`, tags or
an archive of that workspace. No history rewriting, Git initialization, public
remote creation or upload was performed by this preparation task.

Before an actual public release, rerun the candidate scan and its documented
offline CI against the final exported files. Record the source change status and
validation results in [docs/RELEASE_READINESS.md](docs/RELEASE_READINESS.md).
Model rights, independent calibration validation, controller firmware/protocol
reproducibility and supervised hardware acceptance remain release limitations.
