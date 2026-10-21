# C++ ICP over Zenoh

`icp_zenoh` uses the official **zenoh-cpp 1.10.1** API and the existing vanilla
3D ICP implementation. It subscribes to complete raw XYZI scans and publishes a
JSON result containing the estimated rotation and translation, residuals, point
counts, iteration count, termination reason, and registration duration.

## Build and start

Install the dependencies in [INSTALL.md](../INSTALL.md#optional-c-zenoh-node), then
run from the repository root:

```sh
cmake -S . -B build/zenoh -DCEV_ICP_BUILD_TOOLS=OFF \
  -DCEV_ICP_BUILD_ZENOH=ON -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HOME/.local"
cmake --build build/zenoh --parallel
./build/zenoh/icp_zenoh --listen tcp/127.0.0.1:7447
```

Use `--help` to print usage without opening transport. The startup line names the
input/output keys and frame label. Registrations are published on the output key;
subscribe to that key to inspect JSON. The node does not print every result.

The first valid scan initializes the reference without publishing. Send at least
two valid scans to obtain a registration. Start your result subscriber before
sending data: the application provides a live stream without retained history.

## Connect sensors and consumers

Default input is `rslidar/points/raw`; output is `rslidar/icp/registration`.
Configure your sensor adapter to connect to the node's TCP endpoint and publish
one scan per sample using the binary layout below. It can also connect through
an existing router. A subscriber to the output key receives JSON bytes; the
official [Zenoh C++ publisher/subscriber examples](https://github.com/eclipse-zenoh/zenoh-cpp/tree/1.10.1/examples)
show how to declare these entities with `zenoh::Session`.

For another machine, listen on a reachable interface:

```sh
./build/zenoh/icp_zenoh --listen tcp/0.0.0.0:7447
```

Connect the sensor and result consumer to that machine's actual address, such as
`tcp/192.168.1.50:7447`. `0.0.0.0` is a listen address. Loopback addresses work
only on the same machine/network namespace. Multicast discovery is disabled;
matching keys alone do not establish a connection.

To use a router already listening on port 7447, give the node a distinct listener:

```sh
./build/zenoh/icp_zenoh --listen tcp/127.0.0.1:7448 \
  --connect tcp/127.0.0.1:7447
# Or connect without opening a listener:
./build/zenoh/icp_zenoh --listen '' --connect tcp/127.0.0.1:7447
```

Point the sensor and consumer at the router as well. The application stays in
peer mode. A separate router is optional for the direct connection example.

## Command-line options

Each option takes a separate value, for example `--iterations 50`. Unknown,
repeated, missing, malformed, and out-of-range options fail with exit code 1.

| Option | Default | Meaning |
| --- | --- | --- |
| `--listen` | `tcp/127.0.0.1:7447` | Incoming endpoint; an empty string disables listening. |
| `--connect` | None | Outgoing endpoint, in addition to listening. |
| `--input-key` | `rslidar/points/raw` | Raw XYZI subscription expression. |
| `--output-key` | `rslidar/icp/registration` | Concrete JSON publication key. |
| `--frame-id` | `rslidar` | Sensor-frame label; no coordinate conversion. |
| `--max-points` | `1000000` | Positive declared-record limit before filtering, at most `INT_MAX`. |
| `--iterations` | `100` | Positive maximum number of transform updates. |
| `--time-limit-ms` | Disabled | Nonnegative cooperative registration budget, including setup and residual evaluation. |
| `--rotation-tolerance` | `1e-6` | Incremental rotation tolerance in radians. |
| `--translation-tolerance` | `1e-6` | Incremental translation tolerance in input length units. |
| `--help` | — | Print usage and exit. |

Both transform-change tolerances must be met to report convergence. The time
limit takes priority over the iteration cap, and both take priority over
convergence on the same update. A zero time budget produces an identity candidate
after initialization when the initial geometry/correspondences are usable.
Limits and cancellation are checked between phases: setup or one ICP update can
overrun. `1e-6` illustrates the API; tune tolerances for your sensor noise and units.

Each pair starts from identity. Large motion, low overlap, or repetitive geometry
can prevent correct registration even when a tolerance is reached. This node
supports vanilla 3D ICP only and does not accumulate odometry or forecast motion.

## Raw XYZI wire format

All fields are little-endian. The 16-byte header is:

| Offset | Type | Value |
| --- | --- | --- |
| `0` | `i32` | Capture timestamp seconds. |
| `4` | `u32` | Capture timestamp nanoseconds, less than `1000000000`. |
| `8` | `u32` | Point record count. |
| `12` | `u32` | Record stride, exactly `16`. |

Each following record has `f32 x`, `f32 y`, `f32 z`, `f32 intensity` at byte
offsets 0, 4, 8, and 12. Total payload length must be exactly `16 + count * 16`.
The decoder checks the bound and length before allocating point storage. It
discards records with non-finite XYZ, ignores intensity, and converts XYZ to
double precision internally. Precision already lost in `f32` cannot be recovered.

At least three non-collinear finite points are required. Planar geometry is valid.
Correspondence covariance must also have rank at least two; rank uses a `1e-12`
threshold relative to the largest singular value. The node reports an error when
inputs or matches cannot constrain rotation.

Serialized ROS PointCloud2/CDR, JSON arrays, CSV, and 20-byte segmented records
do not match this format. Convert those inputs in the sensor adapter. Preserve
consistent coordinate frames and length units; the frame label does not convert
them. With meter inputs, translation and RMSE are in meters.

## Registration messages

The output is schema-version-1 JSON. Existing fields remain available; point
counts, initial RMSE, and registration duration are additional fields:

```json
{
  "schema_version": 1,
  "current_stamp": { "sec": 2, "nanosec": 0 },
  "reference_stamp": { "sec": 1, "nanosec": 0 },
  "frame_id": "rslidar",
  "direction": "current_to_reference",
  "status": "converged",
  "transform": {
    "translation": [-0.01, 0.0, 0.0],
    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]
  },
  "rmse": 0.000001,
  "initial_rmse": 0.01,
  "source_count": 1000,
  "target_count": 1000,
  "retained_count": 1000,
  "iteration_count": 2,
  "termination": "transform_change",
  "registration_ms": 2.5,
  "error_code": null
}
```

This is illustrative; exact numbers depend on the scans and machine.

- `transform` maps **current into reference**: `p_reference ≈ R * p_current + t`.
  Translation is `[x,y,z]`; rotation is a normalized **X,Y,Z,W** quaternion.
  The estimate is a point-coordinate transform, not a world-frame vehicle pose.
- `rmse` is `sqrt(sum(distance_squared) / source_count)` using nearest neighbors
  at the reported pose. `initial_rmse` uses identity. Both are computed from
  coordinate residuals independently of the legacy 3D cost.
- `retained_count` equals the number of finite source points for vanilla.
  `source_count` and `target_count` are after filtering.
- `registration_ms` measures validation, setup, ICP, and residual checks. It
  excludes payload decoding, waiting for input, and publishing. Errors before
  alignment report zero duration.
- `status` is `converged`, `limit`, or `error`. `termination` is `transform_change`,
  `maximum_iterations`, or `time_limit`; it is null on error. Error results have
  a null transform, RMSE, initial RMSE, and retained count. Point counts may still
  be known. Decode failures have no trusted current timestamp. `error_code` is
  null for candidates and a stable snake-case code for errors.

Limit results contain candidate poses and do not establish convergence. Numeric
convergence also does not establish a globally correct alignment. Assess RMSE,
overlap, and plausible motion in the consuming application.

## Reference, overload, and shutdown

After priming, every geometrically valid processed scan becomes the next
reference, including after an alignment failure or limit. Decode errors and
invalid input geometry leave the reference unchanged. Restarting clears it.
Arrival order is used; capture timestamps are not sorted or synchronized.

A capacity-one Zenoh ring keeps only the newest pending input while processing
is busy. Intermediate scans may be skipped. Output congestion control permits
drops too. The reference is therefore the previous **processed valid** scan,
which may differ from the previous published or received scan. Use one node and
distinct keys per sensor. Before composing accepted transforms into odometry,
identify the exact reference using `reference_stamp`; missing results or failed
registrations can leave its world pose unknown.

SIGINT and SIGTERM request cooperative shutdown. The idle loop checks every
10 ms; active computation checks between phases. Entities are destroyed before
closing the session. Setup or one processing phase may delay shutdown.

## Tests and troubleshooting

```sh
ctest --test-dir build/zenoh --output-on-failure
# Just the deterministic two-session TCP demonstration:
ctest --test-dir build/zenoh -R zenoh_loopback --output-on-failure
```

Unit tests cover decoding, geometry, known poses, independent residuals, limits,
and reference recovery. The TCP test publishes synthetic scans, receives JSON,
checks error recovery, verifies the capacity-one ring, and stops the worker using
processing barriers. The CLI test checks validation and both shutdown signals.
No LiDAR hardware is needed; socket tests need localhost binding permission.

| Symptom/code | Check |
| --- | --- |
| No result | First scan only primes; subscribe to the output before sending a second valid frame. |
| Connection/port error | Correct TCP address, accessible port, and no conflicting listener. |
| `truncated_header`, `invalid_length` | Header/count must match exact payload length, with no trailing data. |
| `invalid_timestamp`, `invalid_stride` | Nanoseconds below one billion; stride exactly 16. |
| `point_limit` | Reduce records upstream or adjust the bound; this flag does not downsample. |
| `empty_cloud`, `insufficient_correspondences` | Too few usable points remain after filtering. |
| `degenerate_geometry` | Input or nearest-neighbor pairs cannot constrain 3D rotation; inspect geometry and initial overlap. |
| `non_finite_input`, `numerical_failure` | Check coordinate scale and validity. |
| `limit` | Inspect candidate accuracy and tune point count, iteration/time budgets, or tolerances. |
| Unexpected direction | Pose maps current points into reference coordinates; quaternion order is XYZW. |

The source documentation in `tools/icp_zenoh.cpp` and `tools/icp_zenoh_support.h`
is the reference for defaults and contracts.
