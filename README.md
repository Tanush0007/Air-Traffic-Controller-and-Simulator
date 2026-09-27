# Air Traffic Control Simulator

A continuous-time flight simulator in **Python 3** that tracks aircraft
position, velocity, and acceleration under kinematic motion, detecting
crashes, safe landings, and mid-air collisions **at the exact instant they
occur** — not just at the whole-numbered instants when instructions arrive.

Verified against the assignment's own worked example and an additional
**11-check test suite** covering edge cases implied but not spelled out
in the spec (ground-touch "bounces," re-zeroing acceleration exactly at
touchdown, near-misses, and instructions sent to already-finished flights).

---

## Architecture & Core Features

- **Exact Event-Time Detection, Not Polling:** Every terminal condition
  (ground impact, speed limit, collision) is the root of a quadratic in
  elapsed time, solved directly with the quadratic formula (`ground()`)
  rather than stepping time forward in small increments — so an event at
  an irrational time like `t = 7.464...` is caught exactly, not
  approximated.
- **Epsilon-Tolerant Comparisons:** All comparisons that depend on
  floating-point arithmetic go through `is_equal` / `is_less` /
  `is_greater` (and their `_equal` variants) with a fixed `EPS = 1e-6`,
  rather than raw `==`/`<`, so accumulated floating-point error can't
  flip a decision at a boundary.
- **Lazy State Advancement:** A `Flight` only recomputes its position and
  velocity when queried or instructed (`status()`), rather than on a
  fixed clock tick — `future(time)` answers "what would happen by this
  time" without mutating state, and `status(time)` is the only method
  that commits that outcome.
- **Event-Ordered Collision Resolution:** `AirTrafficControl.coll()` finds
  the *earliest* upcoming collision among all currently flying aircraft
  by solving for when their relative position hits zero along each axis,
  and `simulation()`/`forward()` step the whole system to exactly that
  instant before resolving it — so an earlier accident correctly removes
  a flight from a later collision it would otherwise have been part of,
  without that accident creating any new collision for others.
- **Deterministic Tie-Breaking:** At a single instant, structural damage
  and ground impact are resolved before collisions are even checked, so
  a flight that has just crashed or landed is never available to collide
  with another flight at that same instant.
- **Two Independent Object Models:** `Flight` behaves correctly in total
  isolation (no controller needed) for single-flight tests, while
  `AirTrafficControl` owns a collection of flights and adds the
  cross-flight logic (collision detection) that no single `Flight` could
  compute on its own.

---

## Running It

No external dependencies — just Python 3.

**1. Run a command file through the driver:**

```bash
python3 src/driver.py tests/visible_cases_1.txt
```

or pipe commands in directly:

```bash
python3 src/driver.py < tests/visible_cases_1.txt
```

**2. Run the automated test suite:**

```bash
python3 tests/test_atc.py
```

Expected output:

```
11 checks run, 11 passed, 0 failed.
```

## Command Reference

| Command | Effect |
|---|---|
| `CREATE t id max x y z vx vy vz` | `atc.create(t, id, max, (x,y,z), (vx,vy,vz))` |
| `UPDATE t id ax ay az` | `atc.update(t, id, (ax,ay,az))` |
| `STATUS t id` | prints `atc.status(t, id)` |
| `FLIGHT max x y z vx vy vz` | `f = Flight(max, (x,y,z), (vx,vy,vz))` |
| `ACCEL t ax ay az` | `f.set_acceleration(t, (ax,ay,az))` |
| `FSTATUS t` | prints `f.status(t)` |

`CREATE`/`UPDATE`/`STATUS` share one `AirTrafficControl` instance;
`FLIGHT`/`ACCEL`/`FSTATUS` exercise a single standalone `Flight`, matching
the assignment's distinction between "Air Traffic Control Tests" and
"Single Flight Tests."

---

## Project Structure

```
src/
    student_code.py       Flight and AirTrafficControl classes (the actual submission)
    driver.py             Parses a command file and calls into student_code.py
tests/
    visible_cases_1.txt   The assignment's own worked example
    test_atc.py            Automated test suite (11 hand-derived + spec checks)
```

Note: unlike a C++ project, Python has no separate header/implementation
split, so there's no `include/` folder here — `src/` holds the actual
code, `tests/` holds everything that exercises it.

---

## Verified Behavior

Each of these was worked out by hand from the spec's physics, then
confirmed against the actual code:

- Speed exactly at the maximum is fine; only *exceeding* it is an accident.
- A descending flight that reaches `z = 0` while `v.z < 0` crashes at the
  exact (possibly irrational) instant it happens.
- A flight that grazes `z = 0` with `v = 0` but with acceleration still
  pushing it *upward* (`a.z > 0`) is correctly treated as still flying —
  not landed, not crashed — matching the spec's explicit carve-out for
  this case.
- The same trajectory, with acceleration re-zeroed by an instruction
  arriving exactly at the moment of touchdown, is correctly classified as
  a genuine safe landing.
- Two flights on a collision course are both marked `accident` at the
  exact instant their positions coincide; a near-miss at a different
  altitude is not.
- Querying a flight id that was never created returns `does not exist`
  rather than raising an error.
- Sending an instruction to a flight that has already landed or crashed
  is silently ignored, per spec.

---

## Development Notes

*(Left for you to fill in with your own account of the assignment — what
you found tricky, any bugs you hit while implementing `coll()` or the
quadratic root-finding in `ground()`, or how you reasoned through the
event-ordering rule the spec calls out.)*

---

## Possible Updates

- **N-flight collision clustering:** `coll()` is O(n²) over currently
  flying aircraft per event; fine for the assignment's scale, but a
  spatial index (grid or k-d tree) would help at much larger fleet sizes.
- **Structured driver output:** Extend `driver.py` to optionally emit
  JSON instead of one line per query, for easier diffing against a
  reference grader.
