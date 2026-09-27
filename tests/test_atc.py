"""
Automated test suite for the Air Traffic Control simulator.

Every expected value below was hand-derived from the assignment spec's
physics (or taken directly from the assignment's own worked example) and
then confirmed by running it through student_code.py -- these are
correctness checks against the spec, not just recorded/regression output.

Run:
    python3 tests/test_atc.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from driver import run

g_checks = 0
g_failed = 0


def check(actual, expected, label):
    global g_checks, g_failed
    g_checks += 1
    if actual != expected:
        g_failed += 1
        print(f"FAIL [{label}]  expected={expected!r}  actual={actual!r}")


def test_assignment_sample():
    # Directly from the assignment spec (visible_cases_1.txt).
    lines = [
        "FLIGHT 10 0 0 4 0 0 0",
        "ACCEL 0 0 0 2",
        "FSTATUS 0",
        "ACCEL 2 0 0 -2",
        "FSTATUS 5",
        "FSTATUS 7",
        "FSTATUS 8",
    ]
    out = run(lines)
    check(out, ["flying", "flying", "flying", "accident"], "assignment_sample")


def test_exceeds_max_speed_is_accident():
    # Speed strictly greater than max at creation -> accident.
    out = run(["FLIGHT 5 0 0 10 6 0 0", "FSTATUS 0"])
    check(out, ["accident"], "exceeds_max_speed")


def test_exactly_max_speed_is_fine():
    # Exactly at max speed is fine per spec ("only exceeding destroys it").
    out = run(["FLIGHT 5 0 0 10 5 0 0", "FSTATUS 0"])
    check(out, ["flying"], "exactly_max_speed")


def test_crash_while_descending():
    # z=10, vz=-5, a=0 -> hits ground at t=2 while still descending.
    out = run(["FLIGHT 100 0 0 10 0 0 -5", "FSTATUS 1", "FSTATUS 2", "FSTATUS 3"])
    check(out, ["flying", "accident", "accident"], "crash_while_descending")


def test_immediate_safe_landing_at_creation():
    # z=0, v=0, a=0 at creation -> landed safely immediately, per spec.
    out = run(["FLIGHT 10 0 0 0 0 0 0", "FSTATUS 0"])
    check(out, ["landed safely"], "immediate_safe_landing")


def test_bounce_case_stays_flying():
    # Touches z=0 with v=0 but acceleration still pushing upward (a.z>0):
    # spec explicitly treats this as still flying, not landed or crashed.
    out = run([
        "FLIGHT 100 0 0 8 0 0 -4",
        "ACCEL 0 0 0 1",
        "FSTATUS 3", "FSTATUS 4", "FSTATUS 5",
    ])
    check(out, ["flying", "flying", "flying"], "bounce_case")


def test_genuine_safe_landing_after_rezeroing_accel():
    # Same trajectory as the bounce case, but acceleration is re-zeroed
    # exactly at the moment of touchdown -> genuine safe landing.
    out = run([
        "FLIGHT 100 0 0 8 0 0 -4",
        "ACCEL 0 0 0 1",
        "ACCEL 4 0 0 0",
        "FSTATUS 4", "FSTATUS 10",
    ])
    check(out, ["landed safely", "landed safely"], "safe_landing_after_rezero")


def test_two_flights_collide():
    out = run([
        "CREATE 0 A 100 0 0 10 1 0 0",
        "CREATE 0 B 100 10 0 10 -1 0 0",
        "STATUS 5 A",
        "STATUS 5 B",
        "STATUS 10 A",
    ])
    check(out, ["accident", "accident", "accident"], "two_flights_collide")


def test_near_miss_does_not_collide():
    # Same x-closing-speed scenario, but different altitudes -> no collision.
    out = run([
        "CREATE 0 A 100 0 0 10 1 0 0",
        "CREATE 0 B 100 10 0 20 -1 0 0",
        "STATUS 5 A",
        "STATUS 5 B",
    ])
    check(out, ["flying", "flying"], "near_miss")


def test_unknown_flight_id():
    out = run(["STATUS 0 GHOST"])
    check(out, ["does not exist"], "unknown_flight_id")


def test_instruction_to_finished_flight_has_no_effect():
    out = run([
        "CREATE 0 A 100 0 0 10 1 0 0",
        "CREATE 0 B 100 10 0 10 -1 0 0",
        "STATUS 5 A",           # collision -> accident
        "UPDATE 6 A 0 0 5",     # instruction to a finished flight: no effect
        "STATUS 7 A",           # still accident, no error
    ])
    check(out, ["accident", "accident"], "instruction_to_finished_flight")


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()

    print(f"\n{g_checks} checks run, {g_checks - g_failed} passed, {g_failed} failed.")
    return 0 if g_failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
