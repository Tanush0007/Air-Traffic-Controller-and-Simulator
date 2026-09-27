"""
Command-file driver for the Air Traffic Control simulator.

Reads a text file of commands (one per line, matching the assignment's
command table) and calls into student_code.py exactly the way the real
grader does, printing STATUS/FSTATUS results as they occur.

Supported commands:
    CREATE t id max x y z vx vy vz   -> atc.create(t, id, max, (x,y,z), (vx,vy,vz))
    UPDATE t id ax ay az             -> atc.update(t, id, (ax,ay,az))
    STATUS t id                      -> print(atc.status(t, id))
    FLIGHT max x y z vx vy vz        -> f = Flight(max, (x,y,z), (vx,vy,vz))   [single-flight tests]
    ACCEL t ax ay az                 -> f.set_acceleration(t, (ax,ay,az))
    FSTATUS t                        -> print(f.status(t))

CREATE/UPDATE/STATUS use one shared AirTrafficControl instance.
FLIGHT/ACCEL/FSTATUS use one standalone Flight instance (no controller),
matching the assignment's description of "Single Flight Tests" that
exercise the Flight class in isolation.

Usage:
    python3 driver.py tests/visible_cases_1.txt
    python3 driver.py < tests/visible_cases_1.txt
"""
import sys
from student_code import AirTrafficControl, Flight


def run(lines):
    atc = AirTrafficControl()
    flight = None
    output = []

    for raw_line in lines:
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        cmd = parts[0].upper()

        if cmd == 'CREATE':
            t, flight_id, mx = float(parts[1]), parts[2], float(parts[3])
            x, y, z = float(parts[4]), float(parts[5]), float(parts[6])
            vx, vy, vz = float(parts[7]), float(parts[8]), float(parts[9])
            atc.create(t, flight_id, mx, (x, y, z), (vx, vy, vz))

        elif cmd == 'UPDATE':
            t, flight_id = float(parts[1]), parts[2]
            ax, ay, az = float(parts[3]), float(parts[4]), float(parts[5])
            atc.update(t, flight_id, (ax, ay, az))

        elif cmd == 'STATUS':
            t, flight_id = float(parts[1]), parts[2]
            output.append(atc.status(t, flight_id))

        elif cmd == 'FLIGHT':
            mx = float(parts[1])
            x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
            vx, vy, vz = float(parts[5]), float(parts[6]), float(parts[7])
            flight = Flight(mx, (x, y, z), (vx, vy, vz))

        elif cmd == 'ACCEL':
            t = float(parts[1])
            ax, ay, az = float(parts[2]), float(parts[3]), float(parts[4])
            flight.set_acceleration(t, (ax, ay, az))

        elif cmd == 'FSTATUS':
            t = float(parts[1])
            output.append(flight.status(t))

        else:
            raise ValueError(f"Unknown command: {line!r}")

    return output


def main():
    if len(sys.argv) > 1:
        with open(sys.argv[1]) as f:
            lines = f.readlines()
    else:
        lines = sys.stdin.readlines()

    for result in run(lines):
        print(result)


if __name__ == '__main__':
    main()
