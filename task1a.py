"""
===================================================
    eLSI Sprint 1 - Task 1A : PID Line Following
===================================================

Participant template.

HOW TO RUN
  1. Open the Task 1A scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_task1a.py --eval
  3. Run this file:      python3 task1a_template.py

WHAT YOU IMPLEMENT
  Only control_loop(). Everything else (connecting, receiving sensors,
  sending motor commands) is handled for you by CoppeliaClient.
  Don't Edit this file except control_loop().
  You can add helper functions if you like.

Team ID: [ XXX ]
"""

import time

from connector_task1a import CoppeliaClient

# The five line sensors, ordered left -> right across the robot.
# Each value is in [0.0, 1.0]; a higher value means the line is detected.
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

integral = 0
previous_error = 0
last_error = 0
black_mode = False

def control_loop(sensors):

    global integral
    global previous_error
    global last_error
    global black_mode

    pos = [-2, -1, 0, 1, 2]

    values = [
        sensors['left_corner'],
        sensors['left'],
        sensors['middle'],
        sensors['right'],
        sensors['right_corner']
    ]

    if not black_mode and sum(values) > 3.5:
        black_mode = True

    
    if not black_mode:
        base_speed = 4
        Kp = 2.2

    
    else:
        base_speed = 3
        Kp = 3.6

        values = [1.0 - v for v in values]

    wsum = 0.0
    ssum = 0.0

    for i in range(5):
        wsum += values[i] * pos[i]
        ssum += values[i]

    
    error = wsum / ssum
    last_error = error
    
    Ki = 0.0
    Kd = 0.0

    integral += error

    derivative = error - previous_error

    correction = (
        Kp * error +
        Ki * integral +
        Kd * derivative
    )

    previous_error = error

    left_speed = base_speed + correction
    right_speed = base_speed - correction

    return left_speed, right_speed
    
def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_task1a. Running... (Ctrl+C to stop)")

    last_sensors = None
    try:
        while True:
            # Pull the freshest sensor packet; reuse the last one between packets.
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            left, right = control_loop (last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.05)   # ~20 Hz control loop
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0)   # stop the robot
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()
