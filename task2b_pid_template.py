"""
===================================================
  eLSI Sprint 1 - Task 2B : PID Line Following + Pick & Place (dual line)
===================================================
"""

import time
from connector_2b import CoppeliaClient

# The five line sensors, ordered left -> right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

# =============================================================================
#  TODO (participants): implement the four functions below.
#  You may add helper functions anywhere in this section.
# =============================================================================

integral = 0
previous_error = 0
last_error = 0
black_mode = False
black_mode_counter = 0
count = 1
last_state = 0
last_base_speed = 7.5
last_correction = 0.0

def control_loop(sensors):
    global integral
    global previous_error
    global last_error
    global black_mode
    global count  # Added to fix UnboundLocalError
    global last_state, current_state
    global last_base_speed 
    global last_correction 

    pos = [-2, -1, 0, 1, 2]
    values = [
        sensors['left_corner'],
        sensors['left'],
        sensors['middle'],
        sensors['right'],
        sensors['right_corner']
    ]
    
    if count <= 4:
        max_speed = 7.5
        count = count + 1
        base_speed = max_speed
        correction = 0.0

    else:
        if sum(values) < 0.3:
            base_speed=last_base_speed
            correction=last_correction
            pass
        
        else:
            if sum(values) > 3.5:
                current_state = 1  # White background
                black_mode = True  # Keep your existing black_mode synced
            elif sum(values) < 1.5:
                current_state = 0  # Black background
                black_mode = False # Keep your existing black_mode synced
            else:
                current_state = last_state  # Keep stable if in between

            if current_state != last_state:
                last_state = current_state
                time.sleep(0.005)  # Add 10ms delay
                return 5.0, 5.0    # Move forward straight to cross the line safely
            if not black_mode and sum(values) > 3.5:
                black_mode = True

            if not black_mode:
                max_speed = 7.5
                min_speed = 3.6
                Kp = 2.7
                Kd = 1.4
            else:
                max_speed = 5.9
                min_speed = 2.8
                Kp = 4.5
                Kd = 1.6
                values = [1.0 - v for v in values]

            wsum = 0.0
            ssum = 0.0
            for i in range(5):
                wsum += values[i] * pos[i]
                ssum += values[i]

            # Prevent division by zero if all sensors read 0
            if ssum == 0:
                error = last_error
            else:
                error = wsum / ssum
                
            last_error = error

            Ki = 0.0
            integral += error
            derivative = error - previous_error
            correction = (
                Kp * error +
                Ki * integral +
                Kd * derivative
            )
            previous_error = error

            error_frac = min(abs(error) / 2.0, 1.0)
            base_speed = max_speed - error_frac * (max_speed - min_speed)
    
    last_base_speed = base_speed
    last_correction = correction
    left_speed = base_speed + correction
    right_speed = base_speed - correction
    return left_speed, right_speed


def detect_color(sensors):
    """Identify the colour of the box in front from the RGB sensor."""
    # ----- placeholder: never detects. REPLACE THIS. -----
    pass
    

def should_pick(sensors, carrying_color):
    """Decide whether to send a PICK this cycle."""
    # ----- placeholder: never picks. REPLACE THIS. -----
    pass


def should_drop(sensors, carrying_color):
    """Decide whether to send a DROP this cycle."""
    # ----- placeholder: never drops. REPLACE THIS. -----
    pass
    

# =============================================================================
#  Main loop (Don't Edit this)
# =============================================================================
def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_v1_2b. Running... (Ctrl+C to stop)")

    last_sensors   = None
    carrying_color = None   # colour of the box currently held, or None
    delivered      = 0      # number of boxes released so far

    try:
        while True:
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            # --- Pick (empty-handed only) ---
            if carrying_color is None and should_pick(last_sensors, carrying_color):
                colour_seen = detect_color(last_sensors)     # read BEFORE picking
                success = client.send_pick()
                print(f"PICK attempted (saw {colour_seen!r}) — success={success}")
                if success:
                    carrying_color = colour_seen

            # --- Drop (only while carrying) ---
            if carrying_color is not None and should_drop(last_sensors, carrying_color):
                success = client.send_drop()
                print(f"DROP attempted ({carrying_color!r}) — success={success}")
                if success:
                    delivered += 1
                    carrying_color = None
                    print(f"Delivered {delivered} box(es) so far.")

            # --- Motor command ---
            left, right = control_loop(last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.05)   # ~20 Hz control loop

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0)
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()