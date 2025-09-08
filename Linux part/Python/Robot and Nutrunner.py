#Robot and Nutrunner

def robot():
    print("Start")
    approach_screw_position()
    send_signal_to_nutrunner("start")

    enable_compliant_mode()
    move_up_to_touch_screw()

    # Search and engage
    while True:
        search_spiral_path()
        if get_torque_z() < -5:
            break
    stop_searching()

    # Unfasten
    follow_screw_movement()

    # Assess and leave
    while True:
        if z_position_decreased():
            break

    disable_compliant_mode()
    move_down()
    send_signal_to_nutrunner("stop")
    print("End")


def nutrunnner():
    print("Waiting for signal from robot...")
    while not received_signal_from_robot("start"):
        pass

    rotate_low_speed_fastening()

    while True:
        if get_torque() > 10:
            break

    rotate_high_speed_unfastening()

    while not received_signal_from_robot("stop"):
        pass

    stop_rotation()


# --- Simulated helper functions for both robot and nutrunnner logic ---

def approach_screw_position():
    print("Approaching position below the screw head...")

def send_signal_to_nutrunner(signal_type):
    print(f"Sending '{signal_type}' signal to nutrunnner...")

def enable_compliant_mode():
    print("Compliant mode enabled")

def move_up_to_touch_screw():
    print("Moving up to contact screw head...")

def search_spiral_path():
    print("Searching in spiral path for screw alignment...")

def get_torque_z():
    # Simulated torque along Z axis
    return -6  # Example torque value

def stop_searching():
    print("Stopped searching after alignment")

def follow_screw_movement():
    print("Following the screw's movement during unfastening...")

def z_position_decreased():
    # Simulate detection of screw being pulled out
    return True

def disable_compliant_mode():
    print("Compliant mode disabled")

def move_down():
    print("Robot moving down to leave")

def received_signal_from_robot(signal_type):
    # Simulate signal reception from robot
    return True

def rotate_low_speed_fastening():
    print("Nutrunner rotating at low speed in fastening direction...")

def get_torque():
    # Simulated torque reading
    return 12  # Example high torque value

def rotate_high_speed_unfastening():
    print("Nutrunner rotating at high speed in unfastening direction...")

def stop_rotation():
    print("Nutrunner stopped rotating")