#!/usr/bin/env python3

"""
Six-Axis Robot Controller
=========================

Control modes:
    1. Joint-angle control
    2. End-effector XYZ control
    3. Teach-and-confirm pick-and-place

Pick-and-place teaching:
    - Repeatedly command a joint-angle or XYZ target.
    - The robot moves to each target.
    - Confirm whether the current position is the final PICK/PLACE point.
    - Once both points are taught, enter the number of pick-to-place cycles.
    - Each leg is planned from the robot's latest state.

IMPORTANT:
    Joint-angle targets are ABSOLUTE angles in degrees.
    Example: if J1 is at 60 degrees and you want it at 80 degrees,
    enter 80 degrees, not +20 degrees.

Robot:
    6 revolute joints
    No gripper configured
"""

import sys
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from pymoveit2 import MoveIt2


# ============================================================
# ROBOT CONFIGURATION
# ============================================================

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

BASE_LINK = "base_link"
END_EFFECTOR = "tool0"
PLANNING_GROUP = "arm"

# These limits must agree with the URDF and MoveIt configuration.
JOINT_LIMITS = {
    "shoulder_pan_joint": (-360.0, 360.0),
    "shoulder_lift_joint": (-360.0, 360.0),
    "elbow_joint": (-360.0, 360.0),
    "wrist_1_joint": (-360.0, 360.0),
    "wrist_2_joint": (-360.0, 360.0),
    "wrist_3_joint": (-360.0, 360.0),
}

POSITION_TOLERANCE = 0.005
JOINT_TOLERANCE = 0.001

PLANNING_TIMEOUT = 30.0
EXECUTION_TIMEOUT = 60.0


# ============================================================
# USER INPUT
# ============================================================

def read_number(label):
    while True:
        try:
            value = float(input(f"{label}: ").strip())

            if math.isfinite(value):
                return value

            print("Please enter a finite number.")

        except ValueError:
            print("Invalid input. Try again.")

        except KeyboardInterrupt:
            raise


def read_positive_integer(label):
    while True:
        try:
            value = int(input(f"{label}: ").strip())

            if value > 0:
                return value

            print("Enter a number greater than zero.")

        except ValueError:
            print("Please enter a valid whole number.")

        except KeyboardInterrupt:
            raise


def ask_confirmation(message):
    answer = input(f"\n{message} [y/N]: ").strip().lower()
    return answer in ("y", "yes")


def select_control_mode():
    while True:
        print("\n======================================")
        print("       ROBOT CONTROL MODE")
        print("======================================")
        print("1. Joint-angle control")
        print("2. End-effector XYZ control")
        print("3. Pick-and-place (teach positions)")
        print("4. Exit")

        choice = input("\nSelect mode: ").strip()

        if choice in ("1", "2", "3", "4"):
            return choice

        print("Invalid selection. Choose 1, 2, 3 or 4.")


def select_target_mode(position_name):
    while True:
        print(f"\n--- Set {position_name} position ---")
        print("1. Joint angles (degrees)")
        print("2. End-effector XYZ (metres)")
        print("3. Cancel teaching")

        choice = input("Select input type: ").strip()

        if choice in ("1", "2", "3"):
            return choice

        print("Invalid selection. Choose 1, 2 or 3.")


# ============================================================
# JOINT ANGLE INPUT
# ============================================================

def get_joint_angles():
    print("\n======================================")
    print("        SIX JOINT ANGLE INPUT")
    print("======================================")
    print("Enter ABSOLUTE joint angles in degrees.")
    print("These are target angles, not angle increments.")

    angles_degrees = []

    for index, joint_name in enumerate(JOINT_NAMES):
        lower, upper = JOINT_LIMITS[joint_name]

        print(f"\nJ{index + 1}: {joint_name}")
        print(f"Allowed range: {lower}° to {upper}°")

        while True:
            angle = read_number(
                f"Enter J{index + 1} target angle (degrees)"
            )

            if lower <= angle <= upper:
                angles_degrees.append(angle)
                break

            print(
                f"Out of range! Enter between {lower}° and {upper}°."
            )

    print("\n========== TARGET JOINT ANGLES ==========")
    for name, angle in zip(JOINT_NAMES, angles_degrees):
        print(f"{name}: {angle:.2f}°")

    return [math.radians(angle) for angle in angles_degrees]


# ============================================================
# XYZ INPUT
# ============================================================

def get_xyz_coordinates():
    print("\n======================================")
    print("       END-EFFECTOR XYZ INPUT")
    print("======================================")
    print("Enter target coordinates in metres.")

    x = read_number("X coordinate")
    y = read_number("Y coordinate")
    z = read_number("Z coordinate")

    print("\n========== TARGET POSITION ==========")
    print(f"X: {x:.4f} m")
    print(f"Y: {y:.4f} m")
    print(f"Z: {z:.4f} m")

    return (x, y, z)


# ============================================================
# TRAJECTORY INFORMATION
# ============================================================

def print_trajectory(trajectory):
    if trajectory is None:
        print("No trajectory available.")
        return

    if not trajectory.points:
        print("Trajectory contains no points.")
        return

    first_point = trajectory.points[0]
    last_point = trajectory.points[-1]

    print("\n========== TRAJECTORY DETAILS ==========")
    print("Joint names:", trajectory.joint_names)

    print("\nStarting joint positions:")
    for name, position in zip(
        trajectory.joint_names, first_point.positions
    ):
        print(f"  {name}: {math.degrees(position):.2f}°")

    print("\nTarget joint positions:")
    for name, position in zip(
        trajectory.joint_names, last_point.positions
    ):
        print(f"  {name}: {math.degrees(position):.2f}°")

    duration = last_point.time_from_start
    total_time = duration.sec + duration.nanosec / 1e9

    print(f"\nEstimated trajectory duration: {total_time:.2f} seconds")
    print("========================================")


# ============================================================
# PLAN A TARGET
# ============================================================

def plan_target(moveit, target, operation_name):
    """
    target format:
        {"mode": "joint", "values": [six joint positions in radians]}
        {"mode": "xyz", "values": (x, y, z) in metres}
    """

    print(f"\nPlanning: {operation_name}")

    if target["mode"] == "joint":
        trajectory = moveit.plan(
            joint_positions=target["values"],
            joint_names=JOINT_NAMES,
            tolerance_joint_position=JOINT_TOLERANCE,
            timeout_sec=PLANNING_TIMEOUT,
        )

    elif target["mode"] == "xyz":
        x, y, z = target["values"]

        trajectory = moveit.plan(
            position=[x, y, z],
            frame_id=BASE_LINK,
            target_link=END_EFFECTOR,
            tolerance_position=POSITION_TOLERANCE,
            timeout_sec=PLANNING_TIMEOUT,
        )

    else:
        print("Unknown target type.")
        return None

    if trajectory is None or not trajectory.points:
        print(f"\nPlanning failed for: {operation_name}")

        failure = moveit.last_failure()
        if failure:
            print("Reason:", failure)

        return None

    print(f"\n{operation_name} planned successfully.")
    print_trajectory(trajectory)

    return trajectory


# ============================================================
# EXECUTE A PLANNED TRAJECTORY
# ============================================================

def execute_trajectory(moveit, trajectory, operation_name,
                        ask_before_execution=True):
    if trajectory is None or not trajectory.points:
        print("Cannot execute: trajectory is empty.")
        return False

    if ask_before_execution:
        if not ask_confirmation("Execute this movement?"):
            print("Movement cancelled.")
            return False

    print(f"\nExecuting: {operation_name}")

    if not moveit.execute(trajectory):
        print("MoveIt rejected the trajectory.")

        failure = moveit.last_failure()
        if failure:
            print("Reason:", failure)

        return False

    success = moveit.wait_until_executed(
        timeout_sec=EXECUTION_TIMEOUT,
        cancel_on_timeout=True,
    )

    if success:
        print(f"\n{operation_name} completed successfully!")
        return True

    print(f"\n{operation_name} did not complete.")

    failure = moveit.last_failure()
    if failure:
        print("Reason:", failure)

    return False


def move_to_target(moveit, target, operation_name,
                   ask_before_execution=True):
    trajectory = plan_target(moveit, target, operation_name)

    if trajectory is None:
        return False

    return execute_trajectory(
        moveit,
        trajectory,
        operation_name,
        ask_before_execution=ask_before_execution,
    )


# ============================================================
# NORMAL JOINT / XYZ MOVEMENT
# ============================================================

def move_by_angles(moveit):
    angles = get_joint_angles()

    if not ask_confirmation(
        "Plan movement using these six absolute joint angles?"
    ):
        print("Joint target cancelled.")
        return False

    target = {"mode": "joint", "values": angles}

    return move_to_target(
        moveit, target, "Joint-angle movement"
    )


def move_by_xyz(moveit):
    xyz = get_xyz_coordinates()

    if not ask_confirmation("Plan movement to this XYZ position?"):
        print("XYZ target cancelled.")
        return False

    target = {"mode": "xyz", "values": xyz}

    return move_to_target(
        moveit, target, "End-effector XYZ movement"
    )


# ============================================================
# TEACH PICK / PLACE POSITION
# ============================================================

def teach_position(moveit, position_name):
    """
    Repeatedly accepts a joint-angle or XYZ target and moves the robot.
    After each successful movement, asks whether the current robot
    position is the final taught point.

    The last successfully reached target is saved when the user confirms.
    """

    print("\n")
    print("=" * 48)
    print(f"       TEACH {position_name.upper()} POSITION")
    print("=" * 48)
    print(
        "You can make as many adjustments as needed.\n"
        "After each movement, confirm whether the position is final."
    )

    adjustment_number = 1

    while True:
        print(
            f"\n========== {position_name.upper()} "
            f"ADJUSTMENT {adjustment_number} =========="
        )

        mode = select_target_mode(position_name)

        if mode == "3":
            print(f"{position_name} teaching cancelled.")
            return None

        if mode == "1":
            values = get_joint_angles()
            target = {"mode": "joint", "values": values}

        else:
            values = get_xyz_coordinates()
            target = {"mode": "xyz", "values": values}

        if not ask_confirmation(
            f"Move robot to this {position_name} adjustment?"
        ):
            print("Target not executed. You can enter another adjustment.")
            adjustment_number += 1
            continue

        success = move_to_target(
            moveit,
            target,
            f"{position_name} adjustment {adjustment_number}",
            ask_before_execution=False,
        )

        if not success:
            print("This adjustment was not completed.")
            if ask_confirmation("Try another adjustment?"):
                adjustment_number += 1
                continue

            return None

        if ask_confirmation(
            f"Is this the FINAL {position_name.upper()} position?"
        ):
            print(f"\n{position_name.upper()} position saved.")
            return target

        print(
            f"\n{position_name} position not finalized. "
            "Enter another target to adjust the robot."
        )

        adjustment_number += 1


# ============================================================
# PICK-AND-PLACE CYCLE
# ============================================================

def run_pick_and_place(moveit):
    print("\n==========================================")
    print("          PICK-AND-PLACE SETUP")
    print("==========================================")

    # Teach the pick point.
    pick_target = teach_position(moveit, "pick")

    if pick_target is None:
        print("Pick-and-place setup cancelled.")
        return False

    # Teach the place point.
    place_target = teach_position(moveit, "place")

    if place_target is None:
        print("Pick-and-place setup cancelled.")
        return False

    # Ask how many complete pick-to-place transfers are required.
    print("\n==========================================")
    print("       PICK-AND-PLACE CONFIGURATION")
    print("==========================================")

    cycles = read_positive_integer(
        "How many pick-to-place cycles should be performed"
    )

    print("\n========== FINAL CONFIGURATION ==========")
    print("Pick position :", pick_target["mode"].upper())
    print("Place position:", place_target["mode"].upper())
    print("Number of cycles:", cycles)

    print("\nMovement sequence:")
    print("  PICK -> PLACE")
    print(f"  Repeated {cycles} time(s).")

    if not ask_confirmation(
        "Start the pick-and-place sequence?"
    ):
        print("Pick-and-place sequence cancelled.")
        return False

    # Each target is planned again immediately before execution.
    # This ensures planning starts from the latest robot state.
    for cycle in range(1, cycles + 1):
        print("\n")
        print("#" * 52)
        print(f"          PICK-AND-PLACE CYCLE {cycle}/{cycles}")
        print("#" * 52)

        # Move to the taught pick position.
        print(f"\n[CYCLE {cycle}] Moving to PICK position...")

        success = move_to_target(
            moveit,
            pick_target,
            f"Cycle {cycle}: Move to PICK",
            ask_before_execution=False,
        )

        if not success:
            print(
                "\nPick-and-place stopped because movement to "
                "the PICK position failed."
            )
            return False

        # Move to the taught place position.
        print(f"\n[CYCLE {cycle}] Moving to PLACE position...")

        success = move_to_target(
            moveit,
            place_target,
            f"Cycle {cycle}: Move to PLACE",
            ask_before_execution=False,
        )

        if not success:
            print(
                "\nPick-and-place stopped because movement to "
                "the PLACE position failed."
            )
            return False

        print(f"\nCycle {cycle}/{cycles} completed.")

    print("\n==========================================")
    print("     PICK-AND-PLACE COMPLETED")
    print("==========================================")
    print(f"Successfully completed {cycles} cycle(s).")
    print("Robot is at the PLACE position.")

    return True


# ============================================================
# MAIN
# ============================================================

def main():
    print("\n==========================================")
    print("       SIX-AXIS ROBOT CONTROLLER")
    print("==========================================")

    print("\nRobot configuration:")
    print("  Number of joints : 6")
    print("  Joint type       : Revolute")
    print("  Gripper          : Not configured")
    print("  Planning group   :", PLANNING_GROUP)
    print("  Base frame       :", BASE_LINK)
    print("  End-effector     :", END_EFFECTOR)

    rclpy.init()

    node = Node(
        "six_axis_robot_controller",
        parameter_overrides=[
            Parameter("use_sim_time", value=True)
        ],
    )

    moveit = None
    executor = MultiThreadedExecutor(num_threads=4)
    spin_thread = None

    try:
        # Initialize MoveIt.
        moveit = MoveIt2(
            node=node,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=PLANNING_GROUP,
        )

        executor.add_node(node)

        spin_thread = threading.Thread(
            target=executor.spin,
            daemon=True,
        )
        spin_thread.start()

        # Check joint-state feedback.
        print("\nWaiting for robot joint states...")

        if not moveit.wait_for_joint_state(timeout_sec=10.0):
            raise RuntimeError(
                "Joint states unavailable. "
                "Check joint_state_broadcaster."
            )

        print("Robot joint states received.")
        print("MoveIt interface initialized.")

        # Main control loop.
        while True:
            choice = select_control_mode()

            if choice == "1":
                move_by_angles(moveit)

            elif choice == "2":
                move_by_xyz(moveit)

            elif choice == "3":
                run_pick_and_place(moveit)

            elif choice == "4":
                print("\nExiting robot controller.")
                break

            if not ask_confirmation(
                "Do you want to perform another operation?"
            ):
                print("\nRobot control session completed.")
                break

    except KeyboardInterrupt:
        print("\nRobot controller interrupted.")

        if moveit is not None:
            moveit.cancel_execution()

        return 130

    except Exception as exc:
        print(f"\nERROR: {exc}")
        return 1

    finally:
        print("\nShutting down robot controller...")

        if moveit is not None:
            moveit.destroy()

        executor.shutdown()

        if spin_thread is not None:
            spin_thread.join(timeout=2.0)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("ROS 2 shutdown complete.")

    return 0


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    sys.exit(main())
