#!/usr/bin/env python3

"""
Six-Axis Robot Controller
=========================

Control modes:
    1. Joint-angle control
    2. End-effector XYZ control

Robot:
    6 revolute joints
    No gripper

MoveIt planning group: arm
Base frame:             base_link
End-effector frame:     tool0
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
    "wrist_3_joint"
]

BASE_LINK = "base_link"       
END_EFFECTOR = "tool0"       

PLANNING_GROUP = "arm"

# Joint limits in degrees.
# These must agree with your URDF and MoveIt configuration.

JOINT_LIMITS = {
    "shoulder_pan_joint":   (-360.0, 360.0),
    "shoulder_lift_joint":  (-360.0, 360.0),
    "elbow_joint":          (-360.0, 360.0),
    "wrist_1_joint":        (-360.0, 360.0),
    "wrist_2_joint":        (-360.0, 360.0),
    "wrist_3_joint":        (-360.0, 360.0)
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


def ask_confirmation(message):

    answer = input(
        f"\n{message} [y/N]: "
    ).strip().lower()

    return answer in ("y", "yes")


def select_control_mode():

    while True:

        print("\n======================================")
        print("       ROBOT CONTROL MODE")
        print("======================================")

        print("1. Joint-angle control")
        print("2. End-effector XYZ control")
        print("3. Exit")

        choice = input("\nSelect mode: ").strip()

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

    print("Enter angles in degrees.")
    print("Each angle must be within its configured limits.")

    angles_degrees = []

    for index, joint_name in enumerate(JOINT_NAMES):

        lower, upper = JOINT_LIMITS[joint_name]

        print(f"\nJ{index + 1}: {joint_name}")
        print(f"Allowed range: {lower}° to {upper}°")

        while True:

            angle = read_number(
                f"Enter J{index + 1} angle (degrees)"
            )

            if lower <= angle <= upper:

                angles_degrees.append(angle)
                break

            print(
                f"Out of range! Enter between "
                f"{lower}° and {upper}°."
            )

    print("\n========== TARGET JOINT ANGLES ==========")

    for name, angle in zip(JOINT_NAMES, angles_degrees):

        print(f"{name}: {angle:.2f}°")

    # Convert degrees to radians for MoveIt.

    angles_radians = [
        math.radians(angle)
        for angle in angles_degrees
    ]

    return angles_radians


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

    return x, y, z


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
        trajectory.joint_names,
        first_point.positions
    ):

        print(
            f"  {name}: {math.degrees(position):.2f}°"
        )

    print("\nTarget joint positions:")

    for name, position in zip(
        trajectory.joint_names,
        last_point.positions
    ):

        print(
            f"  {name}: {math.degrees(position):.2f}°"
        )

    duration = last_point.time_from_start

    total_time = (
        duration.sec +
        duration.nanosec / 1e9
    )

    print(f"\nEstimated trajectory duration: {total_time:.2f} seconds")

    print("========================================")


# ============================================================
# PLAN AND EXECUTE
# ============================================================

def plan_and_execute(moveit, trajectory, operation_name):

    if trajectory is None:

        print("\nPlanning failed.")

        failure = moveit.last_failure()

        if failure:
            print("Reason:", failure)

        return False

    if not trajectory.points:

        print("Trajectory has no points.")
        return False

    print(f"\n{operation_name} planned successfully.")

    print_trajectory(trajectory)

    if not ask_confirmation("Execute this movement?"):

        print("Movement cancelled.")
        return False

    if not moveit.execute(trajectory):

        print("MoveIt rejected the trajectory.")

        failure = moveit.last_failure()

        if failure:
            print("Reason:", failure)

        return False

    print("\nExecuting robot movement...")

    success = moveit.wait_until_executed(
        timeout_sec=EXECUTION_TIMEOUT,
        cancel_on_timeout=True
    )

    if success:

        print("\nRobot movement completed successfully!")
        return True

    print("\nRobot movement did not complete.")

    failure = moveit.last_failure()

    if failure:
        print("Reason:", failure)

    return False


# ============================================================
# JOINT-SPACE MOVEMENT
# ============================================================

def move_by_angles(moveit):

    angles = get_joint_angles()

    if not ask_confirmation(
        "Plan movement using these six joint angles?"
    ):

        print("Joint target cancelled.")
        return False

    print("\nMoveIt is planning joint-space movement...")

    trajectory = moveit.plan(

        joint_positions=angles,

        joint_names=JOINT_NAMES,

        tolerance_joint_position=JOINT_TOLERANCE,

        timeout_sec=PLANNING_TIMEOUT
    )

    return plan_and_execute(
        moveit,
        trajectory,
        "Joint-angle movement"
    )


# ============================================================
# END-EFFECTOR XYZ MOVEMENT
# ============================================================

def move_by_xyz(moveit):

    x, y, z = get_xyz_coordinates()

    if not ask_confirmation(
        "Plan movement to this XYZ position?"
    ):

        print("XYZ target cancelled.")
        return False

    print("\nMoveIt is calculating inverse kinematics...")

    trajectory = moveit.plan(

        position=[x, y, z],

        frame_id=BASE_LINK,

        target_link=END_EFFECTOR,

        tolerance_position=POSITION_TOLERANCE,

        timeout_sec=PLANNING_TIMEOUT
    )

    return plan_and_execute(
        moveit,
        trajectory,
        "End-effector XYZ movement"
    )


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
            Parameter(
                "use_sim_time",
                value=True
            )
        ]
    )

    moveit = None

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    spin_thread = None

    try:

        # ----------------------------------------------------
        # Initialize MoveIt
        # ----------------------------------------------------

        moveit = MoveIt2(

            node=node,

            joint_names=JOINT_NAMES,

            base_link_name=BASE_LINK,

            end_effector_name=END_EFFECTOR,

            group_name=PLANNING_GROUP
        )

        executor.add_node(node)

        spin_thread = threading.Thread(
            target=executor.spin,
            daemon=True
        )

        spin_thread.start()

        # ----------------------------------------------------
        # Check joint state feedback
        # ----------------------------------------------------

        print("\nWaiting for robot joint states...")

        if not moveit.wait_for_joint_state(
            timeout_sec=10.0
        ):

            raise RuntimeError(
                "Joint states unavailable. "
                "Check joint_state_broadcaster."
            )

        print("Robot joint states received.")
        print("MoveIt interface initialized.")

        # ----------------------------------------------------
        # Main control loop
        # ----------------------------------------------------

        while True:

            choice = select_control_mode()

            if choice == "1":

                move_by_angles(moveit)

            elif choice == "2":

                move_by_xyz(moveit)

            elif choice == "3":

                print("\nExiting robot controller.")
                break

            if not ask_confirmation(
                "Do you want to perform another movement?"
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
