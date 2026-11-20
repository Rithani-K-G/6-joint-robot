#!/usr/bin/env python3

"""
Six-Joint Robot Controller
==========================

Robot configuration:
    J1-J4 : Revolute arm joints
    J5-J6 : Prismatic gripper joints

Architecture:
    Python
       |
       v
    MoveIt 2
       |
       +---- arm planning group
       |         |
       |         v
       |     arm_controller
       |
       +---- gripper planning group
                 |
                 v
             gripper_controller
       |
       v
    Gazebo Harmonic

Features:
    1. Accept XYZ coordinates in metres.
    2. Plan arm movement using inverse kinematics.
    3. Inspect the generated trajectory.
    4. Execute the arm trajectory.
    5. Open and close gripper using MoveIt.
    6. Repeat multiple XYZ movements.

Reference frame: base_link
Arm end-effector: tool0
Gripper reference: wrist_link
"""

import sys
import math
import threading

import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from pymoveit2 import MoveIt2


# ============================================================
# ROBOT CONFIGURATION
# ============================================================

# Four revolute arm joints
ARM_JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_joint"
]

# Two prismatic gripper joints
GRIPPER_JOINT_NAMES = [
    "gripper_1_joint",
    "gripper_2_joint"
]

# Robot frames
BASE_LINK = "base_link"
ARM_END_EFFECTOR = "tool0"

# Gripper group uses joint-space targets.
# This link is only a reference for the MoveIt interface.
GRIPPER_REFERENCE_LINK = "gripper_1_link"
GRIPPER_BASE_LINK = "wrist_link"

# MoveIt planning groups
ARM_GROUP = "arm"
GRIPPER_GROUP = "gripper"

# Gripper positions (metres)
GRIPPER_OPEN = [0.020, 0.020]
GRIPPER_CLOSED = [0.000, 0.000]

# Planning parameters
ARM_POSITION_TOLERANCE = 0.005
PLANNING_TIMEOUT = 30.0
EXECUTION_TIMEOUT = 60.0

# Gripper planning tolerance (metres)
GRIPPER_TOLERANCE = 0.001


# ============================================================
# TERMINAL INPUT
# ============================================================

def read_coordinate(label: str) -> float:
    """Read one finite XYZ coordinate in metres."""

    while True:

        try:

            value = float(
                input(f"{label} (metres): ").strip()
            )

            if math.isfinite(value):
                return value

            print("Enter a finite number.")

        except ValueError:

            print("Invalid input. Example: 0.30")

        except KeyboardInterrupt:

            raise


def ask_confirmation(message: str) -> bool:
    """Ask the user for confirmation."""

    answer = input(
        f"\n{message} [y/N]: "
    ).strip().lower()

    return answer in ("y", "yes")


# ============================================================
# TRAJECTORY DEBUGGING
# ============================================================

def print_trajectory(trajectory, joint_type="arm"):
    """Display trajectory information."""

    if trajectory is None:
        print("No trajectory available.")
        return

    if not trajectory.points:
        print("Trajectory contains no points.")
        return

    print("\n========== TRAJECTORY DEBUG ==========")

    print("Joint names:", trajectory.joint_names)

    first_point = trajectory.points[0]
    last_point = trajectory.points[-1]

    print("\nStarting positions:")

    for name, position in zip(
        trajectory.joint_names,
        first_point.positions
    ):

        if joint_type == "arm":
            print(
                f"  {name}: {math.degrees(position):.3f} degrees"
            )
        else:
            print(
                f"  {name}: {position:.5f} metres"
            )

    print("\nFinal positions:")

    for name, position in zip(
        trajectory.joint_names,
        last_point.positions
    ):

        if joint_type == "arm":
            print(
                f"  {name}: {math.degrees(position):.3f} degrees"
            )
        else:
            print(
                f"  {name}: {position:.5f} metres"
            )

    duration = last_point.time_from_start

    total_time = (
        duration.sec +
        duration.nanosec / 1e9
    )

    print(f"\nTrajectory duration: {total_time:.3f} seconds")

    print("======================================\n")


# ============================================================
# PLAN AND EXECUTE
# ============================================================

def execute_trajectory(
    moveit,
    trajectory,
    operation_name
):
    """Execute a previously planned MoveIt trajectory."""

    if trajectory is None:

        print(f"\n{operation_name}: Planning failed.")

        failure = moveit.last_failure()

        if failure:
            print("Reason:", failure)

        return False

    print(f"\n{operation_name}: Trajectory planned successfully.")

    if not trajectory.points:

        print("Trajectory contains no points.")
        return False

    print_trajectory(
        trajectory,
        joint_type=(
            "gripper"
            if "gripper" in operation_name.lower()
            else "arm"
        )
    )

    if not ask_confirmation("Execute this trajectory?"):

        print("Trajectory planned but not executed.")
        return False

    if not moveit.execute(trajectory):

        print("MoveIt rejected the trajectory.")

        failure = moveit.last_failure()

        if failure:
            print("Reason:", failure)

        return False

    print("\nTrajectory sent. Waiting for execution...")

    success = moveit.wait_until_executed(
        timeout_sec=EXECUTION_TIMEOUT,
        cancel_on_timeout=True
    )

    if success:

        print(f"{operation_name} completed successfully!")
        return True

    print(f"{operation_name} did not complete successfully.")

    failure = moveit.last_failure()

    if failure:
        print("Reason:", failure)

    return False


# ============================================================
# ARM XYZ MOVEMENT
# ============================================================

def move_arm_to_xyz(
    arm_moveit,
    x,
    y,
    z
):
    """Plan and execute Cartesian arm movement."""

    print("\n========== ARM MOVEMENT ==========")

    print(
        f"Target XYZ: X={x:.4f}, Y={y:.4f}, Z={z:.4f} m"
    )

    print("\nMoveIt is calculating inverse kinematics...")

    trajectory = arm_moveit.plan(

        position=[x, y, z],

        frame_id=BASE_LINK,

        target_link=ARM_END_EFFECTOR,

        tolerance_position=ARM_POSITION_TOLERANCE,

        timeout_sec=PLANNING_TIMEOUT
    )

    if trajectory is None:

        print("\nNo valid arm trajectory found.")

        failure = arm_moveit.last_failure()

        if failure:
            print("Planning failure:", failure)

        return False

    return execute_trajectory(
        arm_moveit,
        trajectory,
        "Arm movement"
    )


# ============================================================
# GRIPPER MOVEMENT
# ============================================================

def move_gripper(
    gripper_moveit,
    target_positions,
    operation_name
):
    """Plan and execute gripper joint movement through MoveIt."""

    print("\n========== GRIPPER MOVEMENT ==========")

    print(f"Operation: {operation_name}")

    print(
        f"Gripper 1 target: {target_positions[0]:.3f} m"
    )

    print(
        f"Gripper 2 target: {target_positions[1]:.3f} m"
    )

    print("\nMoveIt is planning gripper movement...")

    trajectory = gripper_moveit.plan(

        joint_positions=target_positions,

        joint_names=GRIPPER_JOINT_NAMES,

        tolerance_joint_position=GRIPPER_TOLERANCE,

        timeout_sec=PLANNING_TIMEOUT
    )

    return execute_trajectory(
        gripper_moveit,
        trajectory,
        f"Gripper {operation_name}"
    )


# ============================================================
# GRIPPER MENU
# ============================================================

def gripper_menu(gripper_moveit):
    """Allow the user to open or close the gripper."""

    while True:

        print("\n====================================")
        print("           GRIPPER CONTROL")
        print("====================================")

        print("1. Open gripper")
        print("2. Close gripper")
        print("3. Skip gripper operation")
        print("4. Exit program")

        choice = input("\nEnter choice: ").strip()

        if choice == "1":

            move_gripper(
                gripper_moveit,
                GRIPPER_OPEN,
                "OPEN"
            )

        elif choice == "2":

            move_gripper(
                gripper_moveit,
                GRIPPER_CLOSED,
                "CLOSE"
            )

        elif choice == "3":

            print("Skipping gripper operation.")
            return "continue"

        elif choice == "4":

            print("Exiting robot control.")
            return "exit"

        else:

            print("Invalid choice. Select 1, 2, 3 or 4.")


# ============================================================
# MAIN
# ============================================================

def main():

    print("\n")
    print("==========================================")
    print("       SIX-JOINT ROBOT CONTROL")
    print("==========================================")

    print("\nRobot configuration:")
    print("  Arm joints     : 4 revolute")
    print("  Gripper joints : 2 prismatic")

    print("\nControl architecture:")
    print("  Python -> MoveIt 2 -> ROS 2 Control -> Gazebo")

    print("\nReference frame:", BASE_LINK)
    print("Arm end-effector:", ARM_END_EFFECTOR)

    # --------------------------------------------------------
    # Initialize ROS 2
    # --------------------------------------------------------

    rclpy.init()

    node = Node(
        "six_joint_robot_client",

        parameter_overrides=[
            Parameter(
                "use_sim_time",
                value=True
            )
        ]
    )

    arm_moveit = None
    gripper_moveit = None

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    spin_thread = None

    try:

        # ----------------------------------------------------
        # Create ARM MoveIt interface
        # ----------------------------------------------------

        arm_moveit = MoveIt2(

            node=node,

            joint_names=ARM_JOINT_NAMES,

            base_link_name=BASE_LINK,

            end_effector_name=ARM_END_EFFECTOR,

            group_name=ARM_GROUP
        )

        # ----------------------------------------------------
        # Create GRIPPER MoveIt interface
        # ----------------------------------------------------

        gripper_moveit = MoveIt2(

            node=node,

            joint_names=GRIPPER_JOINT_NAMES,

            base_link_name=GRIPPER_BASE_LINK,

            end_effector_name=GRIPPER_REFERENCE_LINK,

            group_name=GRIPPER_GROUP
        )

        # ----------------------------------------------------
        # Start ROS 2 executor
        # ----------------------------------------------------

        executor.add_node(node)

        spin_thread = threading.Thread(

            target=executor.spin,

            daemon=True
        )

        spin_thread.start()

        # ----------------------------------------------------
        # Verify joint state feedback
        # ----------------------------------------------------

        print("\nWaiting for robot joint states...")

        if not arm_moveit.wait_for_joint_state(
            timeout_sec=10.0
        ):

            raise RuntimeError(
                "Arm joint states unavailable. "
                "Check joint_state_broadcaster."
            )

        if not gripper_moveit.wait_for_joint_state(
            timeout_sec=10.0
        ):

            raise RuntimeError(
                "Gripper joint states unavailable. "
                "Check gripper joint definitions and broadcaster."
            )

        print("Robot joint states received.")

        print("\nBoth MoveIt interfaces initialized.")

        # ----------------------------------------------------
        # Main movement loop
        # ----------------------------------------------------

        while True:

            print("\n==========================================")
            print("          NEW XYZ TARGET")
            print("==========================================")

            try:

                x = read_coordinate("X")
                y = read_coordinate("Y")
                z = read_coordinate("Z")

            except KeyboardInterrupt:

                print("\nCoordinate input cancelled.")
                break

            print("\nTarget position:")

            print(f"  X = {x:.4f} m")
            print(f"  Y = {y:.4f} m")
            print(f"  Z = {z:.4f} m")

            if not ask_confirmation(
                "Plan this XYZ target?"
            ):

                print("Target cancelled.")
                continue

            # ------------------------------------------------
            # Arm movement
            # ------------------------------------------------

            arm_success = move_arm_to_xyz(
                arm_moveit,
                x,
                y,
                z
            )

            if not arm_success:

                print("\nArm movement was not completed.")

                if not ask_confirmation(
                    "Try another XYZ target?"
                ):
                    break

                continue

            print("\nRobot reached the requested XYZ position.")

            # ------------------------------------------------
            # Gripper control
            # ------------------------------------------------

            gripper_result = gripper_menu(
                gripper_moveit
            )

            if gripper_result == "exit":
                break

            # ------------------------------------------------
            # Next XYZ target
            # ------------------------------------------------

            if not ask_confirmation(
                "Enter another XYZ target?"
            ):

                print("\nRobot control session completed.")
                break

        return 0

    except KeyboardInterrupt:

        print("\nRobot control interrupted.")

        if arm_moveit is not None:
            arm_moveit.cancel_execution()

        if gripper_moveit is not None:
            gripper_moveit.cancel_execution()

        return 130

    except Exception as exc:

        print(f"\nERROR: {exc}")

        return 1

    finally:

        # ----------------------------------------------------
        # Cleanup
        # ----------------------------------------------------

        print("\nShutting down robot control interface...")

        if arm_moveit is not None:
            arm_moveit.destroy()

        if gripper_moveit is not None:
            gripper_moveit.destroy()

        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        if spin_thread is not None:
            spin_thread.join(timeout=2.0)

        print("ROS 2 shutdown complete.")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    sys.exit(main())
