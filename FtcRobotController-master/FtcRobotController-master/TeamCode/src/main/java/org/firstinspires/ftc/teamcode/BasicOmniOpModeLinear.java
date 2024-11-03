/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear OpMode")
public class BasicOmniOpModeLinear extends LinearOpMode {

    // Timer to track runtime
    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain motors
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    // Additional motors and servos
    private DcMotorEx armMotor = null;
    private DcMotorEx viperMotor = null;
    private CRServo servoPivot = null;
    private CRServo servoGripper = null;

    // Flag to check if arm is moving to a preset position
    private boolean isArmInPresetPosition = false;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables with names that match the robot configuration
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotorEx.class, "viperMotor");
        servoPivot = hardwareMap.get(CRServo.class, "servoPivot");
        servoGripper = hardwareMap.get(CRServo.class, "servoGripper");

        // Reverse direction for left motors to ensure correct movement
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        // Set ZeroPowerBehavior to BRAKE for all drivetrain motors
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Encoder initialization for armMotor and viperMotor
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Constants for arm and viper positions
        int ARM_POSITION_REST = 0;         // 0 Degrees
        int ARM_POSITION_DROP = -1800;     // THIS NUMBER IS RANDOM, TRIAL AND ERROR
        int VIPER_REST = 0;
        int VIPER_MAX_EXTENSION = 0;   // PLEASE FIX THIS VALUE

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Player 1 Controls (Chassis)
            double axial = -gamepad1.left_stick_y;     // Forward and backward
            double lateral = gamepad1.left_stick_x;    // Strafing left and right
            double yaw = gamepad1.right_stick_x;       // Rotation

            // Calculate power for each wheel based on movement control inputs
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the power values to ensure none exceed 100%
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Set calculated power to each wheel motor
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Arm Control with preset positions
            if (gamepad2.x && !isArmInPresetPosition) {
                // Move to drop position
                armMotor.setTargetPosition(ARM_POSITION_DROP);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1); // Adjust power as needed
                isArmInPresetPosition = true;
            } else if (gamepad2.y && !isArmInPresetPosition) {
                // Move to rest position
                armMotor.setTargetPosition(ARM_POSITION_REST);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1); // Adjust power as needed
                isArmInPresetPosition = true;
            }

            // If the arm has reached its target, reset the flag and stop the motor
            if (!armMotor.isBusy() && isArmInPresetPosition) {
                isArmInPresetPosition = false;
                armMotor.setPower(0);  // Stop the motor when it reaches the position
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Switch back to manual control
            }

            // Allow fine-tuning adjustments with the left stick if no preset position is being targeted
            if (!isArmInPresetPosition) {
                double armManualInput = -gamepad2.left_stick_y;  // Fine-tuning adjustments
                armMotor.setPower(armManualInput * 0.1);  // Scale power for finer control
            }

            // Viper Slide Control
            if (gamepad2.a) {
                // Retract slide
                viperMotor.setTargetPosition(VIPER_REST);
                viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperMotor.setPower(1.0); // Set power as needed
            } else if (gamepad2.b) {
                // Extend slide
                viperMotor.setTargetPosition(VIPER_MAX_EXTENSION);
                viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperMotor.setPower(1.0); // Set power as needed
            }

            // Servo controls for pivoting and gripping mechanism
            if (gamepad2.left_bumper) {
                servoPivot.setPower(-1.0);    // Move pivot in one direction
            } else if (gamepad2.right_bumper) {
                servoPivot.setPower(1.0);     // Move pivot in the other direction
            } else {
                servoPivot.setPower(0.0);     // Stop the pivot servo
            }

            // Control the gripper servo with Gamepad 2 left and right triggers
            if (gamepad2.left_trigger > 0.5) {
                servoGripper.setPower(1.0);   // Close the gripper
            } else if (gamepad2.right_trigger > 0.5) {
                servoGripper.setPower(-1.0);  // Open the gripper
            } else {
                servoGripper.setPower(0.0);   // Stop the gripper servo
            }

            // Display telemetry data for debugging and status updates
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/right power", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/right power", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm Position (ticks)", armMotor.getCurrentPosition());
            telemetry.addData("Viper Slide Position (ticks)", viperMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
