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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name  = "Basic: Omni Linear OpMode",
        group = "Linear OpMode"
)
public class BasicOmniOpModeLinear extends LinearOpMode
{
    // Declare OpMode members for each of the 4 motors.
    // The motors correspond to ports 3, 1, 2, 0 respectively.
    // It's not intuitive and will be changed later.
    // The linear actuator, armMotor, and viperMotor are assigned to ports
    // 0, 1, and 2 respectively, on the expansion hub.
    // servoPivot is on port 0 and servoGripper is on port 1 of the Control Hub
    private ElapsedTime runtime       = new ElapsedTime();
    private DcMotorEx leftFrontDrive  = null;
    private DcMotorEx leftBackDrive   = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive  = null;
    private DcMotorEx linearActuator  = null;
    private DcMotorEx armMotor        = null;
    private DcMotorEx viperMotor      = null;
    private CRServo servoPivot        = null;
    private CRServo servoGripper      = null;

    @Override
    public void runOpMode()
    {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        linearActuator  = hardwareMap.get(DcMotorEx.class, "linearActuator");
        armMotor        = hardwareMap.get(DcMotorEx.class, "armMotor");
        viperMotor      = hardwareMap.get(DcMotorEx.class, "viperMotor");
        servoPivot      = hardwareMap.get(CRServo.class, "servoPivot");
        servoGripper    = hardwareMap.get(CRServo.class, "servoGripper");


        // Setting direction, since some of the motors are attached backwards
        // and need to be reversed.
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        // Declaring and initializing the servo position variables.
        double servoPivotPosition   = 0.0;
        double servoGripperPosition = 0.0;

        // Wait for the game to start (driver presses START), and updates telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Max is used later for speed control.
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial      = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral    =  gamepad1.left_stick_x;
            double yaw        =  gamepad1.right_stick_x;
            double armInput   = -gamepad2.left_stick_y;
            double viperInput =  gamepad2.right_stick_y;




            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 0.1)
            {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Linear Actuator code
            if (gamepad1.a) {
                linearActuator.setPower(0.5);
            }
            if (gamepad1.b) {
                linearActuator.setPower(-0.5);
            }

            // Code to power the servos in the intake mechanism
            if (gamepad2.a)
            {
                servoPivotPosition += 0.02;
            }
            else if (gamepad2.y)
            {
                servoPivotPosition -= 0.02;
            }

            if (gamepad2.x)
            {
                servoGripperPosition += 0.02;
            }
            else if (gamepad2.b)
            {
                servoGripperPosition -= 0.02;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Set power+position to the arm, actuator, and servos
            armMotor.setPower(armInput / 1.0);
            viperMotor.setPower(viperInput / 0.1);
            linearActuator.setPower(0);
            //servoGripper.setPosition(servoGripperPosition);
            //servoPivot.setPosition(servoPivotPosition);


            // Updating telemetry on DS.
            telemetry.addData("Status: ", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right: ", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right: ", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}
