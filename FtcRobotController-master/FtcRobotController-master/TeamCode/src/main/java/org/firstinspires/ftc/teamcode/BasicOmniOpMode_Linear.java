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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name  = "Basic: Omni Linear OpMode",
        group = "Linear OpMode"
        )
public class BasicOmniOpMode_Linear extends LinearOpMode
{
    // Declare OpMode members for each of the 4 motors.
    // The motors correspond to ports 3, 1, 2, 0 respectively.
    // It's not intuitive and will be changed later.
    private ElapsedTime runtime     = new ElapsedTime();
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;
    private DcMotor linearActuator  = null;
    private DcMotor armMotor        = null;
    private DcMotor viperMotor      = null;


    // Declaring variables for the built-in encoders for each motor.
    // We will be using the pod for getting data, going to need more reading to see
    // if these are even necessary. We might just be directly using goToPosition without a var.
    //private int leftFrontPosition   = null;
    //private int leftBackPosition    = null;
    //private int rightFrontPosition  = null;
    //private int rightBackPosition   = null;

    // Variables for the odometry calculator.
    // oldTime will be used in frequency calculation.
    //private GoBildaPinpointDriver odometryCalc;
    //private double oldTime = 0.0;
    @Override
    public void runOpMode()
    {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBackDrive");
        //odometryCalc    = hardwareMap.get(GoBildaPinpointDriver.class, "odometryCalc");
        linearActuator  = hardwareMap.get(DcMotor.class, "linearActuator");
        armMotor        = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor      = hardwareMap.get(DcMotor.class, "viperMotor");


        // Setting direction, since some of the motors are attached backwards
        // and need to be reversed.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Since the odometry pods are not at the center of the robot, we need to include the offset.
        // We will need to see how many units offset it is. For now, we use placeholders.
        // More information on the GitHub link attached on the push message.
        //odometryCalc.setOffsets( num1 , num2 );

        // Setting encoder res and directions
        // FORWARD means FRONT for front-back, and LEFT for left-right.
        //odometryCalc.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odometryCalc.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
        //                                  GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Recalibrates the odometry and resets the position.
        //odometryCalc.recalibrateIMU();
        //odometryCalc.resetPosAndIMU();

        // Wait for the game to start (driver presses START), and updates telemetry
        telemetry.addData("Status", "Initialized");
        //telemetry.addData("X offset", odo.getXOffset());
        //telemetry.addData("Y offset", odo.getYOffset());
        //telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        //telemetry.addData("Device Scalar", odo.getYawScalar());
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
            double viperInput = -gamepad2.right_stick_y;


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

            //Linear Actuator code
            if (gamepad1.a) {
                linearActuator.setPower(0.5);
            }
            if (gamepad1.b) {
                linearActuator.setPower(-0.5);
            }


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            armMotor.setPower(armInput / 1.0);
            viperMotor.setPower(viperInput / 0.1);
            linearActuator.setPower(0);


            // Code to get frequency.
            //double newTime = getRuntime();
            //double loopTime = newTime - oldTime;
            //double frequency = 1.0 / loopTime;
            //oldTime = newTime;

            // Calculations for telemetry
            //Pose2D pos = odometryCalc.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //Pose2D vel = odometryCalc.getVelocity();
            //String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));

            // Updating telemetry on DS.
            telemetry.addData("Status: ", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right: ", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right: ", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("Position: ", data);
            //telemetry.addData("Velocity: ", velocity);
            //telemetry.addData("Status: ", odo.getDeviceStatus());
            //telemetry.addData("Pinpoint Frequency: ", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            //telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();
        }
    }
}
