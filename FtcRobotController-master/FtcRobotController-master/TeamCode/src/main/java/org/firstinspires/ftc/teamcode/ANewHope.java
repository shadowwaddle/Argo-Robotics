package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="A New Hope", group="Linear OpMode")
public class Ri3d extends LinearOpMode {

    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor armMotor = null;
    public DcMotor liftMotor = null;
    public CRServo intake = null;
    public Servo wrist = null;

    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 42.0 / 10.0 / 360.0;
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
     final double ARM_COLLECT               = 10 * ARM_TICKS_PER_DEGREE;
     final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
     final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
     final double ARM_SCORE_SAMPLE_IN_LOW   = 90 * ARM_TICKS_PER_DEGREE;
     final double ARM_ATTACH_HANGING_HOOK   = 110 * ARM_TICKS_PER_DEGREE;
     final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;

    final double INTAKE_COLLECT = 1.5; // Increased intake speed
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = -0.5;

    final double WRIST_FOLDED_IN = 1.0;
    final double WRIST_FOLDED_OUT = 0.7;

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_START = 50 * LIFT_TICKS_PER_MM;
    final double LIFT_INTAKE = 400 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 500 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_START;
    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    boolean isIntaking = false;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        liftMotor = hardwareMap.dcMotor.get("viperMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake = hardwareMap.get(CRServo.class, "servoGripper");
        wrist = hardwareMap.get(Servo.class, "servoPivot");
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Player 1 Controls (Chassis & Gripper)
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = (axial - lateral + yaw);
            double rightFrontPower = (axial + lateral - yaw);
            double leftBackPower = (axial + lateral + yaw);
            double rightBackPower = (axial - lateral - yaw);

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if (gamepad1.left_bumper) {
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad1.right_bumper) {
                intake.setPower(INTAKE_OFF);
            } else if (gamepad1.y) {
                intake.setPower(INTAKE_DEPOSIT);
            }

            // Wrist Position
            if (gamepad1.dpad_left) {
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_right) {
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

            // Arm Preset Positions
            if (gamepad2.a) {
                armPosition = ARM_COLLECT;
                isIntaking = true;
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad2.y) {
                armPosition = ARM_ATTACH_HANGING_HOOK;
                isIntaking = false;
            } else if (gamepad2.x) {
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
                isIntaking = false;
            } else if (gamepad2.dpad_left) {
                /* This turns off the intake, folds in the wrist, and moves the arm
                back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                //liftPosition = LIFT_COLLAPSED;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
        }

            // Fine Adjustments for Arm Position using Left Stick Y on gamepad2
            armPosition += gamepad2.left_stick_y * 10; // Adjust the value to control the sensitivity

            // Apply Arm Position
            armMotor.setTargetPosition((int) (armPosition));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Lift Control with Trigger Adjustments
            if (gamepad2.right_trigger > 0) {
                liftPosition += (50 * gamepad2.right_trigger); // Extend
            } else if (gamepad2.left_trigger > 0) {
                liftPosition -= (50 * gamepad2.left_trigger); // Retract
            }

            // Enforce lift boundaries
            if (isIntaking) {
                if (liftPosition > LIFT_INTAKE) {
                    liftPosition = LIFT_INTAKE;
                } else if (liftPosition < LIFT_START) {
                    liftPosition = LIFT_START;
                }
            } else {
                if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET) {
                    liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
                } else if (liftPosition < LIFT_START) {
                    liftPosition = LIFT_START;
                }
            }

            liftMotor.setTargetPosition((int) liftPosition);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.7);

            // Telemetry Data
            telemetry.addData("Lift Current Position", liftMotor.getCurrentPosition());
            telemetry.addData("Lift Target Position", liftPosition);
            telemetry.addData("Arm Target Position", armMotor.getTargetPosition());
            telemetry.addData("Arm Encoder", armMotor.getCurrentPosition());
            telemetry.update();

            // Check for Arm Overcurrent
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }
        }
    }
}