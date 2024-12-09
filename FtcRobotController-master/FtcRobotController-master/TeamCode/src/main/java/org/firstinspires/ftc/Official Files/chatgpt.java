
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
    final double LIFT_SCORING_IN_HIGH_BASKET = 500 * LIFT_TICKS_PER_MM;

    // Arm Deceleration Constants
    final double ARM_DECELERATION_THRESHOLD = 400; // Distance (ticks) to start decelerating
    final double ARM_MIN_VELOCITY = 200; // Minimum velocity (ticks/sec) near the target
    final double ARM_MAX_VELOCITY = 2100; // Maximum velocity (ticks/sec)

    final double ARM_JOINT_HEIGHT = 300; // Example: Height of arm joint from ground in mm
    final double LIFT_LENGTH = 500; // Example: Length of lift extension in mm
    final double GROUND_OFFSET = 50; // Example: Desired intake height from the ground in mm

    // Lift Deceleration Constants (if needed)
    final double LIFT_DECELERATION_THRESHOLD = 300; // Distance (ticks) to start decelerating
    final double LIFT_MIN_VELOCITY = 300; // Minimum velocity (ticks/sec) near the target
    final double LIFT_MAX_VELOCITY = 1000; // Maximum velocity (ticks/sec)

    double liftPosition = LIFT_START;
    double armPosition = ARM_COLLECT;
    boolean isIntaking = false;

    @Override
    public void runOpMode() {
        initializeHardware();

        wrist.setPosition(WRIST_FOLDED_OUT);
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            controlChassis();
            controlIntake();
            controlWrist();
            controlArmAndLift();
            telemetry.update();
        }
    }

    private void initializeHardware() {
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

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
    }

    private void controlChassis() {
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
    }

    private void controlIntake() {
        if (gamepad1.left_bumper) {
            intake.setPower(INTAKE_COLLECT);
        } else if (gamepad1.right_bumper) {
            intake.setPower(INTAKE_DEPOSIT);
        } else if (gamepad1.y) {
            intake.setPower(INTAKE_OFF);
        }
    }

    private void controlWrist() {
        if (gamepad1.dpad_left) {
            wrist.setPosition(WRIST_FOLDED_IN);
        } else if (gamepad1.dpad_right) {
            wrist.setPosition(WRIST_FOLDED_OUT);
        }
    }

    private void controlArmAndLift() {
        if (isIntaking) {
            // Calculate arm position dynamically
            double cosTheta = (ARM_JOINT_HEIGHT - GROUND_OFFSET) /
                    (LIFT_LENGTH + liftPosition / LIFT_TICKS_PER_MM);
            cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));
            double angleRadians = Math.acos(cosTheta);
            armPosition = Math.toDegrees(angleRadians) * ARM_TICKS_PER_DEGREE;
        }

        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        liftMotor.setTargetPosition((int) liftPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);

        telemetry.addData("Arm Position (ticks)", armPosition);
        telemetry.addData("Lift Position (ticks)", liftPosition);
    }
}