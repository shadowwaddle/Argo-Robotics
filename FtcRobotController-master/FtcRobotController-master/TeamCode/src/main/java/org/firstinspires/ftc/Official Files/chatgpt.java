

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "A New Hope", group = "Linear OpMode")
public class ANewHope extends LinearOpMode {

    // Hardware
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx armMotor = null;
    private DcMotorEx liftMotor = null;
    private CRServo intake = null;
    private Servo wrist = null;

    // Constants
    private static final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 42.0 / 10.0 / 360.0;
    private static final double ARM_COLLAPSED_INTO_ROBOT = 0;
    private static final double ARM_COLLECT = 10 * ARM_TICKS_PER_DEGREE;
    private static final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;

    private static final double INTAKE_COLLECT = 1.5;
    private static final double INTAKE_OFF = 0.0;
    private static final double INTAKE_DEPOSIT = -0.5;

    private static final double WRIST_FOLDED_IN = 1.0;
    private static final double WRIST_FOLDED_OUT = 0.7;

    private static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    private static final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    private static final double LIFT_SCORING_IN_HIGH_BASKET = 500 * LIFT_TICKS_PER_MM;

    // Deceleration Constants
    private static final double ARM_DECELERATION_THRESHOLD = 400;
    private static final double ARM_MIN_VELOCITY = 200;
    private static final double ARM_MAX_VELOCITY = 2100;

    private static final double LIFT_DECELERATION_THRESHOLD = 300;
    private static final double LIFT_MIN_VELOCITY = 300;
    private static final double LIFT_MAX_VELOCITY = 1000;

    // Variables
    private double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    private double liftPosition = LIFT_COLLAPSED;
    private boolean isIntaking = false;

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
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "viperMotor");
        intake = hardwareMap.get(CRServo.class, "servoGripper");
        wrist = hardwareMap.get(Servo.class, "servoPivot");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        } else {
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
        if (gamepad2.a) {
            armPosition = ARM_COLLECT;
            isIntaking = true;
            wrist.setPosition(WRIST_FOLDED_OUT);
            intake.setPower(INTAKE_COLLECT);
        } else if (gamepad2.y) {
            armPosition = ARM_SCORE_SPECIMEN;
            isIntaking = false;
            wrist.setPosition(WRIST_FOLDED_IN);
        } else if (gamepad2.dpad_up) {
            liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
        } else if (gamepad2.dpad_down) {
            liftPosition = LIFT_COLLAPSED;
        }

        enforceLiftBoundaries();
        updateArmAndLift();
    }

    private void enforceLiftBoundaries() {
        if (isIntaking) {
            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET) {
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
        } else {
            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET) {
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            } else if (liftPosition < LIFT_COLLAPSED) {
                liftPosition = LIFT_COLLAPSED;
            }
        }
    }

    private void updateArmAndLift() {
        // Deceleration for Arm
        double armVelocity = calculateDecelerationVelocity(
            armMotor.getCurrentPosition(),
            armPosition,
            ARM_DECELERATION_THRESHOLD,
            ARM_MIN_VELOCITY,
            ARM_MAX_VELOCITY
        );

        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(armVelocity);

        // Deceleration for Lift
        double liftVelocity = calculateDecelerationVelocity(
            liftMotor.getCurrentPosition(),
            liftPosition,
            LIFT_DECELERATION_THRESHOLD,
            LIFT_MIN_VELOCITY,
            LIFT_MAX_VELOCITY
        );

        liftMotor.setTargetPosition((int) liftPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setVelocity(liftVelocity);
    }

    private double calculateDecelerationVelocity(double currentPosition, double targetPosition, double decelerationThreshold, double minVelocity, double maxVelocity) {
        double distanceToTarget = Math.abs(targetPosition - currentPosition);
        if (distanceToTarget < decelerationThreshold) {
            return minVelocity + (maxVelocity - minVelocity) * (distanceToTarget / decelerationThreshold);
        } else {
            return maxVelocity;
        }
    }
}

