package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonBase", group="Autonomous")
public class AutonBase extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotorEx armMotor;
    private DcMotorEx liftMotor;
    private CRServo intake;
    private Servo wrist;
    private DistanceSensor distanceSensor;

    // Movement Constants
    private static final double TICKS_PER_REVOLUTION = 537.6;
    private static final double WHEEL_DIAMETER_INCHES = 3.77;
    private static final double ROBOT_DIAMETER_INCHES = 16.0;

    private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION) / (Math.PI * WHEEL_DIAMETER_INCHES);
    private static final double TICKS_PER_DEGREE = (ROBOT_DIAMETER_INCHES * Math.PI * TICKS_PER_INCH * 1.82) / 360;
    private static final int MOVEMENT_DELAY_MS = 150;

    // Arm Constants
    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 42.0 / 10.0 / 360.0;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 10 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;

    // Lift Constants
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_START = 50 * LIFT_TICKS_PER_MM;
    final double LIFT_INTAKE = 400 * LIFT_TICKS_PER_MM;

    // Arm Deceleration Constants
    final double ARM_DECELERATION_THRESHOLD = 200; // Distance (ticks) to start decelerating
    final double ARM_MIN_VELOCITY = 500; // Minimum velocity (ticks/sec) near the target
    final double ARM_MAX_VELOCITY = 2100; // Maximum velocity (ticks/sec)

    // Lift Deceleration Constants
    final double LIFT_DECELERATION_THRESHOLD = 300; // Distance (ticks) to start decelerating
    final double LIFT_MIN_VELOCITY = 300; // Minimum velocity (ticks/sec) near the target
    final double LIFT_MAX_VELOCITY = 1000; // Maximum velocity (ticks/sec)

    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    double liftPosition = LIFT_START;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftRear = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRear = hardwareMap.get(DcMotor.class, "rightBackDrive");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "viperMotor");
        intake = hardwareMap.get(CRServo.class, "servoGripper");
        wrist = hardwareMap.get(Servo.class, "servoPivot");

        // Configure motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Autonomous Ready.");
        telemetry.update();
        waitForStart();

        // Autonomous sequence
        moveForward(24, 0.5);
        turnRight(90, 0.5);
        setArmAndLiftToIntake();
        moveForward(12, 0.3);
        stopAllMotors();
    }

    // Autonomous movement methods
    private void moveForward(double inches, double speed) {
        executeMovement(inches, speed, 1, 1, 1, 1);
    }

    private void moveBackward(double inches, double speed) {
        executeMovement(inches, speed, -1, -1, -1, -1);
    }

    private void turnLeft(double degrees, double speed) {
        executeMovement(degrees * TICKS_PER_DEGREE, speed, -1, -1, 1, 1);
    }

    private void turnRight(double degrees, double speed) {
        executeMovement(degrees * TICKS_PER_DEGREE, speed, 1, 1, -1, -1);
    }

    private void executeMovement(double value, double speed, int lfDir, int lrDir, int rfDir, int rrDir) {
        int ticks = (int) (value * TICKS_PER_INCH);
        setTargetPosition(leftFront, lfDir * ticks);
        setTargetPosition(leftRear, lrDir * ticks);
        setTargetPosition(rightFront, rfDir * ticks);
        setTargetPosition(rightRear, rrDir * ticks);
        decelerateMotors(ticks, speed);
        delayAfterMovement();
    }

    private void setTargetPosition(DcMotor motor, int position) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void decelerateMotors(int targetPosition, double maxSpeed) {
        while (opModeIsActive() &&
                (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy())) {
            double currentPosition = Math.abs(leftFront.getCurrentPosition());
            double distanceToTarget = Math.abs(targetPosition - currentPosition);
            double deceleratedSpeed = calculateDeceleration(distanceToTarget, ARM_DECELERATION_THRESHOLD, ARM_MIN_VELOCITY, maxSpeed);
            setMotorPowers(deceleratedSpeed);
        }
        setMotorPowers(0);
    }

    private double calculateDeceleration(double distanceToTarget, double threshold, double minSpeed, double maxSpeed) {
        if (distanceToTarget < threshold) {
            return minSpeed + (maxSpeed - minSpeed) * (distanceToTarget / threshold);
        } else {
            return maxSpeed;
        }
    }

    private void setMotorPowers(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    private void delayAfterMovement() {
        sleep(MOVEMENT_DELAY_MS);
    }

    // Arm and Lift Functions
    private void setArmAndLiftToIntake() {
        armPosition = ARM_COLLECT;
        liftPosition = LIFT_INTAKE;
        armMotor.setTargetPosition((int) armPosition);
        liftMotor.setTargetPosition((int) liftPosition);
        ((DcMotorEx) armMotor).setVelocity(ARM_MAX_VELOCITY);
        ((DcMotorEx) liftMotor).setVelocity(LIFT_MAX_VELOCITY);
        intake.setPower(1.0); // Start intake
    }

    private void stopAllMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        intake.setPower(0);
    }
}