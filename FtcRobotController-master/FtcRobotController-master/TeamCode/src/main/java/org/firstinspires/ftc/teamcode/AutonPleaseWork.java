package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutonTest2", group="Autonomous")
public class AutonTest2 extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotorEx armMotor;
    private DcMotorEx liftMotor;
    private CRServo intake;
    private Servo wrist;
    private DistanceSensor distanceSensor;

    // Constants
    private static final double TICKS_PER_REVOLUTION = 537.6;
    private static final double WHEEL_DIAMETER_INCHES = 3.77;
    private static final double ROBOT_DIAMETER_INCHES = 16.0;

    private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION) / (Math.PI * WHEEL_DIAMETER_INCHES);
    private static final double TICKS_PER_DEGREE = (ROBOT_DIAMETER_INCHES * Math.PI * TICKS_PER_INCH * 1.82) / 360;
    private static final int MOVEMENT_DELAY_MS = 150;

    static final double DISTANCE_THRESHOLD = 5.0;

    // Arm and Lift Constants
    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 42.0 / 10.0 / 360.0;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 10 * ARM_TICKS_PER_DEGREE;
    final double ARM_FLAT = 30 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_START = 50 * LIFT_TICKS_PER_MM;

    final double DECELERATION_THRESHOLD = 200; // Ticks to start deceleration
    final double MIN_VELOCITY = 0.2; // Minimum motor power near target
    final double MAX_VELOCITY = 0.8; // Maximum motor power

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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");

        // Configure motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for start
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        waitForStart();

        // Set initial slide, intake, and arm positions
        armMotor.setPosition(ARM_FLAT);
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        // Autonomous movements
        moveForward(24, 0.5); // Move forward 24 inches
        turnRight(90, 0.5); // Turn right 90 degrees

        while (opModeIsActive()) {
            // Read and display distance sensor
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
    }

    private void moveForward(double inches, double speed) {
        executeMovement(inches, speed, 1, 1, 1, 1);
    }

    private void turnRight(double degrees, double speed) {
        int ticks = (int) (degrees * TICKS_PER_DEGREE);
        executeMovement(ticks, speed, 1, 1, -1, -1);
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
            double deceleratedSpeed = calculateDeceleration(distanceToTarget, DECELERATION_THRESHOLD, MIN_VELOCITY, maxSpeed);

            setMotorPowers(deceleratedSpeed);

            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.addData("Speed", deceleratedSpeed);
            telemetry.update();
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
}