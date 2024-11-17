package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

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

    // Movement Constants
    private static final double TICKS_PER_REVOLUTION = 537.6;
    private static final double WHEEL_DIAMETER_INCHES = 3.77;
    private static final double ROBOT_DIAMETER_INCHES = 16.0;

    private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION) / (Math.PI * WHEEL_DIAMETER_INCHES);
    private static final double TICKS_PER_DEGREE = (ROBOT_DIAMETER_INCHES * Math.PI * TICKS_PER_INCH * 1.1) / 360;
    private static final int MOVEMENT_DELAY_MS = 300;

    // Arm Constants
    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 42.0 / 10.0 / 360.0;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 6 * ARM_TICKS_PER_DEGREE;
    final double ARM_START = 20 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_HIGH_BASKET = 100 * ARM_TICKS_PER_DEGREE;

    // Wrist Constants
    final double WRIST_FOLDED_IN = 1.0;
    final double WRIST_FOLDED_OUT = 0.7;

    // Lift Constants
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SHORT = 90 * LIFT_TICKS_PER_MM;
    final double LIFT_INTAKE = 400 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 500 * LIFT_TICKS_PER_MM;

    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    double liftPosition = LIFT_COLLAPSED;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        telemetry.addLine("Autonomous Ready.");
        telemetry.update();

        waitForStart();

        // Initial setup: arm to collect, lift to intake
        setArmAndLiftToStart();

        // Autonomous sequence example
        setArmToDrop();
        moveForward(20, 0.5);
        sleep(10000);
        
    }

    private void initializeHardware() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftRear   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRear  = hardwareMap.get(DcMotor.class, "rightBackDrive");
        armMotor   = hardwareMap.get(DcMotorEx.class, "armMotor");
        liftMotor  = hardwareMap.get(DcMotorEx.class, "viperMotor");
        intake     = hardwareMap.get(CRServo.class, "servoGripper");
        wrist      = hardwareMap.get(Servo.class, "servoPivot");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Autonomous movement methods
    private void moveForward(double inches, double speed) {
        int ticks = (int) (inches * TICKS_PER_INCH);
        setTargetPosition(leftFront, ticks);
        setTargetPosition(leftRear, ticks);
        setTargetPosition(rightFront, ticks);
        setTargetPosition(rightRear, ticks);
        setMotorPowers(speed);
        waitForMotors();
        delayAfterMovement();
    }

    private void moveBackward(double inches, double speed) {
        moveForward(-inches, speed);
    }

    private void turnLeft(double degrees, double speed) {
        int ticks = (int) (degrees * TICKS_PER_DEGREE);
        setTargetPosition(leftFront, -ticks);
        setTargetPosition(leftRear, -ticks);
        setTargetPosition(rightFront, ticks);
        setTargetPosition(rightRear, ticks);
        setMotorPowers(speed);
        waitForMotors();
        delayAfterMovement();
    }

    private void turnRight(double degrees, double speed) {
        turnLeft(-degrees, speed);
    }

    private void setTargetPosition(DcMotor motor, int position) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setMotorPowers(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    private void waitForMotors() {
        while (opModeIsActive() && 
              (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy())) {
            telemetry.addData("Status", "Motors Moving");
            telemetry.update();
        }
        setMotorPowers(0);
    }

    private void delayAfterMovement() {
        sleep(MOVEMENT_DELAY_MS);
    }

    // Arm and Lift Functions
    private void setArmAndLiftToStart() {
        wrist.setPosition(WRIST_FOLDED_OUT);
        armPosition = ARM_START;
        liftPosition = LIFT_SHORT;
        updateArmAndLift();
    }

    private void setArmAndLiftToIntake() {
        wrist.setPosition(WRIST_FOLDED_OUT);
        armPosition = ARM_COLLECT;
        liftPosition = LIFT_SHORT;
        updateArmAndLift();
        intake.setPower(1.0);
    }

    private void setArmToDrop() {
        armPosition = ARM_HIGH_BASKET;
        liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
        updateArmAndLift();
    }

    private void updateArmAndLift() {
        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        liftMotor.setTargetPosition((int) liftPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
    }

    private void stopAllMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        armMotor.setPower(0);
        liftMotor.setPower(0);
        intake.setPower(0);
    }
}