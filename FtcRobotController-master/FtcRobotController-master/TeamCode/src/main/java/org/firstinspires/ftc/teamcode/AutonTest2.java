package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AutonTest2", group="Autonomous")
public class AutonTest2 extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor armMotor;
    private DcMotor liftMotor;
    private DistanceSensor distanceSensor;
    private Servo wrist;
    private CRServo intake;

    private static final double TICKS_PER_REVOLUTION = 537.6; // Example, replace with actual TPR
    private static final double WHEEL_DIAMETER_INCHES = 3.77;
    private static final double ROBOT_DIAMETER_INCHES = 16.0;

    private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION) / (Math.PI * WHEEL_DIAMETER_INCHES);
    private static final double TICKS_PER_DEGREE = (ROBOT_DIAMETER_INCHES * Math.PI * TICKS_PER_INCH * 1.82) / 360;
    private static final int MOVEMENT_DELAY_MS = 150; // 200 ms delay to smooth transitions
    
    static final double DISTANCE_THRESHOLD = 5.0;
    
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

    double liftPosition = 20 * ARM_TICKS_PER_DEGREE;
    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    boolean isIntaking = false;
    
    

    @Override
    public void runOpMode() {
        
        // Encoder Stuff
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftRear = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRear = hardwareMap.get(DcMotor.class, "rightBackDrive");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        liftMotor = hardwareMap.dcMotor.get("viperMotor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
        
        intake = hardwareMap.get(CRServo.class, "servoGripper");
        wrist = hardwareMap.get(Servo.class, "servoPivot");
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        armMotor.setTargetPosition((int) (armPosition));
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the start command
        waitForStart();
        
        moveForward(7,0.5);
        
        turnRight(90, 0.5);
        //moveForward(24, 0.5);    // Move forward 24 inches
        //moveBackward(12, 0.6);
        //strafeRight(10, 0.5);
        //strafeLeft(10, 0.5);
        //turnLeft(90, 0.6);
        
        while (opModeIsActive()) {

            // Read the distance sensor
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance ", distance);
            telemetry.update();

            // Check if the object is within range to trigger motor movement
            
            while (distance > DISTANCE_THRESHOLD) {
                leftFront.setPower(0.5);
                leftRear.setPower(0.5);
                rightRear.setPower(0.5);
                rightFront.setPower(0.5);
            }
        }
    }

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
        int ticks = (int) (inches * TICKS_PER_INCH);

        setTargetPosition(leftFront, -ticks);
        setTargetPosition(leftRear, -ticks);
        setTargetPosition(rightFront, -ticks);
        setTargetPosition(rightRear, -ticks);

        setMotorPowers(speed);
        waitForMotors();
        delayAfterMovement();
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
        int ticks = (int) (degrees * TICKS_PER_DEGREE);

        setTargetPosition(leftFront, ticks);
        setTargetPosition(leftRear, ticks);
        setTargetPosition(rightFront, -ticks);
        setTargetPosition(rightRear, -ticks);

        setMotorPowers(speed);
        waitForMotors();
        delayAfterMovement();
    }

    private void strafeLeft(double inches, double speed) {
        int ticks = (int) (inches * TICKS_PER_INCH);

        setTargetPosition(leftFront, -ticks);
        setTargetPosition(leftRear, ticks);
        setTargetPosition(rightFront, ticks);
        setTargetPosition(rightRear, -ticks);

        setMotorPowers(speed);
        waitForMotors();
        delayAfterMovement();
    }

    private void strafeRight(double inches, double speed) {
        int ticks = (int) (inches * TICKS_PER_INCH);

        setTargetPosition(leftFront, ticks);
        setTargetPosition(leftRear, -ticks);
        setTargetPosition(rightFront, -ticks);
        setTargetPosition(rightRear, ticks);

        setMotorPowers(speed);
        waitForMotors();
        delayAfterMovement();
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
    
    // Delay to prevent choppiness after each movement
    private void delayAfterMovement() {
        sleep(MOVEMENT_DELAY_MS);
    }
}
