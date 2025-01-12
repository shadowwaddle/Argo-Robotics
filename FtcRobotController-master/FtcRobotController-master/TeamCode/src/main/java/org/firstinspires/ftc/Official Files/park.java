package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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


    // Movement Constants
    private static final double TICKS_PER_REVOLUTION = 537.6; // Example, replace with actual TPR
    private static final double WHEEL_DIAMETER_INCHES = 3.77;
    private static final double ROBOT_DIAMETER_INCHES = 16.0;

    private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION) / (Math.PI * WHEEL_DIAMETER_INCHES);
    private static final double TICKS_PER_DEGREE = (ROBOT_DIAMETER_INCHES * Math.PI * TICKS_PER_INCH * 1.1) / 360;
    private static final int MOVEMENT_DELAY_MS = 150; // 200 ms delay to smooth transitions
    
    static final double DISTANCE_THRESHOLD = 5.0;

    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 42.0 / 10.0 / 360.0;
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 6 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_START                 = 20 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;
    
    // Wrist Constants
    final double WRIST_FOLDED_IN = 1.0;
    final double WRIST_FOLDED_OUT = 0.7;

    // Lift Constants
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SHORT = 90 * LIFT_TICKS_PER_MM;
    final double LIFT_INTAKE = 400 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 500 * LIFT_TICKS_PER_MM;

    // Arm Deceleration Constants
    final double ARM_DECELERATION_THRESHOLD = 200;
    final double ARM_MIN_VELOCITY = 500;
    final double ARM_MAX_VELOCITY = 2100;

    // Lift Deceleration Constants
    final double LIFT_DECELERATION_THRESHOLD = 300;
    final double LIFT_MIN_VELOCITY = 300;
    final double LIFT_MAX_VELOCITY = 1000;

    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    double liftPosition = LIFT_COLLAPSED;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftRear   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRear  = hardwareMap.get(DcMotor.class, "rightBackDrive");
        armMotor   = hardwareMap.get(DcMotorEx.class, "armMotor");
        liftMotor  = hardwareMap.get(DcMotorEx.class, "viperMotor");
        intake     = hardwareMap.get(CRServo.class, "servoGripper");
        wrist      = hardwareMap.get(Servo.class, "servoPivot");

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
        
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Autonomous Ready.");
        telemetry.update();

        waitForStart();

        // Initial setup: arm to collect, lift to intake
        setArmAndLiftToStart();

        // Autonomous sequence example
        moveForward(30, 0.5);
        turnLeft(90, 0.5);
        moveBackward(3, 0.5);
        
        setArmAndLiftToIntake();
        //waitForMotors();
        moveForward(7, 0.5);
        sleep(4000);
        setArmToDrop();
        
    
    }
    
    //Sleep function
    private void sleep(double time) {
        try {
            Thread.sleep((int) time);
        } catch (InterruptedException e) {
            return;
        }
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
    
    // Deceleration

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

    // Arm and Lift Functions
    private void setArmAndLiftToStart() {
        wrist.setPosition(WRIST_FOLDED_OUT);
        armPosition = ARM_START;
        liftPosition = LIFT_SHORT;

        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        liftMotor.setTargetPosition((int) liftPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
    }
    private void setArmAndLiftToIntake() {
        wrist.setPosition(WRIST_FOLDED_OUT);
        armPosition = ARM_COLLECT;
        liftPosition = LIFT_SHORT;
        

        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        liftMotor.setTargetPosition((int) liftPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);

        intake.setPower(1.0); // Start intake
    }
    
    private void setArmToDrop() {
        armPosition = (ARM_ATTACH_HANGING_HOOK);
        
        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
    }
    
    private void setSlideOut() {
        liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
        
        liftMotor.setTargetPosition((int) liftPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
    }
    
    
    
    

    private void stopAllMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        intake.setPower(0);
    }
}