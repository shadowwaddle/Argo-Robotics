package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "OneHighBasket", group = "Linear OpMode")
public class OneHighBasket extends LinearOpMode {

    // Hardware components
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor armMotor, liftMotor;
    private Servo grip, nodRight, nodLeft;

    // Constants
    private static final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 42.0 / 10.0 / 360.0;
    private static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    private static final double ARM_HIGH_BASKET = 120 * ARM_TICKS_PER_DEGREE;
    private static final double ARM_LOW_BASKET = 120 * ARM_TICKS_PER_DEGREE;
    private static final double ARM_SPECIMEN = 60 * ARM_TICKS_PER_DEGREE;
    private static final double ARM_COLLAPSED_INTO_ROBOT = 0;
    private static final double ARM_COLLECT = 20 * ARM_TICKS_PER_DEGREE;

    private static final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    private static final double LIFT_START = 0 * LIFT_TICKS_PER_MM;
    private static final double LIFT_INTAKE = 500 * LIFT_TICKS_PER_MM;
    
    // State variables (Moved outside runOpMode)
private double armPosition = 20 * ARM_TICKS_PER_DEGREE;
private double liftPosition = LIFT_START;
private double nodPosition = 0;

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addLine("Autonomous Ready.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
           closeGrip();
           dropOff();
           moveBackward(700, 0.5);
            openGrip();
            sleep(2000);
           moveForward(300, 0.5);
           drive();
           sleep(1000);
           reset();
           // strafe(800,0.6,true);
            //moveForward(500, 0.6); // Move forward for 1000 encoder ticks
        }
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        liftMotor = hardwareMap.dcMotor.get("viperMotor");
        grip = hardwareMap.get(Servo.class, "grip");
        nodRight = hardwareMap.get(Servo.class, "nodRight");
        nodLeft = hardwareMap.get(Servo.class, "nodLeft");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    private void resetEncoders() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveForward(int ticks, double power) {
        setDriveTarget(ticks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(power);
        waitForMotors();
    }

    private void moveBackward(int ticks, double power) {
        setDriveTarget(-ticks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(power);
        waitForMotors();
    }
    
    private void turnLeft(int ticks, double power) {
    leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() - ticks);
    rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + ticks);
    leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - ticks);
    rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + ticks);

    setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
    setDrivePower(power);
    waitForMotors();
}

    private void turnRight(int ticks, double power) {
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + ticks);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - ticks);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + ticks);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() - ticks);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(power);
        waitForMotors();
    }
    
    private void strafe(int ticks, double power, boolean right) {
    // Strafe right: Left wheels move forward, Right wheels move backward
    // Strafe left: Left wheels move backward, Right wheels move forward
    int direction = right ? 1 : -1;

    leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (ticks * direction));
    rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - (ticks * direction));
    leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - (ticks * direction));
    rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (ticks * direction));

    setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
    setDrivePower(power);
    waitForMotors();
    }

    private void setDriveTarget(int ticks) {
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + ticks);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + ticks);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + ticks);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + ticks);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    private void setDrivePower(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void waitForMotors() {
        while (opModeIsActive() &&
                (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            telemetry.addLine("Moving...");
            telemetry.update();
        }
        setDrivePower(0);
    }
    
    private void dropOff() {
    liftPosition = 2200;
    armPosition = 120 * ARM_TICKS_PER_DEGREE;
    nodPosition = 0.35 * Math.PI;

    // Set target positions
    armMotor.setTargetPosition((int) armPosition);
    liftMotor.setTargetPosition((int) liftPosition);

    // Run motors to position
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    armMotor.setPower(1.0);
    liftMotor.setPower(1.0);

    // Move nod servos after both lift and arm movements start
    nodLeft.setPosition(nodPosition);
    nodRight.setPosition(1 - nodPosition);
}

private void drive() {
    armPosition = 20 * ARM_TICKS_PER_DEGREE;
    liftPosition = 0;
    nodPosition = 0.2 * Math.PI;

    // Set target positions
    armMotor.setTargetPosition((int) armPosition);
    liftMotor.setTargetPosition((int) liftPosition);

    // Run motors to position
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    armMotor.setPower(1.0);
    liftMotor.setPower(1.0);

    // Move nod servos after both lift and arm movements start
    nodLeft.setPosition(nodPosition);
    nodRight.setPosition(1 - nodPosition);
}
private void pickUp() {
    armPosition = 5.0 * ARM_TICKS_PER_DEGREE;
    liftPosition = 210;
    nodPosition = 0.19 * Math.PI;

    // Set target positions
    armMotor.setTargetPosition((int) armPosition);
    liftMotor.setTargetPosition((int) liftPosition);

    // Run motors to position
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    armMotor.setPower(1.0);
    liftMotor.setPower(1.0);

    // Move nod servos after both lift and arm movements start
    nodLeft.setPosition(nodPosition);
    nodRight.setPosition(1 - nodPosition);
}
private void reset() {
    armPosition = 0 * ARM_TICKS_PER_DEGREE;
    liftPosition = 0;
    nodPosition = 0.20 * Math.PI;

    // Set target positions
    armMotor.setTargetPosition((int) armPosition);
    liftMotor.setTargetPosition((int) liftPosition);

    // Run motors to position
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    armMotor.setPower(1.0);
    liftMotor.setPower(1.0);

    // Move nod servos after both lift and arm movements start
    nodLeft.setPosition(nodPosition);
    nodRight.setPosition(1 - nodPosition);
}


    private void moveArmToPosition(double position) {
        armMotor.setTargetPosition((int) position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(2100);
        while (opModeIsActive() && armMotor.isBusy()) {
            telemetry.addData("Moving Arm", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void moveLiftToPosition(double position) {
        liftMotor.setTargetPosition((int) position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.75);
        while (opModeIsActive() && liftMotor.isBusy()) {
            telemetry.addData("Moving Lift", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void openGrip() {
        grip.setPosition(0.2);
    }

    private void closeGrip() {
        grip.setPosition(0.64);
    }
}
