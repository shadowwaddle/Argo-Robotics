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

@TeleOp(name = "Old Backup", group = "Linear OpMode")
public class AnOldHope extends LinearOpMode {

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

    private static final double ARM_DECELERATION_THRESHOLD = 400;
    private static final double ARM_MIN_VELOCITY = 200;
    private static final double ARM_MAX_VELOCITY = 2100;

    private static final double DEFAULT_ARM_ANGLE = 14.4; // Degrees
 
    // State variables
    
    private double armPosition = 20 * ARM_TICKS_PER_DEGREE;
    private double liftPosition = LIFT_START;
    private double nodPosition = 0;
    private boolean intakeModeActive = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleChassisControl();
            handleGripControl();
            handleNodControl();
            handleLiftControl();
            handleArmControl();
            updateTelemetry();
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

        setMotorDirections();
        setMotorBehaviors();
        resetEncoders();
    }

    private void setMotorDirections() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setMotorBehaviors() {
        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        leftFrontDrive.setZeroPowerBehavior(brake);
        rightFrontDrive.setZeroPowerBehavior(brake);
        leftBackDrive.setZeroPowerBehavior(brake);
        rightBackDrive.setZeroPowerBehavior(brake);
        armMotor.setZeroPowerBehavior(brake);
    }

    private void resetEncoders() {
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void handleChassisControl() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double leftFrontPower = axial - lateral + yaw;
        double rightFrontPower = axial + lateral - yaw;
        double leftBackPower = axial + lateral + yaw;
        double rightBackPower = axial - lateral - yaw;

        normalizeAndSetMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void normalizeAndSetMotorPower(double... powers) {
        double max = 1.0;
        for (double power : powers) max = Math.max(max, Math.abs(power));

        for (int i = 0; i < powers.length; i++) powers[i] /= max;

        leftFrontDrive.setPower(powers[0]);
        rightFrontDrive.setPower(powers[1]);
        leftBackDrive.setPower(powers[2]);
        rightBackDrive.setPower(powers[3]);
    }

    private void handleGripControl() {
        if (gamepad1.right_bumper) grip.setPosition(0.2); // Open
        if (gamepad1.left_bumper) grip.setPosition(0.74); // Closed
    }

    private void handleNodControl() {
        if (gamepad1.dpad_up) nodPosition = 0.35 * Math.PI;
        if (gamepad1.dpad_right) nodPosition = 0.2 * Math.PI;
        if (gamepad1.dpad_down) nodPosition = 0.05 * Math.PI;

        nodLeft.setPosition(nodPosition);
        nodRight.setPosition(1 - nodPosition);
    }

    private void handleLiftControl() {
        if (gamepad2.a) intakeModeActive = true; // Activate intake mode
        if (gamepad2.x) intakeModeActive = false; // Deactivate intake mode

        if (gamepad2.right_trigger > 0) liftPosition += 15 * gamepad2.right_trigger;
        if (gamepad2.left_trigger > 0) liftPosition -= 15 * gamepad2.left_trigger;

        liftPosition = Math.max(0, liftPosition);

      //  if (intakeModeActive) adjustArmForLift();

        liftMotor.setTargetPosition((int) liftPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    private void handleArmControl() {
    // Turn intake mode ON when `gamepad2.a` is pressed and move the arm to the intake position
    if (gamepad2.a) {
        intakeModeActive = true; // Enable intake mode
        armPosition = 20 * ARM_TICKS_PER_DEGREE; // Move arm to intake position
        liftPosition = 600; // Set lift position for intake
        nodPosition = 0.05 * Math.PI; // Optional: adjust nod position
    }

    // Turn intake mode OFF when `gamepad2.x` is pressed
    if (gamepad2.x) {
        intakeModeActive = false; // Disable intake mode
        armPosition = 20 * ARM_TICKS_PER_DEGREE;
        liftPosition = 0;
        nodPosition = 0.2 * Math.PI;
    }

    // Preset arm positions for other specific inputs
    if (gamepad2.y) {
        armPosition = 130 * ARM_TICKS_PER_DEGREE;
        liftPosition = 2400;
        nodPosition = 0.35 * Math.PI;
    } else if (gamepad2.dpad_down) {
        armPosition = 5 * ARM_TICKS_PER_DEGREE;
        liftPosition = 200;
        nodPosition = 0.23 * Math.PI;
    } else if (gamepad2.dpad_up) {
        armPosition = 91 * ARM_TICKS_PER_DEGREE;
        liftPosition = 0;
        nodPosition = 0.2 * Math.PI;
    } else if (gamepad2.dpad_right) {
        armPosition = 80 * ARM_TICKS_PER_DEGREE;
        liftPosition = 0;
        nodPosition = 0.2 * Math.PI;
    }
    
    // Manual Adjust
    if (gamepad2.left_bumper) armPosition -= 10; // Decrease arm position
    if (gamepad2.right_bumper) armPosition += 10 ; // Increase arm position
    
    // Adjust arm position while intaking (fine control)
    if (intakeModeActive) {
        // Manual control with bumpers
        if (gamepad2.left_bumper) armPosition -= 3; // Decrease arm position
        if (gamepad2.right_bumper) armPosition += 3; // Increase arm position

        // // Self-adjust arm position based on liftPosition, respecting limits
        // if (liftPosition <= 650) {
        //     armPosition = Math.max(armPosition, 12 * ARM_TICKS_PER_DEGREE); // Minimum 12 degrees
        // } else if (liftPosition <= 1000) {
        //     armPosition = Math.min(Math.max(armPosition, 12 * ARM_TICKS_PER_DEGREE), 16 * ARM_TICKS_PER_DEGREE);
        // } else if (liftPosition <= 15000) {
        //     armPosition = Math.min(Math.max(armPosition, 16 * ARM_TICKS_PER_DEGREE), 19 * ARM_TICKS_PER_DEGREE);
        // } else {
        //     armPosition = Math.max(armPosition, 19 * ARM_TICKS_PER_DEGREE); // Maximum 19 degrees
        // }
    }

    // Set the arm motor target position
    armMotor.setTargetPosition((int) armPosition);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    // Adjust motor velocity dynamically
    ((DcMotorEx) armMotor).setVelocity(calculateDecelerationVelocity(
            armMotor.getCurrentPosition(),
            armPosition,
            ARM_DECELERATION_THRESHOLD,
            ARM_MIN_VELOCITY,
            ARM_MAX_VELOCITY
    ));
}

    private double calculateDecelerationVelocity(double currentPosition, double targetPosition, double threshold, double minVelocity, double maxVelocity) {
        double distance = Math.abs(targetPosition - currentPosition);
        return distance < threshold
                ? minVelocity + (maxVelocity - minVelocity) * (distance / threshold)
                : maxVelocity;
    }


    private void updateTelemetry() {
        telemetry.addData("Lift Position", liftPosition);
        telemetry.addData("Arm Position", armPosition);
        telemetry.addData("Intake Mode", intakeModeActive);
        telemetry.addData("Arm degrees", armPosition / ARM_TICKS_PER_DEGREE);
        telemetry.update();
    }
}
