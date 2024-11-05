package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Basic: Omni Linear OpMode with PD Control", group = "Linear OpMode")
public class BasicOmniOpModeLinear extends LinearOpMode {

    // Timer to track runtime
    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain motors
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    // Additional motors and servos
    private DcMotorEx armMotor = null;
    private DcMotorEx viperMotor = null;
    private CRServo servoPivot = null;
    private CRServo servoGripper = null;

    // Constants for target positions, PD control, and gravity compensation
    private static final int ARM_POSITION_REST = 100; // 0 Degrees
    private static final int ARM_POSITION_DROP = -1700; // Target in ticks
    private static final double GRAVITY_COMPENSATION_POWER = 0.3; // Adjust to counteract gravity
    private static final int DEAD_ZONE = 20; // Tolerance for reaching the target without oscillation
    private static final double KP = 0.0015; // Proportional gain for PD control
    private static final double KD = 0.002; // Derivative gain for PD control
    private static final double MAX_POWER = 0.5; // Maximum power for arm motor to control speed

    // State flags
    private boolean isArmInPresetPosition = false;
    private int targetPosition = ARM_POSITION_REST; // Initialize with rest position
    private int lastError = 0; // For calculating derivative

    @Override
    public void runOpMode() {

        // Initialize hardware
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotorEx.class, "viperMotor");
        servoPivot = hardwareMap.get(CRServo.class, "servoPivot");
        servoGripper = hardwareMap.get(CRServo.class, "servoGripper");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for start
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Main loop
        while (opModeIsActive()) {

            // Drive controls (Player 1)
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Calculate power for each wheel
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

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

            // Arm Control with preset positions (Player 2)
            if (gamepad2.x && !isArmInPresetPosition) {
                targetPosition = ARM_POSITION_DROP; // Set target to drop position
                isArmInPresetPosition = true;
            } else if (gamepad2.y && !isArmInPresetPosition) {
                targetPosition = ARM_POSITION_REST; // Set target to rest position
                isArmInPresetPosition = true;
            }

            if (isArmInPresetPosition) {
                moveToPositionWithPDControl(targetPosition);
                if (Math.abs(armMotor.getCurrentPosition() - targetPosition) <= DEAD_ZONE) {
                    isArmInPresetPosition = false;
                    armMotor.setPower(GRAVITY_COMPENSATION_POWER); // Apply holding power against gravity
                    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            } else {
                // Allow manual fine-tuning with joystick if no preset position is active
                double armManualInput = -gamepad2.left_stick_y;
                armMotor.setPower(armManualInput * 0.2);
            }

            // Viper Slide Control (using manual controls for now)
            double viperInput = gamepad2.right_stick_y;
            viperMotor.setPower(viperInput / 0.2);

            // Servo controls
            if (gamepad2.left_bumper) {
                servoPivot.setPower(-1.0);
            } else if (gamepad2.right_bumper) {
                servoPivot.setPower(1.0);
            } else {
                servoPivot.setPower(0.0);
            }

            if (gamepad2.left_trigger > 0.5) {
                servoGripper.setPower(1.0);
            } else if (gamepad2.right_trigger > 0.5) {
                servoGripper.setPower(-1.0);
            } else {
                servoGripper.setPower(0.0);
            }

            // Telemetry updates
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/right power", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/right power", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm Position (ticks)", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     * Moves the arm motor towards the target position using PD control with gravity compensation.
     */
    private void moveToPositionWithPDControl(int targetPosition) {
        int currentPosition = armMotor.getCurrentPosition();
        int error = targetPosition - currentPosition;
        int derivative = error - lastError;

        // Calculate power using PD control
        double power = KP * error + KD * derivative;

        // Apply the max power limit
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        // Apply gravity compensation if needed
        if (targetPosition < ARM_POSITION_REST && currentPosition > ARM_POSITION_DROP) {
            power += GRAVITY_COMPENSATION_POWER;
        }

        armMotor.setPower(power);
        lastError = error; // Update last error for derivative calculation
    }
}