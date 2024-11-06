
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomn.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear OpMode")
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
    private Servo servoPivot = null;
    private CRServo servoGripper = null;

    // Constants for deceleration
    private static final double SLOWDOWN_THRESHOLD = 300; // Distance in ticks to start slowing down
    private static final double MIN_POWER = 0.2; // Minimum power when close to target
    private static final double FULL_POWER = 1.0; // Full power for normal movement

    // Constants for ticks per degree and mm
    final double ARM_TICKS_PER_DEGREE = 28 * (250047.0 / 4913.0) * (42.0 / 10.0) / 360.0;
    final double ARM_COMPENSATION_THRESHOLD = 45 * ARM_TICKS_PER_DEGREE;

    // Positions
    final double ARM_COLLAPSED_POSITION = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_INTAKE_POSITION = 10 * ARM_TICKS_PER_DEGREE;
    final double ARM_HIGH_BASKET_POSITION = 100 * ARM_TICKS_PER_DEGREE;
    final double VIPER_MAX_TICKS = 580;

    double armPosition = ARM_COLLAPSED_POSITION;
    double viperPosition = VIPER_COLLAPSED;

    // Gravity compensation factor
    double armLiftComp = 0;

    // State flags
    private boolean isArmInPresetPosition = false;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;
    
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

        // Set ZeroPowerBehavior to BRAKE for all drivetrain motors
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Encoder initialization for armMotor and viperMotor
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Player 1 Controls (Chassis & Gripper)
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

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

            // Gripper
            if (gamepad1.left_bumper) {
                servoGripper.setPower(-1.0); // Collect
            } else if (gamepad1.right_bumper) {
                servoGripper.setPower(0.0); // Off
            } else if (gamepad1.y) {
                servoGripper.setPower(0.5); // Deposit
            }

            // Player 2 Controls (Arm & Viper)
            if (gamepad2.a) { // Set to intake position
                armPosition = ARM_INTAKE_POSITION;
                isArmInPresetPosition = true;
            } else if (gamepad2.y) { // Set to high basket position
                armPosition = ARM_HIGH_BASKET_POSITION;
                isArmInPresetPosition = true;
            }

            // Apply gravity compensation based on viper extension
            if (armPosition < ARM_COMPENSATION_THRESHOLD) {
                armLiftComp = 0.25568 * viperPosition;
            } else {
                armLiftComp = 0;
            }

            // Control arm with deceleration near endpoints
            if (isArmInPresetPosition) {
                double armPower = calculateDecelerationPower(armMotor.getCurrentPosition(), armPosition);
                armMotor.setTargetPosition((int) (armPosition + armLiftComp));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(armPower);

                if (!armMotor.isBusy()) {
                    isArmInPresetPosition = false;
                    armMotor.setPower(0.0);
                    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            } else {
                double armManualInput = -gamepad2.left_stick_y;
                armMotor.setPower(armManualInput * 0.2);
            }

            // Control viper with fine-tuning if not in preset position
             if (gamepad2.right_bumper){
                viperPosition += 2800 * cycletime;
            }
            else if (gamepad2.left_bumper){
                viperPosition -= 2800 * cycletime;
            }
            /*here we check to see if the lift is trying to go higher than the maximum extension.
           *if it is, we set the variable to the max.
            */
            if (viperPosition > VIPER_MAX_TICKS){
                   liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
            //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
           if (viperPosition < 0){
                   viperPosition = 0;
            }

           viperMotor.setTargetPosition((int) (viperPosition));

           ((DcMotorEx) liftMotor).setVelocity(2100);
           viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            
            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/right power", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/right power", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm Position (ticks)", armMotor.getCurrentPosition());
            telemetry.addData("Viper Position (ticks)", viperMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     * Calculates deceleration power for the arm as it approaches the target position.
     */
    private double calculateDecelerationPower(int currentPosition, double targetPosition) {
        double distanceToTarget = Math.abs(targetPosition - currentPosition);

        if (distanceToTarget < SLOWDOWN_THRESHOLD) {
            // Scale power based on how close we are to the target
            double power = MIN_POWER + (FULL_POWER - MIN_POWER) * (distanceToTarget / SLOWDOWN_THRESHOLD);
            return Math.max(MIN_POWER, power); // Ensure power doesn’t drop below MIN_POWER
        } else {
            // Use full power when not near the target
            return FULL_POWER;
        }
    }
}
