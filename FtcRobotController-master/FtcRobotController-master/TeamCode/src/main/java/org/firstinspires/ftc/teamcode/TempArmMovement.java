package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//Temporary OpMode to test the arm angle motion. Will be moved into the main opmode after testing.
@TeleOp(name  = "Arm Movement Testing",
        group = "Linear OpMode")
public class TempArmMovement extends LinearOpMode {

    //Declaring time counter variable, Arm motor object
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor armMotor    = null;

    @Override
    public void runOpMode() {

        //Mapping the motor
        armMotor  = hardwareMap.get(DcMotor.class, "armMotor");

        //While OpMode is active, checks each button to determine movement direction
        //Holding down the button moves it in each direction.
        while (opModeIsActive()) {

            if (gamepad2.a) {
                armMotor.setPower(0.1);
            } else if (gamepad2.b) {
                armMotor.setPower(-0.1);
            } else {
                armMotor.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());

        }

    }
}