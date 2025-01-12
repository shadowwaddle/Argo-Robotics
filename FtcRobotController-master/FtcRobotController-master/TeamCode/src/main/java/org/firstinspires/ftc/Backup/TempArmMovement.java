package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//Temporary OpMode to test the arm angle motion. Will be moved into the main opmode after testing.
//Currently has code for linear side disabled. Make sure to uncomment those lines
//once the slide is mounted.
@TeleOp(name  = "Arm Movement Testing",
        group = "Linear OpMode")
public class TempArmMovement extends LinearOpMode
{
    //Declaring time counter variable, motor objects
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor armMotor    = null;
    private DcMotor viperMotor  = null;

    @Override
    public void runOpMode()
    {
        //Mapping the motors and creating input variable.
        armMotor   = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");

        //While OpMode is active, checks each button to determine movement direction
        //Holding down the button moves it in each direction.

        waitForStart();

        while (opModeIsActive())
        {
            //Input variable creation
            double armInput   = -gamepad2.left_stick_y;
            double viperMotor = -gamepad2.right_stick_y;

            //Setting power
            armMotor.setPower(armInput / 1.0);
            viperMotor.setPower(viperMotor / 0.1);
            telemetry.addData("Status", "Run Time: " + runTime.toString());
        }
    }
}