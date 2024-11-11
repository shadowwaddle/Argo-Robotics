package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name  = "Viper Slide Test",
        group = "Linear OpMode")

public class ViperSlide extends LinearOpMode {
    DcMotor motor;
    private ElapsedTime runtime     = new ElapsedTime();
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "Viper Slide");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(0.5);
            }
            if (gamepad1.b) {
                motor.setPower(-0.5);
            }
            motor.setPower(0);
        }

    }
}