package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name  = "Basic: No Linear Acuator Movement Test",
        group = "Linear OpMode")

public class LinearAcuator extends LinearOpMode {
        private DcMotor linearActuator = null;
        private ElapsedTime runtime    = new ElapsedTime();
        public void runOpMode() {
            linearActuator = hardwareMap.get(DcMotor.class, "linearActuator");

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.a) {
                    linearActuator.setPower(0.5);
                }
                if (gamepad1.b) {
                    linearActuator.setPower(-0.5);
                }
                linearActuator.setPower(0);
            }

        }
}