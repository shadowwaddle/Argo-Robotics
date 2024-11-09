package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;


public class AutonTest extends LinearOpMode {

    public void runOpMode(){
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");

        double distanceLeft = leftDistanceSensor.getDistance(DistanceUnit.CM);
        double distanceRight = rightDistanceSensor.getDistance(DistanceUnit.CM);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory test = drive.trajectoryBuilder(new Poseed())
            if(distanceLeft > distanceRight) {
                .strafeRight(10)
            } else {
                .strafeLeft(10)
            }
            .forward(5)
            .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);


    }

}