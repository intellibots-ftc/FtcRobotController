package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Stage 1 Auto", group="Basic Auto")
public class Stage1Auto extends LinearOpMode {
    AutoDriver robot = new AutoDriver(this);

    @Override
    public void runOpMode(){
        robot.init();
        waitForStart();
        robot.hangNoMotor();
    }
}
