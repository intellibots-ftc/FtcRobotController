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
        robot.basketNoMotor();
        robot.drive(0.2);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Wait for 0.8 seconds
        while (timer.seconds() < 0.8) {
            // Do nothing, just wait
        }
        robot.resetDrive();
        robot.intakeServo.setPower(1);
    }
}
