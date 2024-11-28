package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Stage 2 Auto", group="Basic Auto")
public class Stage2Auto extends LinearOpMode {
    AutoDriver robot = new AutoDriver(this);

    @Override
    public void runOpMode(){
        robot.init();
        waitForStart();
        robot.basketNoMotor();
        robot.drive(0.2);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        robot.positionServo();

        // Wait for 0.8 seconds
        while (timer.seconds() < 3.15) {
            // Do nothing, just wait
        }
        robot.resetDrive();
        robot.runServo(1);
        timer.reset();
        while (timer.seconds()<3){

        }
        robot.drive(-0.4);
        while (timer.seconds() < 5){

        }
        robot.basketUndo();
        timer.reset();
        while(timer.seconds()<2){

        }
    }
}
