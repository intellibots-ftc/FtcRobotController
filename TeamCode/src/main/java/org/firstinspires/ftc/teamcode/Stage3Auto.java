package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Stage 3 Auto", group="Basic Auto")
public class Stage3Auto extends LinearOpMode {
    RobotControl robot = new RobotControl(this);
    double final BASKET_X = 0;
    double final BASKET_Y = 0;
    double final SAMPLE_A_X = 0;
    double final SAMPLES_Y = 0;
    double final SAMPLE_B_X = 0;
    double final SAMPLE_C_X = 0;
    double final ARM_TAKE = 0;
    double final EXTENSION_TAKE = 0;
    

    @Override
    public void runOpMode(){
        robot.init();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        robot.armTarget = robot.ARM_HIGH;
        robot.extTarget = robot.MAX_EXTENSION;
        robot.driveToTarget(BASKET_X, BASKET_Y, 45,0.2);
        robot.positionServo();
        robot.waitUntilReached();
        robot.targetStop();
        robot.intake_servo.setPower(0.3);
        timer.reset();
        while (timer.seconds()<3){

        }
        robot.driveToTarget(SAMPLE_A_X, SAMPLES_Y, 0, 0.2);
        while (timer.seconds() < 4){

        }
        robot.armTarget = 1200;
        robot.extTarget = EXTENSION_TAKE;
        while(timer.seconds()<5){

        }
        robot.armTarget = ARM_TAKE;
        while(timer.seconds()<8){

        }
    }
}
