package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Specimen", name = "5 Spec Auto")
public class FirstSpecimen extends LinearOpMode {
    private AutoControl auto;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        auto = new AutoControl(this);
        auto.initialize();
        auto.power = 0.8;
        auto.weird = 1;
        auto.wallWait = 0;

        while (!isStarted()) {
            auto.updateTelemetry();
        }

        // Main autonomous sequence
        // Move to scoring position
        auto.setup();
        auto.robot.intakeServo.setPower(-1);
        auto.goToChamber(-400);
        auto.wait(0.6);
        auto.scoreSpecimen();
        auto.pushSpikeMark(-1500);
        auto.pushSpikeMark(-1720);
        auto.specimenFive();
        auto.goToChamber(-500);
        auto.skibidi = 2.5;
        auto.scoreSpecimen();
        auto.grabFromWall();
        auto.goToChamber(-450);
        auto.scoreSpecimen();
        auto.grabFromWall();
        auto.goToChamber(-270);
        auto.scoreSpecimen();
        auto.grabFromWall();
        auto.goToChamber(-210);
        auto.scoreSpecimen();
        auto.moveTo(-1300, -310, 90, 1, 5, 1);


        // Park in designated area


    }
}
