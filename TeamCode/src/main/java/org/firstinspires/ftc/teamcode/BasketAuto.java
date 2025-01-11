package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Basket", name = "4 Basket Auto")
public class BasketAuto extends LinearOpMode {
    private AutoControl auto;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        auto = new AutoControl(this);
        auto.initialize();

        while (!isStarted()) {
            auto.updateTelemetry();
        }

        // Main autonomous sequence
        try {
            // Move to scoring position
            auto.setup();
            auto.moveTo(auto.odo.getPosX(),-200, 0, 1, 2, 0);

            auto.moveToBasket();

            // Score the pixel
            auto.scorePixel();

            // Move to and collect from spike mark A
            auto.collectFromSpikeMark(auto.SAMPLE_A_X);

            // Return to scoring position
            auto.moveToBasket();

            // Score second pixel
            auto.scorePixel();

            auto.collectFromSpikeMark(auto.SAMPLE_B_X);
            auto.moveToBasket();
            auto.scorePixel();
            auto.collectFromSpikeMark(auto.SAMPLE_C_X);
            auto.moveToBasket();
            auto.scorePixel();

            // Park in designated area
            auto.parkRobot();

        } catch (InterruptedException e) {
            telemetry.addData("Error", "Autonomous sequence interrupted");
            telemetry.update();
        }
    }
}
