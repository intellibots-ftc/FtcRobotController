package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.BufferedInputStream;

@Autonomous(name="Enhanced Stage 3 Auto", group="Competition")
public class EnhancedStage3Auto extends LinearOpMode {
    private RobotControl robot;
    private GoBildaPinpointDriver odo;
    private EnhancedNavigation navigation;
    private ElapsedTime timer;

    // Field coordinates (in mm)
    private static final double BASKET_X = 950;  // Adjust based on field measurements
    private static final double BASKET_Y = 200;  // Adjust based on field measurements
    private static final double BASKET_HEADING = -45;  // Degrees

    private static final double SAMPLE_A_X = 640;
    private static final double SAMPLE_B_X = 860;
    private static final double SAMPLE_C_X = 1150;
    private static final double SAMPLES_Y = -950;
    private static final double SAMPLE_HEADING = 0;
    private static final double power = 0.4;

    // Arm positions
    private static final int ARM_SCORING = -2800;  // Scoring position
    private static final int ARM_COLLECTING = -0;  // Collecting position
    private static final int EXTENSION_SCORING = -2000;
    private static final int EXTENSION_COLLECTING = -50;

    // Timeouts
    private static final double NAVIGATION_TIMEOUT = 7.0;  // seconds
    private static final double SCORING_TIMEOUT = 3.0;  // seconds
    private static final double COLLECTION_TIMEOUT = 3.0;  // seconds

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot = new RobotControl(this);
        robot.init();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(8.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        navigation = new EnhancedNavigation(robot, odo);
        ElapsedTime timer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        timer.reset();

        // Main autonomous sequence
        try {
            // Move to scoring position
            robot.positionServo();
            
            moveToScoringPosition();

            // Score the pixel
            scorePixel();

            // Move to and collect from spike mark A
            collectFromSpikeMark(SAMPLE_A_X);

            // Return to scoring position
            moveToScoringPosition();

            // Score second pixel
            scorePixel();

            collectFromSpikeMark(SAMPLE_B_X);
            moveToScoringPosition();
            scorePixel();
            collectFromSpikeMark(SAMPLE_C_X);
            moveToScoringPosition();
            scorePixel();

            // Park in designated area
            parkRobot();

        } catch (InterruptedException e) {
            telemetry.addData("Error", "Autonomous sequence interrupted");
            telemetry.update();
        }
    }

    private void moveToScoringPosition() throws InterruptedException {
        navigation.resetController();
        telemetry.addData("Status", "Moving to scoring position");
        telemetry.update();

        // Set arm to scoring position
        robot.armTarget = ARM_SCORING;
        robot.extTarget = EXTENSION_SCORING;

        // Navigate to basket position
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < NAVIGATION_TIMEOUT) {
            if (navigation.navigateToPosition(BASKET_X, BASKET_Y, BASKET_HEADING, power)) {
                break;
            }
            robot.armControl(0);
            robot.extendControl(0);

            // Show current position for debugging
            Pose2D currentPose = odo.getPosition();
            telemetry.addData("Current X", currentPose.getX(DistanceUnit.MM));
            telemetry.addData("Current Y", currentPose.getY(DistanceUnit.MM));
            telemetry.addData("Current Heading", currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.update();

            if (!opModeIsActive()) throw new InterruptedException();
        }
    }

    private void scorePixel() throws InterruptedException {
        telemetry.addData("Status", "Scoring pixel");
        telemetry.update();

        // Activate intake servo to release pixel
        robot.intakeServo.setPower(0.3);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < SCORING_TIMEOUT) {
            if (!opModeIsActive()) throw new InterruptedException();
        }

        robot.intakeServo.setPower(0);
    }

    private void collectFromSpikeMark(double sampleX) throws InterruptedException {
        telemetry.addData("Status", "Collecting from spike mark");
        telemetry.update();

        // Move arm to collecting position
        robot.armTarget = ARM_COLLECTING;
        robot.extTarget = EXTENSION_COLLECTING;

        // Navigate to sample position
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < NAVIGATION_TIMEOUT) {
            if (navigation.navigateToPosition(sampleX, SAMPLES_Y, SAMPLE_HEADING, power)) {
                break;
            }
            robot.armControl(0);
            robot.extendControl(0);
            
            if (!opModeIsActive()) throw new InterruptedException();
        }

        // Activate intake to collect pixel
        robot.intakeServo.setPower(-0.5);

        timer.reset();
        while (opModeIsActive() && timer.seconds() < COLLECTION_TIMEOUT) {
            robot.armControl(0);
            robot.extendControl(0);
            
            if (!opModeIsActive()) throw new InterruptedException();
        }

        robot.intakeServo.setPower(0);
    }


    private void parkRobot() throws InterruptedException {
        telemetry.addData("Status", "Parking");
        telemetry.update();

        // Move arm to safe position
        robot.armTarget = -1500;  // Safe position
        robot.extTarget = -50;    // Retracted position

        // Navigate to parking position (adjust coordinates as needed)
        timer.reset();
        while (opModeIsActive() && timer.seconds() < NAVIGATION_TIMEOUT) {
            if (navigation.navigateToPosition(0, 0, 0, power)) {
                break;
            }
            robot.armControl(0);
            robot.extendControl(0);
            
            if (!opModeIsActive()) throw new InterruptedException();
        }
    }
}
