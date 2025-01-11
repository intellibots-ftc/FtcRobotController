package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class AutoControl {

    private LinearOpMode myOpMode = null;
    public RobotControl robot;
    public GoBildaPinpointDriver odo;
    public EnhancedNavigation navigation;

    // Field coordinates (in mm)
    public static final double BASKET_X = 960;  // Adjust based on field measurements
    public static final double BASKET_Y = -170;  // Adjust based on field measurements
    public static final double BASKET_HEADING = -40.0;  // Degrees

    public static final double SAMPLE_A_X = 620;
    public static final double SAMPLE_B_X = 840;
    public static final double SAMPLE_C_X = 1050;
    public static final double SAMPLES_Y = -900;
    public static final double SAMPLE_HEADING = 0;
    public double power = 0.6;

    // Arm positions
    private static final int ARM_SCORING = -2800;  // Scoring position
    private static final int ARM_COLLECTING = -0;  // Collecting position
    private static final int EXTENSION_SCORING = -2000;

    public double skibidi = 1.0;

    public double specArmTarget = -1950;


    // Timeouts
    public double NAVIGATION_TIMEOUT = 5.0;  // seconds
    private static final double SCORING_TIMEOUT = 1.5;  // seconds
    private static final double COLLECTION_TIMEOUT = 1.5;

    private static double cPower = 1;// seconds

    public AutoControl (LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public void initialize(){
        robot = new RobotControl(myOpMode);
        robot.init();
        robot.resetEncoders();
        cPower = power;

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-6.25, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        navigation = new EnhancedNavigation(robot, odo);
    }

    public void setup(){
        robot.positionServo();
        robot.controllerDrive(0, 1, 0, 1);
    }

    public void moveToBasket() throws InterruptedException {
        // Set arm to scoring position
        robot.armTarget = ARM_SCORING;
        robot.extTarget = -50;

        // Navigate to basket position
        odo.update();
        double startX = odo.getPosX();
        double startY = odo.getPosY();
        double startH = odo.getHeading();

        moveTo((BASKET_X + 2 * startX) / 3, (BASKET_Y + 2 * startY) / 3, (BASKET_HEADING + startH) / 2, power, NAVIGATION_TIMEOUT, 1);

        robot.extTarget = EXTENSION_SCORING;
        moveTo(BASKET_X, BASKET_Y, BASKET_HEADING, power, NAVIGATION_TIMEOUT, 1);
    }

    public void updateTelemetry(){
        odo.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES));
        myOpMode.telemetry.addData("Current", data);
        myOpMode.telemetry.addData("arm pos", robot.armMotor.getCurrentPosition());
        myOpMode.telemetry.addData("target", robot.armTarget);
        myOpMode.telemetry.update();
    }
    public void scorePixel() throws InterruptedException {
        // Activate intake servo to release pixel
        robot.intakeServo.setPower(-0.5);

        wait(SCORING_TIMEOUT);

        robot.intakeServo.setPower(0);
    }

    public void collectFromSpikeMark(double sampleX) throws InterruptedException {

        // Move arm to collecting position
        robot.armTarget = ARM_COLLECTING;
        robot.extTarget = -0;
        robot.intakeServo.setPower(0.5);

        // Navigate to sample position
        moveTo(Math.max(sampleX * 0.8, 500),SAMPLES_Y + 250,SAMPLE_HEADING, power, NAVIGATION_TIMEOUT, cPower);

        // Activate intake to collect pixel

        moveTo(Math.max(sampleX * 0.8, 500), SAMPLES_Y, SAMPLE_HEADING, power, NAVIGATION_TIMEOUT, cPower);
        moveTo(sampleX, SAMPLES_Y, SAMPLE_HEADING, power, NAVIGATION_TIMEOUT, cPower);

        wait(COLLECTION_TIMEOUT);

        robot.intakeServo.setPower(0);
    }


    public void parkRobot() throws InterruptedException {

        // Move arm to safe position// Safe position
        robot.armTarget = -2000;
        robot.extTarget = -0;    // Retracted position

        // Navigate to parking position (adjust coordinates as needed)
        moveTo(500, -1300, 180, power, NAVIGATION_TIMEOUT, cPower);
        moveTo(150, -1300, 180, power, NAVIGATION_TIMEOUT, cPower);
    }

    public void goToChamber(double specimen_X){
        robot.armTarget = (int) specArmTarget;
        robot.extTarget = 0;

        specimen_X = navigation.clamp(specimen_X, -500, -300);
        moveTo(specimen_X, -400, 90, power, 2, 1);
    }

    public void scoreSpecimen(){
        moveTo(odo.getPosX(), -630, 90, power, NAVIGATION_TIMEOUT, 1);
        robot.armTarget = -1630;
        //power = 0.8;
        moveTo(odo.getPosX(), -780, 90, power, NAVIGATION_TIMEOUT, 1);
        robot.intakeServo.setPower(0.5);
        moveTo(odo.getPosX(), -400, 90, power, NAVIGATION_TIMEOUT, 1);
    }

    public void grabFromWall(){
        moveTo(odo.getPosX() + 100, -500, 0, power, 1, 1);
        robot.armTarget = -1200;
        robot.intakeServo.setPower(-0.5);
        moveTo(-1300, -310, -90, power, skibidi, 1);
        robot.armTarget = -820;
        wait(1.0);
        moveTo(-1300, -150, -90, power, 1.5, 1);
    }

    public void pushSpikeMark(double specX){
        moveTo(specX + 250, -500, 90, power, NAVIGATION_TIMEOUT, 1);
        moveTo(specX + 250, -1300, 90, power, NAVIGATION_TIMEOUT, 1);
        moveTo(specX, -1300, 90, power, NAVIGATION_TIMEOUT, 1);
        moveTo(specX, -350, 90, power, NAVIGATION_TIMEOUT, 1);
    }

    public void moveTo(double x, double y, double h, double p, double t, double c){
        navigation.resetController();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (myOpMode.opModeIsActive() && timer.seconds() < t) {
            if (navigation.navigateToPosition(x, y, h, p)) {
                break;
            }
            updateTelemetry();
            robot.armControl(0, c);
            robot.extendControl(0);
        }
        robot.resetDrive();
    }

    public void wait(double t) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (myOpMode.opModeIsActive() && timer.seconds() < t) {
            robot.armControl(0, 1);
            robot.extendControl(0);
        }
    }
}
