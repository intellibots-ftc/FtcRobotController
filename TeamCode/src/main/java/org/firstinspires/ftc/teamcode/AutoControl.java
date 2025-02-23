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

    public double specArmTarget = -2100;
    public double weird = 0.6;
    private boolean isTeleOp = false;


    // Timeouts
    public double NAVIGATION_TIMEOUT = 5.0;  // seconds
    private static final double SCORING_TIMEOUT = 1.5;  // seconds
    private static final double COLLECTION_TIMEOUT = 0.5;
    public double wallWait = 1.0;

    private static double cPower = 1;// seconds
    private static final double specArmAdjust = 50;


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
        robot.armTarget = -1000;
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
        robot.intakeRotatorServo.setPosition(0.5);
        robot.intakeServoGrip.setPosition(0.3433);
        robot.intakeSpinnerServo.setPosition(0.84);

        wait(SCORING_TIMEOUT);

        robot.intakeRotatorServo.setPosition(0.2);
    }

    public void collectFromSpikeMark(double sampleX) throws InterruptedException {

        // Move arm to collecting position
        robot.armTarget = -200;
        robot.extTarget = -0;

        // Navigate to sample position
        moveTo(Math.max(sampleX * 0.8, 500),SAMPLES_Y + 250,SAMPLE_HEADING, power, NAVIGATION_TIMEOUT, cPower);

        // Activate intake to collect pixel
        robot.intakeRotatorServo.setPosition(0.6);

        moveTo(Math.max(sampleX * 0.8, 500), SAMPLES_Y, SAMPLE_HEADING, power, NAVIGATION_TIMEOUT, cPower);
        robot.armTarget = 0;
        moveTo(sampleX, SAMPLES_Y, SAMPLE_HEADING, power, NAVIGATION_TIMEOUT, cPower);
        robot.intakeServoGrip.setPosition(0.15);
        wait(COLLECTION_TIMEOUT);
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
        robot.intakeServoGrip.setPosition(0.15);
        robot.intakeRotatorServo.setPosition(0.26);
        robot.armTarget = (int) ( specArmTarget + specArmAdjust);
        robot.extTarget = 0;

        specimen_X = navigation.clamp(specimen_X, -500, -220);
        moveTo(specimen_X, -400, 90, weird, 2, 1);
    }

    public void scoreSpecimen(){
        //robot.intakeServo.setPower(0);
        moveTo(odo.getPosX(), -630, 90, power, NAVIGATION_TIMEOUT, 1);
        robot.armTarget = (int) (-1300 + specArmAdjust);
        //power = 0.8;
        moveTo(odo.getPosX(), -800, 90, power, NAVIGATION_TIMEOUT, 1);
        //robot.intakeServo.setPower(0.5);
        robot.intakeServoGrip.setPosition(0.3433);
        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds() < 0.2){
            robot.controllerDrive(-1, 0, 0, 1);
        }
    }

    public void grabFromWall(){
        double bing = odo.getPosX() < -1000? odo.getPosX() + 100 : -650;
        moveTo(bing, -330, (180 * Math.signum(bing + 650)) + 180, weird, 1, 1);
        robot.armTarget = (int) (-800 + specArmAdjust);
        //robot.intakeServo.setPower(-1);
        moveTo(-1300, -310, -90, weird, skibidi, 1);
        wait(wallWait);
        moveTo(-1300, -150, -90, power, 1.5, 1);
        robot.intakeServoGrip.setPosition(0.15);
    }
    public void specimenFive(){
        
        double place = -1950;
        moveTo(place + 250, -1250, -90, weird, NAVIGATION_TIMEOUT, 1);
        moveTo(place, -1250, -90, weird, NAVIGATION_TIMEOUT, 1);
        //robot.intakeServo.setPower(-1);
        robot.armTarget = (int) (-800 + specArmAdjust);
        moveTo(place, -500, -90, weird, NAVIGATION_TIMEOUT, 1);
        moveTo(place, -120, -90, weird, NAVIGATION_TIMEOUT, 1);
        robot.intakeServoGrip.setPosition(0.15);
    }

    public void pushSpikeMark(double specX){
        moveTo(specX + 250, -500, 90, weird, NAVIGATION_TIMEOUT, 1);
        moveTo(specX + 250, -1250, 90, weird, NAVIGATION_TIMEOUT, 1);
        moveTo(specX, -1250, 90, weird, NAVIGATION_TIMEOUT, 1);
        moveTo(specX, -350, 90, weird, NAVIGATION_TIMEOUT, 1);
    }

    public void moveToBasket_Teleop(double x, double y, double h){
        if(odo.getPosY() < y - 1000 && odo.getPosX() < x - 400) {
            robot.armTarget = -600;
            robot.extTarget = 0;
            moveTo(x - 400, y-1200, -odo.getHeading(), 1, 5, 1);
        }
        robot.armTarget = ARM_SCORING;
        robot.extTarget = -2100;
        robot.intakeRotatorServo.setPosition(0.26);
        moveTo(x, y, h, 1, 5, 1);
    }

    public void dropOffSample(){
        if(odo.getPosY() < -900){
            robot.extTarget = 0;
            robot.armTarget = -900;
            //robot.intakeRotatorServo.setPosition(1);
            robot.intakeRotatorServo.setPosition(0.26);
            robot.intakeServo.setPower(-0.1);
            moveTo(-1500, odo.getPosY(), 0, 1, 5, 1);
            robot.extTarget = -1500;
            moveTo(odo.getPosX(), -600, -90, 1, 5, 1);
            //robot.intakeServo.setPower(0.5);
            robot.intakeServoGrip.setPosition(0.3433);
        } else if (odo.getPosX() < 0){
            robot.extTarget = 0;
            robot.armTarget = -600;
            //robot.intakeRotatorServo.setPosition(1);
            robot.intakeRotatorServo.setPosition(0.26);
            robot.intakeServo.setPower(-0.1);
            moveTo(odo.getPosX(), -650, 90, 1, 5, 1);
            robot.armTarget = -900;
            moveTo(odo.getPosX(), -400, 90, 1, 5, 1);
            robot.extTarget = -1500;
            moveTo(-1000, -200, 180, 1, 5, 1);
            //robot.intakeServo.setPower(0.5);
            robot.intakeServoGrip.setPosition(0.3433);
        }
    }

    public void moveTo(double x, double y, double h, double p, double t, double c){
        navigation.resetController();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        if(isTeleOp){
            t = 5;
        }
        while (myOpMode.opModeIsActive() && timer.seconds() < t) {
            if(isTeleOp){
                if(myOpMode.gamepad1.right_stick_button){
                    break;
                }
            }
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
            if(isTeleOp){
                if(myOpMode.gamepad1.right_stick_button){
                    break;
                }
            }
            robot.armControl(0, 1);
            robot.extendControl(0);
        }
    }

    public void teleopInitialize(RobotControl r, GoBildaPinpointDriver o, EnhancedNavigation n){
        robot = r;
        odo = o;
        navigation = n;
        isTeleOp = true;
        power = 1;
        weird = 1;
    }
}
