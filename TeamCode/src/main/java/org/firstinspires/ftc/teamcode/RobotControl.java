package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

public class RobotControl {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor armMotor = null;
    public DcMotor extensionMotor=null;
    public CRServo intakeServo =null;
    public Servo intakeRotatorServo=null;

    public static final double ARM_HANG_POS = -1500;
    public static final int ARM_HIGH = -2800;
    public static final int MAX_EXTENSION = -2000;
    public static final double TICKS_PER_DEGREE = 2500/360;
    public static final double TICKS_PER_MM = 1;
    public static final double BASKET_X_AUTO = 0;
    public static final double BASKET_Y_AUTO = 0;
    public static final double BASKET_X_TELE = 950;
    public static final double BASKET_Y_TELE = 180;

    private double leftFrontPower  = 0;
    public int controlOn = 1;
    private double rightFrontPower = 0;
    private double leftBackPower   = 0;
    private double rightBackPower  = 0;
    public int armTarget = 0;
    public int extTarget = 0;

    GoBildaPinpointDriver odo;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotControl (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "arm_motor");
        extensionMotor   = myOpMode.hardwareMap.get(DcMotor.class, "extension_motor");
        intakeRotatorServo = myOpMode.hardwareMap.get(Servo.class, "intake_rotator_servo");
        intakeServo = myOpMode.hardwareMap.get(CRServo.class, "intake_servo");

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(8.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeServo.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void powerDrive(double lf, double rf, double lb, double rb){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontPower  = lf;
        rightFrontPower = rf;
        leftBackPower   = lb;
        rightBackPower  = rb;
    }

    public void defDrive(String mode){
        if (mode == "forward"){
            powerDrive(1, 1, 1, 1);
        }
    }

    public void controllerDrive(double axial, double lateral, double yaw, double mod) {
        if (mod <= 1) {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            mod /= 100;
        }

        leftFrontPower  = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower   = axial - lateral + yaw;
        rightBackPower  = axial + lateral - yaw;

        double max = 0;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontPower *= mod;
        rightFrontPower *= mod;
        leftBackPower *= mod;
        rightBackPower *= mod;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void armControl(double power){
        if (power != 0){
            int validity = 1; /*
            if (power > 0 && armMotor.getCurrentPosition() < -50){
                validity = 1;
            } else if (power < 0 && armMotor.getCurrentPosition() > ARM_HIGH){
                validity = 1;
            }*/
            armMotor.setPower(power * validity);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armTarget = armMotor.getCurrentPosition();
        } else {
            armMotor.setTargetPosition(armTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
        }
    }

    public void extendControl(double power){
        if (power != 0){
            double validity = 0;
            if (power > 0 && extensionMotor.getCurrentPosition() < -50){
                validity = Math.min(1, extensionMotor.getCurrentPosition()/-400);
            } else if (power < 0 && extensionMotor.getCurrentPosition() > MAX_EXTENSION){
                validity = Math.min(1, (extensionMotor.getCurrentPosition() - MAX_EXTENSION)/400);
            }
            extensionMotor.setPower(power * validity);
            extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extTarget = extensionMotor.getCurrentPosition();
        } else if (extensionMotor.getCurrentPosition() < -60) {
            extensionMotor.setTargetPosition(extTarget);
            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extensionMotor.setPower(1);
        } else {
            extensionMotor.setPower(0);
        }
    }

    /*public void rotate (double end, double power){
        controlOn = 0;
        odo.update();
        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.DEGREES);
        double delta = heading - end;

        leftFrontDrive.setTargetPosition((int)leftFrontDrive.getCurrentPosition() + Math.round(-delta * TICKS_PER_DEGREE));
        rightFrontDrive.setTargetPosition((int)rightFrontDrive.getCurrentPosition() + Math.round(delta * TICKS_PER_DEGREE));
        leftBackDrive.setTargetPosition((int)leftBackDrive.getCurrentPosition() + Math.round(-delta * TICKS_PER_DEGREE));
        rightBackDrive.setTargetPosition((int)rightBackDrive.getCurrentPosition() + Math.round(delta * TICKS_PER_DEGREE));

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int i = 0;
        while (Math.abs(leftFrontDrive.getCurrentPosition()-leftFrontDrive.getTargetPosition()) > TICKS_PER_DEGREE && i == 0) {
            if (myOpMode.gamepad1.dpad_right){
                i = 1;
            }
        }
        controlOn = 1;
    } */

    public void DriveToTarget(double tarx, double tary, double tarh, double power){
        controlOn = 0;
        odo.update();

        Pose2D pos = odo.getPosition();
        double posx = pos.getX(DistanceUnit.MM);
        double posy = pos.getY(DistanceUnit.MM);
        double head = -pos.getHeading(AngleUnit.DEGREES);

        double deltax = tarx - posx;
        double deltay = tary - posy;
        double yaw = tarh - head;

        double distance = Math.hypot(deltax, deltay);

        double axial = distance * Math.cos(Math.toRadians(yaw));
        double lateral = distance * Math.sin(Math.toRadians(yaw));

        yaw *= TICKS_PER_DEGREE;

        controllerDrive(axial, lateral, yaw, power * 100);

        distance *= TICKS_PER_MM;

        leftFrontDrive.setTargetPosition((int) (leftFrontDrive.getCurrentPosition() + leftFrontDrive.getPower() * distance));
        rightFrontDrive.setTargetPosition((int) (rightFrontDrive.getCurrentPosition() + rightFrontDrive.getPower() * distance));
        leftBackDrive.setTargetPosition((int) (leftBackDrive.getCurrentPosition() + leftBackDrive.getPower() * distance));
        rightBackDrive.setTargetPosition((int) (rightBackDrive.getCurrentPosition() + rightBackDrive.getPower() * distance));

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDriveTargets(int a, int b, int c, int d){
        leftFrontDrive.setTargetPosition(a);
        rightFrontDrive.setTargetPosition(b);
        leftBackDrive.setTargetPosition(c);
        rightBackDrive.setTargetPosition(d);

    }

    public void targetStop(){
        setDriveTargets(leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
    }

    public void waitUntilReached(){
        while (Math.abs(leftFrontDrive.getCurrentPosition() - leftFrontDrive.getTargetPosition()) > 10) {

        }
    }

    public void resetDrive(){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
