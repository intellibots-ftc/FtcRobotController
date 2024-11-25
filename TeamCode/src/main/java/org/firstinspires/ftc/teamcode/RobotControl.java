package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class RobotControl {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    public DcMotor extensionMotor=null;
    public CRServo intakeServo =null;
    public Servo intakeRotatorServo=null;

    public static final double ARM_HANG_POS = -2000;
    public static final int ARM_HIGH = -3200;
    public static final int MAX_EXTENSION = -2050;

    private double leftFrontPower  = 0;
    private double rightFrontPower = 0;
    private double leftBackPower   = 0;
    private double rightBackPower  = 0;
    public int armTarget = 0;
    public int extTarget = 0;

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

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void powerDrive(double lf, double rf, double lb, double rb){
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

    public void controllerDrive(double axial, double lateral, double yaw) {
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
    }

    public void armControl(double power){
        if (power != 0){
            int validity = 0;
            if (power > 0 && armMotor.getCurrentPosition() < -50){
                validity = 1;
            } else if (power < 0 && armMotor.getCurrentPosition() > ARM_HIGH){
                validity = 1;
            }
            armMotor.setPower(power * validity);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armTarget = armMotor.getCurrentPosition();
        } else {
            armMotor.setTargetPosition(armTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void extendControl(double power){
        if (power != 0){
            double validity = 0;
            if (power > 0 && extensionMotor.getCurrentPosition() < -50){
                validity = Math.min(1, extensionMotor.getCurrentPosition()/-200);
            } else if (power < 0 && extensionMotor.getCurrentPosition() > MAX_EXTENSION){
                validity = Math.min(1, (extensionMotor.getCurrentPosition() - MAX_EXTENSION)/200);
            }
            extensionMotor.setPower(power * validity);
            extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extTarget = extensionMotor.getCurrentPosition();
        } else {
            extensionMotor.setTargetPosition(extTarget);
            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void updateDrive(double mod){
        leftFrontDrive.setPower(leftFrontPower * mod);
        rightFrontDrive.setPower(rightFrontPower * mod);
        leftBackDrive.setPower(leftBackPower * mod);
        rightBackDrive.setPower(rightBackPower * mod);
    }

    public void resetDrive(){
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
