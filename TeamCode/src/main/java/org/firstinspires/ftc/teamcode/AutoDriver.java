package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class AutoDriver {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;

    private Servo intakeRotatorServo = null;
    private CRServo intakeServo = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    /*public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    public static final double ARM_HANG_POS = -2000;
    public static final double ARM_HIGH = -3500;
    public static final double MAX_EXTENSION = -5700;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutoDriver (LinearOpMode opmode) {
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

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void strafe(double direction) {
        leftFrontDrive.setPower(direction);
        leftBackDrive.setPower(-direction);
        rightFrontDrive.setPower(-direction);
        rightBackDrive.setPower(direction);
    }

    public void drive(double direction) {
        leftFrontDrive.setPower(direction);
        leftBackDrive.setPower(direction);
        rightFrontDrive.setPower(direction);
        rightBackDrive.setPower(direction);
    }

    public void turn(double direction) {
        leftFrontDrive.setPower(direction);
        leftBackDrive.setPower(direction);
        rightFrontDrive.setPower(-direction);
        rightBackDrive.setPower(-direction);
    }

    public void rightDiagonal(double direction) {
        leftFrontDrive.setPower(direction);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(direction);
    }

    public void leftDiagonal(double direction) {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(direction);
        rightFrontDrive.setPower(direction);
        rightBackDrive.setPower(0);
    }

    public void hangNoMotor(){
        intakeRotatorServo.setPosition(0.83333);
        leftFrontDrive.setTargetPosition(6100);
        leftBackDrive.setTargetPosition(6100);
        rightFrontDrive.setTargetPosition(6100);
        rightBackDrive.setTargetPosition(6100);
        armMotor.setTargetPosition(ARM_HANG_POS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LeftFrontDrive.isBusy()){
            
        }
        leftFrontDrive.setTargetPosition(5000);
        leftBackDrive.setTargetPosition(5000);
        rightFrontDrive.setTargetPosition(5000);
        rightBackDrive.setTargetPosition(5000);
        armMotor.setTargetPosition(ARM_HANG_POS+500);
    }

    public void basketNoMotor(){
        int check = (armMotor.getCurrentPosition() > ARM_HIGH) ? -1:1;
        boolean endArm = false;
        boolean endExt = false;
        while (!endArm || !endExt){
            if (Math.abs(armMotor.getCurrentPosition() - ARM_HIGH) < 50 || Math.abs(armMotor.getPower() - check)>1){
                armMotor.setPower(0);
                endArm = true;
            } else {
                armMotor.setPower(check);
            }
            if (Math.abs(extensionMotor.getCurrentPosition() - MAX_EXTENSION) < 100 || (extensionMotor.getPower()+1)>1){
                extensionMotor.setPower(0);
                endExt = true;
            } else {
                extensionMotor.setPower(-1);
            }
        }
        intakeServo.setPower(0.5);
    }

    public void resetDrive(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}
