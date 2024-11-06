package org.firstinspires.ftc.teamcode;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="Team code", group="Linear OpMode")
public class Code_X extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    AutoDriver robot = new AutoDriver(this);
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private DcMotor extensionMotor=null;
    private int target = 0;
    private double mod = 1;
    private double slow = 1;
    private CRServo intakeServo =null;
    private Servo intakeRotatorServo=null;

    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        extensionMotor = hardwareMap.get(DcMotor.class, "extension_motor");
        intakeRotatorServo = hardwareMap.get(Servo.class, "intake_rotator_servo");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

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

        // Wait for the game to start (driver presses START)
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            if (Math.abs(axial)<0.15){
                axial=0;
            }
            if (Math.abs(lateral)<0.15){
                lateral=0;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            float armPower = 0;
            float extensionPower = 0;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            if (gamepad1.right_stick_button && mod == 1) {
                slow = 0.3;
            }

            if (gamepad1.right_stick_button && mod == 0.2){
                slow = 1;
            }

            if (!gamepad1.right_stick_button){
                mod = slow;
            }

            // Send calculated power to wheels
            if (gamepad1.b) {
                armPower = 1;
                armMotor.setPower(armPower * mod);
                target = armMotor.getCurrentPosition();
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad1.y) {
                armPower = -1;
                armMotor.setPower(armPower * mod);
                target = armMotor.getCurrentPosition();
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                armPower = 0;
                armMotor.setTargetPosition(target);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if (gamepad1.right_trigger > 0) {
                intakeServo.setPower(1);
            } else if (gamepad1.left_trigger>0) {
                intakeServo.setPower(-0.5);
            } else if (gamepad1.left_stick_button) {
                intakeServo.setPower(0);
            }

            if (gamepad1.right_bumper && extensionMotor.getCurrentPosition() > -5750) {
                extensionPower = Math.max(-1,(-5750-extensionMotor.getCurrentPosition())/200);
                target = Math.max(target, (extensionMotor.getCurrentPosition()+200)/-5)
            } else if (gamepad1.left_bumper && extensionMotor.getCurrentPosition() < -50) {
                extensionPower = Math.min(1,(-extensionMotor.getCurrentPosition())/200);
            } else {
                extensionPower = 0;
            }

            if (gamepad1.x) {
                intakeRotatorServo.setPosition(0.1666);
            } else if (gamepad1.a) {
                intakeRotatorServo.setPosition(0.8333);
            } else {
                intakeRotatorServo.setPosition(0.5);
            }

            if (gamepad1.dpad_up) {
                robot.basketNoMotor();
            }

            leftFrontDrive.setPower(leftFrontPower * mod);
            rightFrontDrive.setPower(rightFrontPower * mod);
            leftBackDrive.setPower(leftBackPower * mod);
            rightBackDrive.setPower(rightBackPower * mod);
            extensionMotor.setPower(extensionPower * mod);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Extension motor position", extensionMotor.getCurrentPosition());
            telemetry.addData("Arm motor position",armMotor.getCurrentPosition());
            telemetry.addData("intake rotator servo pos",intakeRotatorServo.getPosition());
            telemetry.addData("intake power",intakeServo.getPower());
            telemetry.update();
        }
    }}
