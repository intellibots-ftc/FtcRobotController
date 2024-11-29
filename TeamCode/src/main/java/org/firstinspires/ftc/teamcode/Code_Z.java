package org.firstinspires.ftc.teamcode;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp(name="Final code", group="Linear OpMode")
public class Code_Z extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    RobotControl robot = new RobotControl(this);
    private double mod = 1;
    private double slow = 1;
    
    GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses START)
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(8.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

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

            if (robot.controlOn == 1) {
                robot.controllerDrive(axial, lateral, yaw, mod);
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry

            if (gamepad1.right_stick_button && mod == 1) {
                slow = 0.4;
            }

            if (gamepad1.right_stick_button && mod == 0.4){
                slow = 1;
            }

            if (!gamepad1.right_stick_button){
                mod = slow;
            }

            // Send calculated power to wheels
            if (gamepad1.dpad_up) {
                robot.armTarget = robot.ARM_HIGH;
                robot.extTarget = robot.MAX_EXTENSION;
            }

            if (gamepad1.dpad_down) {
                robot.armTarget = -1500;
                robot.extTarget = -50;
            }

            if (gamepad1.dpad_left){
                robot.rotate(0, 0.1);
                robot.DriveToTarget(0, 0, 0.1);
            }

            if (gamepad1.b) {
                robot.armControl(mod);
            } else if (gamepad1.y) {
                robot.armControl(-mod);
            } else {
                robot.armControl(0);

            }

            if (gamepad1.right_trigger > 0) {
                robot.intakeServo.setPower(1);
            } else if (gamepad1.left_trigger>0) {
                robot.intakeServo.setPower(-0.5);
            } else if (gamepad1.left_stick_button) {
                robot.intakeServo.setPower(0);
            }

            if (gamepad1.right_bumper && robot.extensionMotor.getCurrentPosition() > robot.MAX_EXTENSION) {
                robot.extendControl(-mod);
            } else if (gamepad1.left_bumper && robot.extensionMotor.getCurrentPosition() < -50) {
                robot.extendControl(mod);
            } else {
                robot.extendControl(0);
            }

            if (gamepad1.x) {
                robot.intakeRotatorServo.setPosition(0.1666);
            } else if (gamepad1.a) {
                robot.intakeRotatorServo.setPosition(0.8333);
            } else {
                robot.intakeRotatorServo.setPosition(0.5);
            }

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("odo Position", data);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            /*telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);*/
            telemetry.addData("Extension motor position", robot.extensionMotor.getCurrentPosition());
            telemetry.addData("Extension motor target", robot.extTarget);
            telemetry.addData("Arm motor position", robot.armMotor.getCurrentPosition());
            telemetry.addData("Arm motor target", robot.armTarget);
            /*telemetry.addData("intake rotator servo pos",intakeRotatorServo.getPosition());
            telemetry.addData("intake power",intakeServo.getPower());*/
            telemetry.update();
        }
    }}
