package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;

@TeleOp(name="Final code", group="Linear OpMode")
public class Code_Z extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo;
    RobotControl robot = new RobotControl(this);
    private EnhancedNavigation navigation;
    private double mod = 1;
    private double slow = 1;
    private boolean isNavigatingToBasket = false;

    // Basket coordinates (adjust these based on your field setup)
    private final double BASKET_X = robot.BASKET_X_TELE;  // Using constant from RobotControl
    private final double BASKET_Y = robot.BASKET_Y_TELE;  // Using constant from RobotControl
    private static final double BASKET_HEADING = -45.0;  // Degrees

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(8.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        // Initialize navigation system
        navigation = new EnhancedNavigation(robot, odo);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Handle automatic basket navigation when dpad_left is pressed
            if (gamepad1.dpad_left && !isNavigatingToBasket) {
                isNavigatingToBasket = true;
                navigation.resetController(); // Reset the PIDF controller
            }

            if (isNavigatingToBasket) {
                // Use enhanced navigation to move to basket position
                boolean atTarget = navigation.navigateToPosition(BASKET_X, BASKET_Y, BASKET_HEADING, 0.8);
                if (atTarget) {
                    isNavigatingToBasket = false;
                    robot.controlOn = 1;
                    robot.targetStop();// Re-enable manual control
                }

                // Allow cancellation of automatic navigation with dpad_right
                if (gamepad1.dpad_right) {
                    isNavigatingToBasket = false;
                    robot.controlOn = 1;
                }
            } else {
                // Normal teleop control when not navigating to basket
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                // Apply deadband
                if (Math.abs(axial) < 0.15) axial = 0;
                if (Math.abs(lateral) < 0.15) lateral = 0;

                if (robot.controlOn == 1) {
                    robot.controllerDrive(axial, lateral, yaw, mod);
                } else if (axial != 0 || lateral != 0 || yaw != 0) {
                    robot.controllerDrive(axial, lateral, yaw, mod);
                    robot.controlOn = 1;
                }
            }

            // Speed control
            if (gamepad1.right_stick_button) {
                slow = (mod == 1) ? 0.4 : 1;
            }
            if (!gamepad1.right_stick_button) {
                mod = slow;
            }

            // Arm control
            if (gamepad1.dpad_up) {
                robot.armTarget = robot.ARM_HIGH;
                robot.extTarget = robot.MAX_EXTENSION;
            }
            if (gamepad1.dpad_down) {
                robot.armTarget = -1500;
                robot.extTarget = -50;
            }

            // Manual arm control
            if (gamepad1.b) {
                robot.armControl(mod, 1);
            } else if (gamepad1.y) {
                robot.armControl(-mod, 1);
            } else {
                robot.armControl(0, 1);
            }

            // Intake control
            if (gamepad1.right_trigger > 0) {
                robot.intakeServo.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                robot.intakeServo.setPower(-0.5);
            } else if (gamepad1.left_stick_button) {
                robot.intakeServo.setPower(0);
            }

            // Extension control
            if (gamepad1.right_bumper && robot.extensionMotor.getCurrentPosition() > robot.MAX_EXTENSION) {
                robot.extendControl(-mod);
            } else if (gamepad1.left_bumper && robot.extensionMotor.getCurrentPosition() < -50) {
                robot.extendControl(mod);
            } else {
                robot.extendControl(0);
            }

            // Intake rotator control
            if (gamepad1.x) {
                robot.intakeRotatorServo.setPosition(0.1666);
            } else if (gamepad1.a) {
                robot.intakeRotatorServo.setPosition(0.8333);
            } else {
                robot.intakeRotatorServo.setPosition(0.5);
            }

            // Update odometry and telemetry
            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES));

            // Telemetry updates
            telemetry.addData("odo Position", data);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Extension motor position", robot.extensionMotor.getCurrentPosition());
            telemetry.addData("Extension motor target", robot.extTarget);
            telemetry.addData("Arm motor position", robot.armMotor.getCurrentPosition());
            telemetry.addData("Arm motor target", robot.armTarget);
            if (isNavigatingToBasket) {
                telemetry.addData("Navigation", "Moving to Basket");
            }
            telemetry.update();
        }
    }
}
