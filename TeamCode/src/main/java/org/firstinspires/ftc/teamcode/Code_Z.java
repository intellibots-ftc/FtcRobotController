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
    AutoControl auto = new AutoControl(this);
    private EnhancedNavigation navigation;
    private double mod = 1;
    private double slow = 1;
    private boolean isNavigating = false;
    private boolean bing = false;

    // Basket coordinates (adjust these based on your field setup)
    private final double BASKET_X = robot.BASKET_X_TELE;  // Using constant from RobotControl
    private final double BASKET_Y = robot.BASKET_Y_TELE;  // Using constant from RobotControl
    private static final double BASKET_HEADING = -40.0;
    private double specPhase = 0;

    private double navX = 0;
    private double navY = 0;
    private double navH = 0;// Degrees
    private boolean isEditing = false;
    private boolean nearBasket = false;
    double basketXError = BASKET_X;
    double basketYError = BASKET_Y;
    double basketHError = BASKET_HEADING;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(6.25, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //odo.resetPosAndIMU();

        // Initialize navigation system
        navigation = new EnhancedNavigation(robot, odo);

        auto.teleopInitialize(robot, odo, navigation);
        auto.skibidi = 5;

        waitForStart();
        runtime.reset();
        robot.intakeRotatorServo.setPosition(1);

        while (opModeIsActive()) {
            // Handle automatic basket navigation when dpad_left is pressed
            if (gamepad1.dpad_left) {
                /*isNavigating = true;
                navX = basketXError;
                navY = basketYError;
                navH = basketHError;*/
                nearBasket = true;
                navigation.resetController();
                auto.moveToBasket_Teleop(basketXError, basketYError, basketHError);// Reset the PIDF controller
            }
            if (gamepad1.x) {
                navigation.resetController();
                odo.update();
                if(specPhase == 1){
                    /*navX = -400;
                    navY = -630;
                    navH = 90;
                    robot.armTarget = -2050;*/
                    auto.goToChamber(-400);
                    specPhase = 2;
                } else if (specPhase == 0) {
                    auto.grabFromWall();
                    specPhase = 1;
                } else {/*navX = odo.getPosX(); navY = -780; navH = 90; robot.armTarget = -1630;*/
                    auto.scoreSpecimen();
                    specPhase = 0;
                }// Reset the PIDF controller
            }
            if (gamepad1.a) {
                /*isNavigating = true;
                navX = -1300;
                navY = -400;
                navH = -90;
                robot.armTarget = -900;
                navigation.resetController();*/
                //auto.grabFromWall();// Reset the PIDF controller
                auto.dropOffSample();
            }
                // Normal teleop control when not navigating to basket
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

                // Apply deadband

            robot.controllerDrive(axial, lateral, yaw, mod);
            nearBasket = (nearBasket && axial == 0 && lateral == 0);

            // Speed control
            if (gamepad1.right_stick_button) {
                slow = (mod == 1) ? 0.4 : 1;
                basketXError = (isEditing) ? odo.getPosX(): basketXError;
                basketYError = (isEditing) ? odo.getPosY() : basketYError;
                basketHError = (isEditing) ? -odo.getPosition().getHeading(AngleUnit.DEGREES) : basketHError;
                isEditing = nearBasket && gamepad1.right_stick_y < -0.5;

            }
            if (!gamepad1.right_stick_button) {
                mod = slow;
            }

            // Arm control
            if (gamepad1.dpad_up) {
                bing = !robot.isTeleop;
            }
            if (!gamepad1.dpad_up) {
                robot.isTeleop = bing;
            }
            if (gamepad1.dpad_down) {
                if(robot.armMotor.getCurrentPosition() < -600){
                    robot.armTarget = -1000;
                } else {
                    robot.armTarget = -650;
                }
                robot.extTarget = 0;
            }
            if (gamepad1.dpad_right){
                robot.extTarget = 0;
                if(robot.extensionMotor.getCurrentPosition() > -50){
                    robot.armTarget = -4200;
                }
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
                robot.intakeServo.setPower(-1);
            } else if (gamepad1.left_trigger > 0) {
                robot.intakeServo.setPower(1);
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
                //robot.intakeRotatorServo.setPosition(0.1666);
            } else if (gamepad1.a) {
                //robot.intakeRotatorServo.setPosition(0.8333);
            } else {
                //robot.intakeRotatorServo.setPosition(1);
            }

            // Update odometry and telemetry
            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES));

            // Telemetry updates
            telemetry.addData("tigt", robot.intakeRotatorServo.getPosition());
            telemetry.addData("odo Position", data);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Extension motor position", robot.extensionMotor.getCurrentPosition());
            telemetry.addData("Extension motor target", robot.extTarget);
            telemetry.addData("Arm motor position", robot.armMotor.getCurrentPosition());
            telemetry.addData("Arm motor target", robot.armTarget);
            telemetry.addData("near basker", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("editing", odo.getHeading());
            telemetry.addData("basket h errpr", basketHError);
            if (isNavigating) {
                telemetry.addData("Navigation", "Moving to Basket");
            }
            telemetry.update();
        }
    }
}
