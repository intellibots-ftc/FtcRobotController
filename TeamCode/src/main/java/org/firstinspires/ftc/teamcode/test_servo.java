//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import java.util.Locale;
//
//@TeleOp(name="test code", group="Linear OpMode")
//public class test_servo extends LinearOpMode {
//    private ElapsedTime runtime = new ElapsedTime();
//    GoBildaPinpointDriver odo;
//    RobotControl robot = new RobotControl(this);
//    AutoControl auto = new AutoControl(this);
//    private double mod = 1;
//    private double slow = 1;
//    private boolean isNavigating = false;
//    private boolean bing = false;
//
//    // Basket coordinates (adjust these based on your field setup)
//    private final double BASKET_X = robot.BASKET_X_TELE;  // Using constant from RobotControl
//    private final double BASKET_Y = robot.BASKET_Y_TELE;  // Using constant from RobotControl
//    private static final double BASKET_HEADING = -40.0;
//    private double specPhase = 0;
//
//    private double navX = 0;
//    private double navY = 0;
//    private double navH = 0;// Degrees
//    private boolean isEditing = false;
//    private boolean nearBasket = false;
//    double basketXError = BASKET_X;
//    double basketYError = BASKET_Y;
//    double basketHError = BASKET_HEADING;
//    double servoIncrement = 1.0 / 180.0;
//    double servoPos =0;
//    @Override
//    public void runOpMode() {
//        // Initialize robot hardware
//        robot.init();
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        // Initialize odometry
//        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//
//
//        waitForStart();
//        runtime.reset();
//        while (opModeIsActive()) {
//            // Intake rotator control
//            /*
//            if (gamepad1.x) {
////                servoPos += servoIncrement;
//                robot.testServos(0.001);
////                robot.intakeServoGrip.setPosition(0.3433); // gripper abs open position
//
//            }
//            if (gamepad1.b) {
////                servoPos -= servoIncrement;
//                robot.testServos(-0.001);
////                robot.intakeServoGrip.setPosition(0.15); // gripper abs closed position
//            }
//*/
//
////            // Manual arm control
////            if (gamepad1.b) {
////                robot.armControl(mod, 1);
////            } else if (gamepad1.y) {
////                robot.armControl(-mod, 1);
////            } else {
////                robot.armControl(0, 1);
////            }
//
////            if (gamepad1.dpad_up) {
////                robot.intakeRotatorServo.setPosition(1); // rotator starter position
////
////            }
////            if (gamepad1.dpad_down) {
////                robot.intakeRotatorServo.setPosition(0.68); // rotator perpendicular position
////            }
////
////            if (gamepad1.x) {
////                robot.intakeServoSpinner.setPosition(0.5); // spinner abs middle orientaiton 0.28 extended
////
////            }
////            if (gamepad1.b) {
////                robot.intakeServoSpinner.setPosition(0.16); // spinner abs up orientaiton
////            }
////
////            if (gamepad1.a) {
////                robot.intakeServoGrip.setPosition(0.3433); // gripper abs open position
////
////            }
////            if (gamepad1.y) {
////                robot.intakeServoGrip.setPosition(0.15); // gripper abs closed position
////            }
//
//
//            // Telemetry updates
//            telemetry.addData("intake gripper position", robot.intakeServoGrip.getPosition());
//            telemetry.addData("intake spinner position", robot.intakeServoSpinner.getPosition());
//            telemetry.addData("intake rotator position", robot.intakeRotatorServo.getPosition());
//            telemetry.addData("intake rotator position", robot.extensionMotor.getCurrentPosition());
//
//            telemetry.update();
//        }
//    }
//}
