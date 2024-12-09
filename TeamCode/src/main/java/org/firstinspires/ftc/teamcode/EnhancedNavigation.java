package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;

public class EnhancedNavigation {
    private RobotControl robot;
    private GoBildaPinpointDriver odo;
    private ElapsedTime timer;

    // Translation PIDF Constants
    private static final double TRANSLATION_KP = 0.03;  // Proportional gain for translation
    private static final double TRANSLATION_KI = 0.005; // Integral gain for translation
    private static final double TRANSLATION_KD = 0.003;  // Derivative gain for translation
    private static final double TRANSLATION_KF = 0.05;  // Feed-forward term for translation

    // Rotation PIDF Constants
    private static final double ROTATION_KP = 0.03;    // Proportional gain for rotation
    private static final double ROTATION_KI = 0.005;  // Integral gain for rotation
    private static final double ROTATION_KD = 0.003;    // Derivative gain for rotation
    private static final double ROTATION_KF = 0.05;     // Feed-forward term for rotation

    // Error thresholds
    private static final double POSITION_TOLERANCE_MM = 10.0;
    private static final double HEADING_TOLERANCE_DEG = 2.0;

    // Integral term limits
    private static final double MAX_TRANSLATION_INTEGRAL_ERROR = 200.0;
    private static final double MAX_ROTATION_INTEGRAL_ERROR = 45.0;

    // Movement limits
    private static final double MAX_TRANSLATION_POWER = 0.8;
    private static final double MAX_ROTATION_POWER = 0.6;
    private static final double MIN_TRANSLATION_POWER = 0.1;
    private static final double MIN_ROTATION_POWER = 0.08;

    // Error tracking
    private double lastXError = 0;
    private double lastYError = 0;
    private double lastHeadingError = 0;
    private double integralXError = 0;
    private double integralYError = 0;
    private double integralHeadingError = 0;

    public EnhancedNavigation(RobotControl robotControl, GoBildaPinpointDriver odometry) {
        this.robot = robotControl;
        this.odo = odometry;
        this.timer = new ElapsedTime();
    }

    /**
     * Navigate to target position using PIDF control with separate translation and rotation parameters
     */
    public boolean navigateToPosition(double targetX, double targetY, double targetHeading) {
        odo.update();
        Pose2D currentPose = odo.getPosition();

        // Get current position
        double currentX = currentPose.getX(DistanceUnit.MM);
        double currentY = -currentPose.getY(DistanceUnit.MM);
        double currentHeading = -currentPose.getHeading(AngleUnit.DEGREES);

        // Calculate errors
        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double headingError = normalizeAngle(targetHeading - currentHeading);

        // Calculate time delta
        double dt = timer.seconds();
        timer.reset();

        // Calculate derivative terms
        double xDerivative = dt > 0 ? (xError - lastXError) / dt : 0;
        double yDerivative = dt > 0 ? (yError - lastYError) / dt : 0;
        double headingDerivative = dt > 0 ? (headingError - lastHeadingError) / dt : 0;

        // Update integral terms with anti-windup
        integralXError = clamp(integralXError + xError * dt, -MAX_TRANSLATION_INTEGRAL_ERROR, MAX_TRANSLATION_INTEGRAL_ERROR);
        integralYError = clamp(integralYError + yError * dt, -MAX_TRANSLATION_INTEGRAL_ERROR, MAX_TRANSLATION_INTEGRAL_ERROR);
        integralHeadingError = clamp(integralHeadingError + headingError * dt, -MAX_ROTATION_INTEGRAL_ERROR, MAX_ROTATION_INTEGRAL_ERROR);

        // Calculate translation PIDF
        double xPower = calculateTranslationPIDF(xError, integralXError, xDerivative);
        double yPower = calculateTranslationPIDF(yError, integralYError, yDerivative);

        // Calculate rotation PIDF separately
        double headingPower = calculateRotationPIDF(headingError, integralHeadingError, headingDerivative);

        // Transform powers to robot-centric coordinates
        double robotAngle = Math.toRadians(currentHeading);
        double axialPower = xPower * Math.cos(robotAngle) + yPower * Math.sin(robotAngle);
        double lateralPower = -xPower * Math.sin(robotAngle) + yPower * Math.cos(robotAngle);

        // Apply power limits
        axialPower = clamp(axialPower, -MAX_TRANSLATION_POWER, MAX_TRANSLATION_POWER);
        lateralPower = clamp(lateralPower, -MAX_TRANSLATION_POWER, MAX_TRANSLATION_POWER);
        headingPower = clamp(headingPower, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);

        // Apply minimum power thresholds if needed
        if (Math.abs(axialPower) < MIN_TRANSLATION_POWER && Math.abs(axialPower) > 0.01) {
            axialPower = MIN_TRANSLATION_POWER * Math.signum(axialPower);
        }
        if (Math.abs(lateralPower) < MIN_TRANSLATION_POWER && Math.abs(lateralPower) > 0.01) {
            lateralPower = MIN_TRANSLATION_POWER * Math.signum(lateralPower);
        }
        if (Math.abs(headingPower) < MIN_ROTATION_POWER && Math.abs(headingPower) > 0.01) {
            headingPower = MIN_ROTATION_POWER * Math.signum(headingPower);
        }

        // Store errors for next iteration
        lastXError = xError;
        lastYError = yError;
        lastHeadingError = headingError;

        // Apply motor powers
        robot.controllerDrive(axialPower, lateralPower, headingPower, 100);

        // Check if target reached
        boolean atPosition = Math.abs(xError) < POSITION_TOLERANCE_MM &&
                Math.abs(yError) < POSITION_TOLERANCE_MM;
        boolean atHeading = Math.abs(headingError) < HEADING_TOLERANCE_DEG;

        return atPosition && atHeading;
    }

    /**
     * Calculate PIDF output for translation components
     */
    private double calculateTranslationPIDF(double error, double integral, double derivative) {
        return TRANSLATION_KP * error +
                TRANSLATION_KI * integral +
                TRANSLATION_KD * derivative +
                TRANSLATION_KF * Math.signum(error);
    }

    /**
     * Calculate PIDF output for rotation component
     */
    private double calculateRotationPIDF(double error, double integral, double derivative) {
        return ROTATION_KP * error +
                ROTATION_KI * integral +
                ROTATION_KD * derivative +
                ROTATION_KF * Math.signum(error);
    }

    /**
     * Normalize angle to -180 to 180 degrees
     */
    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Clamp value between min and max
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Reset integral terms and error tracking
     */
    public void resetController() {
        integralXError = 0;
        integralYError = 0;
        integralHeadingError = 0;
        lastXError = 0;
        lastYError = 0;
        lastHeadingError = 0;
        timer.reset();
    }
}
