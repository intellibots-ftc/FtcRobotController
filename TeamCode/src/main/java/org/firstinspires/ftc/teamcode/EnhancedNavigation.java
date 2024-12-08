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

    // PIDF Constants
    private static final double kP = 0.03;  // Proportional gain
    private static final double kI = 0.001; // Integral gain
    private static final double kD = 0.01;  // Derivative gain
    private static final double kF = 0.15;  // Feed-forward term

    // Error thresholds
    private static final double POSITION_TOLERANCE_MM = 10.0;
    private static final double HEADING_TOLERANCE_DEG = 2.0;

    // Integral term limits
    private static final double MAX_INTEGRAL_ERROR = 200.0;
    
    // Movement limits
    private static final double MAX_DRIVE_POWER = 0.8;
    private static final double MIN_DRIVE_POWER = 0.1;

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
     * Navigate to target position using PIDF control
     * @param targetX X coordinate in mm
     * @param targetY Y coordinate in mm
     * @param targetHeading Heading in degrees
     * @return true if target reached within tolerance
     */
    public boolean navigateToPosition(double targetX, double targetY, double targetHeading) {
        odo.update();
        Pose2D currentPose = odo.getPosition();
        
        // Get current position
        double currentX = currentPose.getX(DistanceUnit.MM);
        double currentY = currentPose.getY(DistanceUnit.MM);
        double currentHeading = currentPose.getHeading(AngleUnit.DEGREES);

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
        integralXError = clamp(integralXError + xError * dt, -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);
        integralYError = clamp(integralYError + yError * dt, -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);
        integralHeadingError = clamp(integralHeadingError + headingError * dt, -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);

        // Calculate PIDF terms for each component
        double xPower = calculatePIDF(xError, integralXError, xDerivative);
        double yPower = calculatePIDF(yError, integralYError, yDerivative);
        double headingPower = calculatePIDF(headingError, integralHeadingError, headingDerivative);

        // Transform powers to robot-centric coordinates
        double robotAngle = Math.toRadians(currentHeading);
        double axialPower = xPower * Math.cos(robotAngle) + yPower * Math.sin(robotAngle);
        double lateralPower = -xPower * Math.sin(robotAngle) + yPower * Math.cos(robotAngle);

        // Apply power limits and deadband
        axialPower = clamp(axialPower, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        lateralPower = clamp(lateralPower, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        headingPower = clamp(headingPower, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

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
     * Calculate PIDF output for a single component
     */
    private double calculatePIDF(double error, double integral, double derivative) {
        return kP * error + kI * integral + kD * derivative + kF * Math.signum(error);
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