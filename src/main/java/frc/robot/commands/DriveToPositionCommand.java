package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

/**
 * Command that drives the robot to a target X,Y position using a single PID controller
 * for distance. This uses the same vector-based approach as the C++ code - one PID
 * controls the magnitude of movement, and the direction comes from the error vector.
 * This command IGNORES rotation - the robot maintains whatever angle it currently has.
 */
public class DriveToPositionCommand extends Command {
    
    // Subsystem reference
    private final DriveSubsystem m_driveSubsystem;
    
    // Target position (only X and Y matter for this command)
    private final double m_targetX;
    private final double m_targetY;
    private final double m_tolerance;
    
    // Single PID Controller for distance (like the C++ version)
    private final APPID m_drivePID;
    
    // Tunable constants - adjust these for your robot
    private static final double kDriveP = 0.1;              // Proportional gain for distance
    private static final double kDriveI = 0.0;              // Integral gain
    private static final double kDriveD = 0.0;              // Derivative gain
    private static final double kMaxSpeed = 1.0;            // Speed multiplier (like C++ MAX_SPEED)
    private static final double kMaxTranslationSpeed = 0.5; // Max output speed (50% of robot max)
    private static final int kMinCyclesAtTarget = 10;       // Cycles to stay on target before done
    
    // Logging counter to control print frequency
    private int m_loopCounter = 0;
    private double m_initialDistance = 0.0;  // Store initial distance for progress tracking
    
    /**
     * Creates a command to drive to a specific X,Y position using vector-based control
     * @param driveSubsystem The drive subsystem to control
     * @param targetX Target X coordinate in meters (field coordinate system)
     * @param targetY Target Y coordinate in meters (field coordinate system)  
     * @param tolerance How close to get to target before considering "done" (meters)
     */
    public DriveToPositionCommand(DriveSubsystem driveSubsystem, double targetX, double targetY, double tolerance) {
        // Store parameters
        m_driveSubsystem = driveSubsystem;
        m_targetX = targetX;
        m_targetY = targetY;
        m_tolerance = tolerance;
        
        // Create single PID controller for distance (like C++ DrivePID)
        // The PID will control how fast we move toward the target
        m_drivePID = new APPID(kDriveP, kDriveI, kDriveD, tolerance);
        
        // Configure PID output limits
        m_drivePID.setMaxOutput(1.0); // Will be scaled by kMaxSpeed later
        
        // Set minimum cycles on target before considering done
        m_drivePID.setMinDoneCycles(kMinCyclesAtTarget);
        
        // Register that this command uses the drive subsystem
        addRequirements(driveSubsystem);
    }
    
    /**
     * Convenience constructor with default tolerance
     */
    public DriveToPositionCommand(DriveSubsystem driveSubsystem, double targetX, double targetY) {
        this(driveSubsystem, targetX, targetY, 0.05); // Default to 5cm tolerance
    }
    
    @Override
    public void initialize() {
        // Reset loop counter
        m_loopCounter = 0;
        
        // Set PID target to 0 distance (like C++ DrivePID->setDesiredValue(0.0))
        // We want zero error distance, so the PID will work to minimize distance
        m_drivePID.setDesiredValue(0.0);
        
        // Clear any previous PID state
        m_drivePID.reset();
        
        // Calculate and store initial distance for progress tracking
        m_initialDistance = getDistanceToTarget();
        
        // Get current position for logging
        Pose currentPose = m_driveSubsystem.getCustomPose();
        
        // Print startup message
        System.out.println("=== DriveToPosition (Vector Control) STARTED ===");
        System.out.printf("Current: (%.3f, %.3f) meters\n", 
                         currentPose.GetXValue(), currentPose.GetYValue());
        System.out.printf("Target: (%.3f, %.3f) meters\n", m_targetX, m_targetY);
        System.out.printf("Initial distance: %.3f meters\n", m_initialDistance);
        System.out.printf("Tolerance: %.3f meters\n", m_tolerance);
    }
    
    @Override
    public void execute() {
        // Get current robot position from odometry
        Pose currentPose = m_driveSubsystem.getCustomPose();
        
        // Calculate difference vector from current position to target
        // This is like C++ "difference = point.Subtract(Odometry->GetPose())"
        // But we need to reverse it since we want target - current
        double deltaX = m_targetX - currentPose.GetXValue();
        double deltaY = m_targetY - currentPose.GetYValue();
        Vector differenceVector = new Vector(deltaX, deltaY);
        
        // Get the distance (magnitude) and direction (angle) to target
        double distanceToTarget = differenceVector.GetMag();
        double directionAngle = differenceVector.GetAngle().getRadians();
        
        // Use PID to control translation magnitude based on distance error
        // Like C++ "translationMag = -MAX_SPEED * DrivePID->calcPID(difference.GetMag().value())"
        // Negative because PID target is 0, so positive distance should give negative output
        double translationMagnitude = -kMaxSpeed * m_drivePID.calcPID(distanceToTarget);
        
        // Limit the translation magnitude to our max speed
        translationMagnitude = Math.min(translationMagnitude, kMaxTranslationSpeed);
        translationMagnitude = Math.max(translationMagnitude, -kMaxTranslationSpeed);
        
        // Calculate X and Y velocities from magnitude and direction
        // Like C++ "xVel = translationMag * units::math::cos(difference.GetAngle())"
        double xVelocity = translationMagnitude * Math.cos(directionAngle);
        double yVelocity = translationMagnitude * Math.sin(directionAngle);
        
        // Send velocities to drive subsystem (field-relative)
        // rotSpeed = 0 because this command doesn't control rotation
        m_driveSubsystem.drive(xVelocity, yVelocity, 0.0, true);
        
        // Logging and SmartDashboard updates
        updateDashboard(currentPose, differenceVector, translationMagnitude, xVelocity, yVelocity);
        
        // Print debug info every 25 loops (about every 0.5 seconds at 50Hz)
        if (m_loopCounter % 25 == 0) {
            printDebugInfo(currentPose, differenceVector, translationMagnitude, xVelocity, yVelocity);
        }
        
        m_loopCounter++;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop all robot motion
        m_driveSubsystem.drive(0.0, 0.0, 0.0, true);
        
        // Final logging
        Pose finalPose = m_driveSubsystem.getCustomPose();
        System.out.println("=== DriveToPosition (Vector Control) ENDED ===");
        
        if (interrupted) {
            System.out.println("Command was INTERRUPTED");
        } else {
            System.out.println("Command completed successfully");
        }
        
        System.out.printf("Final position: (%.3f, %.3f)\n", 
                         finalPose.GetXValue(), finalPose.GetYValue());
        System.out.printf("Final distance from target: %.3f meters\n", getDistanceToTarget());
        System.out.printf("Distance traveled: %.3f meters\n", m_initialDistance - getDistanceToTarget());
        System.out.printf("Total loops executed: %d\n", m_loopCounter);
    }
    
    @Override
    public boolean isFinished() {
        // Command is finished when we're within tolerance
        // Like C++ "return difference.GetMag().value() < tolerance"
        return getDistanceToTarget() < m_tolerance;
    }
    
    /**
     * Updates SmartDashboard with current command status
     */
    private void updateDashboard(Pose currentPose, Vector differenceVector, double translationMag, 
                                double xVel, double yVel) {
        // Current position
        SmartDashboard.putNumber("DriveToPos/Current X", currentPose.GetXValue());
        SmartDashboard.putNumber("DriveToPos/Current Y", currentPose.GetYValue());
        
        // Target position
        SmartDashboard.putNumber("DriveToPos/Target X", m_targetX);
        SmartDashboard.putNumber("DriveToPos/Target Y", m_targetY);
        
        // Vector information
        SmartDashboard.putNumber("DriveToPos/Distance to Target", differenceVector.GetMag());
        SmartDashboard.putNumber("DriveToPos/Direction Angle (deg)", differenceVector.GetAngle().getDegrees());
        SmartDashboard.putNumber("DriveToPos/Translation Magnitude", translationMag);
        
        // Velocity outputs
        SmartDashboard.putNumber("DriveToPos/X Velocity", xVel);
        SmartDashboard.putNumber("DriveToPos/Y Velocity", yVel);
        SmartDashboard.putNumber("DriveToPos/Total Velocity", Math.sqrt(xVel*xVel + yVel*yVel));
        
        // Progress tracking
        double progressPercent = 0.0;
        if (m_initialDistance > 0.01) { // Avoid division by zero
            progressPercent = ((m_initialDistance - differenceVector.GetMag()) / m_initialDistance) * 100.0;
        }
        SmartDashboard.putNumber("DriveToPos/Progress %", progressPercent);
        
        // Status
        SmartDashboard.putBoolean("DriveToPos/Command Done", isFinished());
    }
    
    /**
     * Prints detailed debug information to console
     */
    private void printDebugInfo(Pose currentPose, Vector differenceVector, double translationMag, 
                               double xVel, double yVel) {
        System.out.printf("Loop %d - Pos:(%.3f,%.3f) Target:(%.3f,%.3f) Dist:%.3f Dir:%.1f° " +
                         "Mag:%.3f Vel:(%.3f,%.3f)\n",
                         m_loopCounter,
                         currentPose.GetXValue(), currentPose.GetYValue(),
                         m_targetX, m_targetY,
                         differenceVector.GetMag(),
                         differenceVector.GetAngle().getDegrees(),
                         translationMag,
                         xVel, yVel);
    }
    
    /**
     * Calculates current distance to target position
     * @return distance in meters
     */
    public double getDistanceToTarget() {
        Pose currentPose = m_driveSubsystem.getCustomPose();
        double deltaX = m_targetX - currentPose.GetXValue();
        double deltaY = m_targetY - currentPose.GetYValue();
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    
    /**
     * Gets the current direction to target
     * @return angle in degrees
     */
    public double getDirectionToTarget() {
        Pose currentPose = m_driveSubsystem.getCustomPose();
        double deltaX = m_targetX - currentPose.GetXValue();
        double deltaY = m_targetY - currentPose.GetYValue();
        Vector directionVector = new Vector(deltaX, deltaY);
        return directionVector.GetAngle().getDegrees();
    }
    
    /**
     * Gets the target X coordinate
     */
    public double getTargetX() {
        return m_targetX;
    }
    
    /**
     * Gets the target Y coordinate  
     */
    public double getTargetY() {
        return m_targetY;
    }
    
    /**
     * Gets the position tolerance
     */
    public double getTolerance() {
        return m_tolerance;
    }
}