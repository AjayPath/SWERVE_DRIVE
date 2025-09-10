package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;

/**
 * Turn to angle command using 0-360 degree normalization with shortest path calculation.
 * This avoids the -180/180 discontinuity that can confuse PID controllers.
 */
public class TurnToAngleCommand extends Command {
    
    private final DriveSubsystem m_driveSubsystem;
    private final double m_targetAngleDegrees;
    private final double m_toleranceDegrees;
    private final APPID m_turnPID;
    
    // Constants
    private static final double kTurnP = 0.05;
    private static final double kTurnI = 0.0;
    private static final double kTurnD = 0.005;
    private static final double kMaxRotationSpeed = 0.4;
    
    public TurnToAngleCommand(DriveSubsystem driveSubsystem, double targetAngleDegrees, double toleranceDegrees) {
        m_driveSubsystem = driveSubsystem;
        // Normalize target angle to [0, 360) range
        m_targetAngleDegrees = Calculations.NormalizeAngle360(targetAngleDegrees);
        m_toleranceDegrees = toleranceDegrees;
        
        // PID target is always 0 (no error)
        m_turnPID = new APPID(kTurnP, kTurnI, kTurnD, toleranceDegrees);
        m_turnPID.setMaxOutput(kMaxRotationSpeed);
        m_turnPID.setDesiredValue(0.0); // Always want zero error
        
        addRequirements(driveSubsystem);
    }
    
    public TurnToAngleCommand(DriveSubsystem driveSubsystem, double targetAngleDegrees) {
        this(driveSubsystem, targetAngleDegrees, 2.0);
    }
    
    @Override
    public void initialize() {
        m_turnPID.reset();
        System.out.println("TurnToAngle: Targeting " + m_targetAngleDegrees + " degrees");
    }
    
    @Override
    public void execute() {
        // Get current angle and normalize to [0, 360) range
        double currentAngle = Calculations.NormalizeAngle360(
            m_driveSubsystem.getCustomPose().GetAngleValue()
        );
        
        // Calculate shortest path error (positive = need to turn CCW, negative = CW)
        double angleError = Calculations.shortestAngularDistance(m_targetAngleDegrees, currentAngle);
        
        // Use PID on the error (target is always 0, current value is the error)
        double rotationOutput = -m_turnPID.calcPID(angleError);
        
        // Debug output (remove in production)
        // System.out.printf("Current: %.1f, Target: %.1f, Error: %.1f, Output: %.3f\n", 
        //     currentAngle, m_targetAngleDegrees, angleError, rotationOutput);
        
        // Drive with rotation only
        m_driveSubsystem.drive(0.0, 0.0, rotationOutput, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0.0, 0.0, 0.0, true);
        if (interrupted) {
            System.out.println("TurnToAngle: Command interrupted");
        } else {
            System.out.println("TurnToAngle: Command completed");
        }
    }
    
    @Override
    public boolean isFinished() {
        double angleError = Math.abs(getAngleError());
        boolean withinTolerance = angleError < m_toleranceDegrees;
        boolean pidDone = m_turnPID.isDone();
        
        return withinTolerance;
    }
    
    /**
     * Get the current angle error (shortest path)
     * @return error in degrees, positive = need CCW rotation, negative = need CW rotation
     */
    public double getAngleError() {
        double currentAngle = Calculations.NormalizeAngle360(
            m_driveSubsystem.getCustomPose().GetAngleValue()
        );
        return Calculations.shortestAngularDistance(m_targetAngleDegrees, currentAngle);
    }
    
    /**
     * Get the target angle
     * @return target angle in degrees (0-360)
     */
    public double getTargetAngle() {
        return m_targetAngleDegrees;
    }
    
    /**
     * Get the current angle
     * @return current angle in degrees (0-360)
     */
    public double getCurrentAngle() {
        return Calculations.NormalizeAngle360(
            m_driveSubsystem.getCustomPose().GetAngleValue()
        );
    }
}