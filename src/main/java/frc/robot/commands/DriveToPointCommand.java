package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

public class DriveToPointCommand extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final APPID drivePID;
    private final Pose targetPose;
    private final double tolerance;
    
    /**
     * Creates a new DriveToPointCommand
     * @param driveSubsystem The drive subsystem
     * @param targetPose The target pose (x, y in meters, angle in degrees)
     * @param tolerance The tolerance for being "on target" (meters)
     */
    public DriveToPointCommand(DriveSubsystem driveSubsystem, Pose targetPose, double tolerance) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = new Pose(targetPose);
        this.tolerance = tolerance;
        
        // Create PID controller - tune these values
        this.drivePID = new APPID(2.0, 0.0, 0.1, tolerance);
        this.drivePID.setDesiredValue(0.0); // Want distance to be 0
        
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        drivePID.reset();
    }
    
    @Override
    public void execute() {
        // Get current position
        Pose currentPose = driveSubsystem.getCustomPose();
        
        // Calculate vector to target
        Vector difference = targetPose.Subtract(currentPose);
        double distanceToTarget = difference.GetMag();
        
        // Set the target distance to current distance (so PID drives it to 0)
        drivePID.setDesiredValue(distanceToTarget);
        
        // Calculate PID output - use 0 as current value so error = distance
        double speed = drivePID.calcPID(0.0);
        
        // Calculate x and y velocities
        double angleToTargetRad = difference.GetAngle().getRadians();
        double xVel = speed * Math.cos(angleToTargetRad);
        double yVel = speed * Math.sin(angleToTargetRad);
        
        // Drive the robot (field relative, no rotation)
        driveSubsystem.drive(xVel, yVel, 0.0, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, true);
    }
    
    @Override
    public boolean isFinished() {
        Pose currentPose = driveSubsystem.getCustomPose();
        double distanceToTarget = targetPose.Subtract(currentPose).GetMag();
        return distanceToTarget <= tolerance;
    }
}