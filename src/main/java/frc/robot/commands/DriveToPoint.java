// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoint extends Command {
  /** Creates a new DriveToPoint. */
  private final DriveSubsystem m_drive;
  private final Pose targetPose;

  private Pose currentPose;
  private double distance;

  private final double tolerance = 0.5; // Inches
  private final double fixedSpeed = 0.5; // Speed we set the drive too

  public DriveToPoint(DriveSubsystem m_drive, Pose tagetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.targetPose = tagetPose;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get the current pose from the robot
    currentPose = m_drive.getCustomPose();

    // Calculate the delta using the target and current poses
    Vector delta = targetPose.Subtract(currentPose);
    
    // Calculate the distance between the two
    distance = Math.sqrt(delta.GetXValue() * delta.GetXValue() + delta.GetYValue() * delta.GetYValue());

    // Run execute until within a threshold  
    if (distance > tolerance) {

      // Calc the unit vector
      double unitX = delta.GetXValue() / distance;
      double unitY = delta.GetYValue() / distance;

      double xSpeed = unitX * fixedSpeed;
      double ySpeed = unitY * fixedSpeed;

      // Run the drive (no rotation yet)
      m_drive.drive(xSpeed, ySpeed, 0, true);

    }

    // Log all data to the display
    updateDashboard();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance <= tolerance;
  }

  private void updateDashboard() {
    
    // Display Target Outputs
    SmartDashboard.putNumber("Target X", targetPose.GetXValue());
    SmartDashboard.putNumber("Target Y", targetPose.GetYValue());
    SmartDashboard.putNumber("Taget Angle", targetPose.GetAngleValue());

    // Display Current Pose
    SmartDashboard.putNumber("Current X", currentPose.GetXValue());
    SmartDashboard.putNumber("Current Y", currentPose.GetYValue());
    SmartDashboard.putNumber("Current Angle", currentPose.GetAngleValue());

    // Display the Distance
    SmartDashboard.putNumber("X Distance", distance);

  }

}
