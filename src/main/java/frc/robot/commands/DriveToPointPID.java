// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPointPID extends Command {
  /** Creates a new DriveToPointPID. */
  private final DriveSubsystem m_drive;
  private final Pose targetPose;

  private final double tolerance = 0.5;

  private Pose currentPose;
  private double distance;
  private double speedOutput;

  private final APPID drivePID;

  public DriveToPointPID(DriveSubsystem m_drive, Pose targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.targetPose = targetPose;
    addRequirements(m_drive);

    drivePID = new APPID(0.01, 0, 0, 0.05);
    drivePID.setMaxOutput(1);
    drivePID.setDesiredValue(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID.resetErrorSum();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = m_drive.getCustomPose();

    Vector delta = targetPose.Subtract(currentPose);

    distance = Math.sqrt(delta.GetXValue() * delta.GetXValue() + delta.GetYValue() * delta.GetYValue());

    speedOutput = drivePID.calcPID(distance);

    // Compute and set speeds
    if (distance > 1e-6) {

      double unitX = delta.GetXValue() / distance;
      double unitY = delta.GetYValue() / distance;

      double xSpeed = unitX * speedOutput;
      double ySpeed = unitY * speedOutput;

      m_drive.drive(xSpeed, ySpeed, 0, true);

    } else {
      m_drive.drive(0, 0, 0, true);
    }

    updateDashboard(speedOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePID.isDone() || distance <= tolerance;
  }

  private void updateDashboard(double speedOutput) {
    SmartDashboard.putNumber("Target X", targetPose.GetXValue());
    SmartDashboard.putNumber("Target Y", targetPose.GetYValue());
    SmartDashboard.putNumber("Current X", currentPose.GetXValue());
    SmartDashboard.putNumber("Current Y", currentPose.GetYValue());
    SmartDashboard.putNumber("Distance Error", distance);
    SmartDashboard.putNumber("PID Output", speedOutput);
  }

}
