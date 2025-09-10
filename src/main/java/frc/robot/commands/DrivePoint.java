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
public class DrivePoint extends Command {
  /** Creates a new DriveToPointPID. */
  private final DriveSubsystem m_drive;
  private final Pose targetPose;

  private final double tolerance = 0.05;
  private double translationMag;

  private final double maxVel = 1.5;
  private final double MAX_SPEED = 1.0;

  private Pose currentPose;
  private double distance;
  private double driveSpeed;

  private final APPID drivePID;

  public DrivePoint(DriveSubsystem m_drive, Pose targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.targetPose = targetPose;
    addRequirements(m_drive);

    drivePID = new APPID(0.015, 0, 0, 0.05);
    drivePID.setDesiredValue(0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID.resetErrorSum();
    drivePID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get the current Pose of the robot from APOdometry
    currentPose = m_drive.getCustomPose();

    // Calculate the delta between the target pose and current pose
    Vector delta = targetPose.Subtract(currentPose);
    
    // Get eucladian distance
    // x * x + y * y
    distance = delta.GetMag();

    // Use the translational mangintude to calculate the PID value
    translationMag = -drivePID.calcPID(distance);
    translationMag = Math.min(translationMag, maxVel);

    // Convert the translation magnitude into x and y velocity coordiates
    double xVel = translationMag * Math.cos(delta.GetAngle().getRadians());
    double yVel = translationMag * Math.sin(delta.GetAngle().getRadians());

    // Clamp the outputs with Max Speeds
    xVel = Math.max(-MAX_SPEED, Math.min(xVel, MAX_SPEED));
    yVel = Math.max(-MAX_SPEED, Math.min(yVel, MAX_SPEED));

    // Send the pid outputs to the drive command
    m_drive.drive(xVel, yVel, 0, true);

    // Update the dashboard display
    updateDashboard(driveSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance < tolerance;
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