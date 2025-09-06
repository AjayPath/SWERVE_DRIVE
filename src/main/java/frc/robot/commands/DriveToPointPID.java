// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPointPID extends Command {
  /** Creates a new DriveToPointPID. */
  private final DriveSubsystem m_drive;
  private final Pose targetPose;

  private final double tolerance = 0.25;
  private double translationMag;
  private double rVel;

  private final double maxVel = 1.5;
  private final double maxRVel = 1.0;
  private final double MAX_SPEED = 3.0;
  private final double MAX_ROT_SPEED = 1.0;

  private Pose currentPose;
  private double distance;
  private double driveSpeed;
  private double turnSpeed;
  private double angleDiff;

  private final APPID drivePID;
  private final APPID turnPID;

  public DriveToPointPID(DriveSubsystem m_drive, Pose targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.targetPose = targetPose;
    addRequirements(m_drive);

    drivePID = new APPID(0.015, 0, 0, 0.05);
    //drivePID.setMaxOutput(1.5);
    drivePID.setDesiredValue(0);

    turnPID = new APPID(0.015, 0, 0, 0.05);
    //turnPID.setMaxOutput(1);
    turnPID.setDesiredValue(0);
    //turnPID.setDesiredValue(targetPose.GetAngleValue());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID.resetErrorSum();
    turnPID.resetErrorSum();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // currentPose = m_drive.getCustomPose();

    // Vector delta = targetPose.Subtract(currentPose);

    // distance = Math.sqrt(delta.GetXValue() * delta.GetXValue() + delta.GetYValue() * delta.GetYValue());

    // angleDiff = Calculations.signedAngleDifference(targetPose.GetAngleValue(), currentPose.GetAngleValue());

    // driveSpeed = -drivePID.calcPID(distance);
    // turnSpeed = -turnPID.calcPID(angleDiff);

    // // Compute and set speeds
    // if (distance > 1e-6) {

    //   double unitX = delta.GetXValue() / distance;
    //   double unitY = delta.GetYValue() / distance;

    //   double xSpeed = unitX * driveSpeed;
    //   double ySpeed = unitY * driveSpeed;

    //   m_drive.drive(xSpeed, ySpeed, turnSpeed, true);

    // } else {
    //   m_drive.drive(0, 0, 0, true);
    // }

    currentPose = m_drive.getCustomPose();

    Vector difference = targetPose.Subtract(currentPose);
    distance = difference.GetMag();

    translationMag = -drivePID.calcPID(distance);
    translationMag = Math.min(translationMag, maxVel);
    //translationMag *= 0.15;

    if (distance < 0) {
      translationMag = -translationMag;
    }

    double xVel = translationMag * Math.cos(difference.GetAngle().getRadians());
    double yVel = translationMag * Math.sin(difference.GetAngle().getRadians());

    xVel = Math.max(-MAX_SPEED, Math.min(xVel, MAX_SPEED));
    yVel = Math.max(-MAX_SPEED, Math.min(yVel, MAX_SPEED));

    Rotation2d currentAngle = Rotation2d.fromDegrees(currentPose.GetAngleValue());
    Rotation2d targetAngle = Rotation2d.fromDegrees(targetPose.GetAngleValue());

    Rotation2d angleDifference = targetAngle.minus(currentAngle);

    Rotation2d normalizedDiff = Calculations.NormalizeAngle(angleDifference);

    //double currentAngle = currentPose.GetAngleValue();
    rVel = -turnPID.calcPID(normalizedDiff.getDegrees());
    rVel = Math.max(-maxRVel, Math.min(rVel, maxRVel));

    m_drive.drive(xVel, yVel, rVel, true);

    //updateDashboard(driveSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return drivePID.isDone() || distance <= tolerance;
    return distance < tolerance;
  }

  // private void updateDashboard(double speedOutput) {
  //   SmartDashboard.putNumber("Target X", targetPose.GetXValue());
  //   SmartDashboard.putNumber("Target Y", targetPose.GetYValue());
  //   SmartDashboard.putNumber("Current X", currentPose.GetXValue());
  //   SmartDashboard.putNumber("Current Y", currentPose.GetYValue());
  //   SmartDashboard.putNumber("Distance Error", distance);
  //   SmartDashboard.putNumber("PID Output", speedOutput);
  //   SmartDashboard.putNumber("PID Turn Output", turnSpeed);
  // }

}
