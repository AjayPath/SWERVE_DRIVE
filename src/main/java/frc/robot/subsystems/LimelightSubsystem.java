// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  
  private final NetworkTable limelight;
  
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Valid ID", hasValidTarget());
    double distance = getDistanceFromTag(getTA());
    
    if (hasValidTarget()) {
      SmartDashboard.putNumber("TAG ID", getTID());
      SmartDashboard.putNumber("TX", getTX());
      SmartDashboard.putNumber("TY", getTY());
      SmartDashboard.putNumber("TA", getTA());
      SmartDashboard.putNumber("Distance From Tag", distance);
    }
  }

  public boolean hasValidTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1.0;
  }

  public double getTX() {
    return limelight.getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return limelight.getEntry("ty").getDouble(0);
  }

  public double getTA() {
    return limelight.getEntry("ta").getDouble(0);
  }

  public int getTID() {
    return (int)limelight.getEntry("tid").getDouble(0);
  }

  public double getDistanceFromTag(double ta) {
    double scale = 2.297; // m
    double distance = (scale / ta);
    return distance;
  }

}
