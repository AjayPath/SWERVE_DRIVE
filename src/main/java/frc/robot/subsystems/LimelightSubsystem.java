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
    limelight = NetworkTableInstance.getDefault().getTable("");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TAG ID", getTagID());
    SmartDashboard.putBoolean("Valid ID", hasValidTarget());
    SmartDashboard.putNumber("X Offset", getXOffset());
    SmartDashboard.putNumber("Y Offset", getYOffset());
    SmartDashboard.putNumber("Tag Area", getTargetArea());
  }

  public boolean hasValidTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1.0;
  }

  public double getXOffset() {
    return limelight.getEntry("tx").getDouble(0);
  }

  public double getYOffset() {
    return limelight.getEntry("ty").getDouble(0);
  }

  public double getTargetArea() {
    return limelight.getEntry("ta").getDouble(0);
  }

  public int getTagID() {
    return (int)limelight.getEntry("tid").getDouble(0);
  }

}
