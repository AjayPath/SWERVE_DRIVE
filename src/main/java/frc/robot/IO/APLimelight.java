// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IO;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.APTree;
import frc.robot.utils.Vector;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class APLimelight {

    private final DriveSubsystem m_drive;

    public APLimelight(DriveSubsystem m_drive) {
        this.m_drive = m_drive;
    }

}
