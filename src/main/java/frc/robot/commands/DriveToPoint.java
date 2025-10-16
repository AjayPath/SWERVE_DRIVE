// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

/**
 * Command to drive the robot to a specific field position and orientation.
 * 
 * Control Strategy:
 * - Translation: PID controller drives distance-to-target toward zero
 * - Rotation: Separate PID controller minimizes angle error
 * - Both controllers run simultaneously for smooth path following
 * 
 * Coordinate System:
 * - Field-relative control (x, y in meters, angle in degrees)
 * - Velocity components calculated from angle-to-target
 * - Shortest path rotation automatically calculated
 * 
 * Completion:
 * - Command finishes when both position and angle are within tolerance
 * - Odometry is updated to exact target pose on successful completion
 */
public class DriveToPoint extends Command {

  // ===========================================================================================
  // Dependencies
  // ===========================================================================================

  private final DriveSubsystem driveSubsystem;

  // ===========================================================================================
  // Control Parameters
  // ===========================================================================================

  private final APPID xPID;  // Controls translation speed based on distance error
  private final APPID yPID;
  private final APPID turnPID;   // Controls rotation speed based on angle error

  private final Pose targetPose;          // Desired final position and orientation
  private final double positionTolerance; // Acceptable position error (meters)
  private final double angleTolerance;    // Acceptable angle error (degrees)

  // ===========================================================================================
  // PID Constants - X Controller
  // ===========================================================================================

  private static final double kXP = 0.6;           // Proportional gain for drive
  private static final double kXI = 0.0;           // Integral gain for drive
  private static final double kXD = 0.05;          // Derivative gain for drive
  private static final double kXMaxDriveSpeed = 0.125;    // Maximum translation speed (0-1)

  // ===========================================================================================
  // PID Constants - X Controller
  // ===========================================================================================

  private static final double kYP = 0.6;           // Proportional gain for drive
  private static final double kYI = 0.0;           // Integral gain for drive
  private static final double kYD = 0.05;          // Derivative gain for drive
  private static final double kYMaxDriveSpeed = 0.125;    // Maximum translation speed (0-1)

  // ===========================================================================================
  // PID Constants - Rotation
  // ===========================================================================================

  private static final double kTurnP = 0.02;              // Proportional gain for rotation
  private static final double kTurnI = 0.0;               // Integral gain for rotation
  private static final double kTurnD = 0.0;               // Derivative gain for rotation
  private static final double kMaxRotationSpeed = .25;   // Maximum rotation speed (rad/s)

  // ===========================================================================================
  // Constructors
  // ===========================================================================================

  /**
   * Creates a command to drive to a specific field position and orientation.
   * 
   * @param driveSubsystem the drive subsystem
   * @param targetX target x position (meters)
   * @param targetY target y position (meters)
   * @param targetAngle target heading (degrees)
   * @param positionTolerance acceptable position error (meters)
   * @param angleTolerance acceptable angle error (degrees)
   */
  public DriveToPoint(DriveSubsystem driveSubsystem, double targetX, double targetY, 
                      double targetAngle, double positionTolerance, double angleTolerance) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = new Pose(targetX, targetY, targetAngle);
    this.positionTolerance = positionTolerance;
    this.angleTolerance = angleTolerance;

    // Initialize translation PID controller
    // Goal: Drive distance-to-target toward zero
    this.xPID = new APPID(kXP, kXI, kXD, positionTolerance);
    this.xPID.setMaxOutput(kXMaxDriveSpeed);

    this.yPID = new APPID(kYP, kYI, kYD, positionTolerance);
    this.yPID.setMaxOutput(kYMaxDriveSpeed);

    // Initialize rotation PID controller
    // Goal: Drive angle error toward zero
    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleTolerance);
    this.turnPID.setMaxOutput(kMaxRotationSpeed);

    addRequirements(driveSubsystem);
  }

  /**
   * Creates a command with default tolerances (0.1m position, 2° angle).
   * 
   * @param driveSubsystem the drive subsystem
   * @param targetX target x position (meters)
   * @param targetY target y position (meters)
   * @param targetAngle target heading (degrees)
   */
  public DriveToPoint(DriveSubsystem driveSubsystem, double targetX, double targetY, double targetAngle) {
    this(driveSubsystem, targetX, targetY, targetAngle, 0.1, 2.0);
  }

  // ===========================================================================================
  // Command Lifecycle
  // ===========================================================================================

  @Override
  public void initialize() {
    xPID.reset();
    yPID.reset();
    turnPID.reset();

    SmartDashboard.putNumber("TARGET_X", targetPose.GetXValue());
    SmartDashboard.putNumber("TARGET_Y", targetPose.GetYValue());
    SmartDashboard.putNumber("TARGET_ANGLE", targetPose.GetAngleValue());
  }

  @Override
  public void execute() {
    // Get current robot position
    Pose currentPose = driveSubsystem.getCustomPose();

    // ===========================================================================================
    // Translation Control
    // ===========================================================================================

    // Calculate vector from current position to target
    Vector difference = targetPose.Subtract(currentPose);
    double distanceToTarget = difference.GetMag();
    double xToTarget = difference.GetXValue();
    double yToTarget = difference.GetYValue();

    // PID drives distance toward zero
    // Setpoint = current distance, measurement = 0, so error = distanc
    xPID.setDesiredValue(xToTarget);
    yPID.setDesiredValue(yToTarget);

    double xSpeed = xPID.calcPID(0);
    double ySpeed = yPID.calcPID(0);

    // Convert speed to x/y velocity components using direction to target
    double angleToTargetRad = difference.GetAngle().getRadians();
    double xVel = xSpeed * Math.cos(angleToTargetRad);
    double yVel = ySpeed * Math.sin(angleToTargetRad);

    SmartDashboard.putNumber("X ERROR", difference.GetXValue());
    SmartDashboard.putNumber("Y ERROR", difference.GetYValue());

    // ===========================================================================================
    // Rotation Control
    // ===========================================================================================

    // Normalize both angles to [0, 360) for consistent comparison
    double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
    double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());

    turnPID.setDesiredValue(targetAngle);

    // Calculate shortest angular path (handles wrapping, e.g., 350° to 10°)
    double angleError = Calculations.shortestAngularDistance(targetAngle, currentAngle);
    SmartDashboard.putNumber("ANGLE_ERROR", angleError);

    // PID on angle error (negative sign for correct rotation direction)
    turnPID.setDesiredValue(angleError);
    double rotationOutput = turnPID.calcPID(0);
    SmartDashboard.putNumber("TARGET ANGLE", targetAngle);

    // ===========================================================================================
    // Drive Robot
    // ===========================================================================================

    // Apply both translation and rotation simultaneously
    // Field-relative mode ensures x/y velocities are relative to field
    driveSubsystem.drive(xVel, yVel, rotationOutput, false);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all robot motion
    driveSubsystem.drive(0.0, 0.0, 0.0, true);

    if (interrupted) {
      System.out.println("DrivePoint: Command interrupted");
    } else {
      // Command completed successfully - snap odometry to exact target
      // This prevents accumulated error from affecting subsequent paths
      driveSubsystem.setOdom(
          targetPose.GetXValue(),
          targetPose.GetYValue(),
          targetPose.GetAngleValue());

      System.out.println("DrivePoint: Command completed - Pose updated to target");
    }
  }

  @Override
  public boolean isFinished() {
    Pose currentPose = driveSubsystem.getCustomPose();

    // Check if position is within tolerance
    double distanceToTarget = targetPose.Subtract(currentPose).GetMag();
    boolean positionOnTarget = distanceToTarget <= positionTolerance;

    // Check if angle is within tolerance (using shortest path)
    double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
    double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
    double angleError = Math.abs(Calculations.shortestAngularDistance(targetAngle, currentAngle));
    boolean angleOnTarget = angleError <= angleTolerance;

    // Command finishes only when both position AND angle are satisfied
    return positionOnTarget && angleOnTarget;
  }

  // ===========================================================================================
  // Status Methods
  // ===========================================================================================

  /**
   * Gets the current distance from robot to target position.
   * Useful for monitoring command progress.
   * 
   * @return distance to target in meters
   */
  public double getDistanceToTarget() {
    Pose currentPose = driveSubsystem.getCustomPose();
    return targetPose.Subtract(currentPose).GetMag();
  }

  /**
   * Gets the current angle error (shortest path).
   * Useful for monitoring rotation progress.
   * 
   * @return angle error in degrees (positive = need to rotate CCW)
   */
  public double getAngleError() {
    Pose currentPose = driveSubsystem.getCustomPose();
    double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
    double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
    return Calculations.shortestAngularDistance(targetAngle, currentAngle);
  }
}