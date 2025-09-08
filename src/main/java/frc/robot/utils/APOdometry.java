// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MAXSwerveModule;

/** Add your docs here. */
public class APOdometry {

    private static APOdometry instance;
    private final Pigeon2 gyro;
    private final List<MAXSwerveModule> swerveMods;
    private final List<Pose> modulePoses;
    private final List<Vector> wheelOffsets;
    private final double[] lastWheelDistances;
    private Pose lastCenter;
    private double lastTime;
    private Vector lastVelocity;

    // Constructor
    private APOdometry(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
        lastWheelDistances = new double[swerveMods.size()];
        this.swerveMods = swerveMods;
        this.modulePoses = new ArrayList<>(swerveMods.size());
        this.gyro = gyro;

        double a = DriveConstants.kWheelBase / 2;
        double b = DriveConstants.kTrackWidth / 2;

        wheelOffsets = List.of(
            new Vector(a, b),
            new Vector(a, -b),
            new Vector(-a, b),
            new Vector(-a, -b)
        );

        // Set up all modules at (0,0) with current wheel angle
        for (int i = 0; i < swerveMods.size(); i++) {
            Rotation2d initialAngle = swerveMods.get(i).getPosition().angle;
            modulePoses.add(new Pose(0, 0, initialAngle));
        }

        lastCenter = new Pose(0, 0, gyro.getRotation2d());
        lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    }

    public static APOdometry getInstance(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
        if (instance == null) {
            instance = new APOdometry(swerveMods, gyro);
        }
        return instance;
    }

    // Update Values
    public void update() {
        for (int i = 0; i < swerveMods.size(); i++) {

            // Read mods reported pos and angle
            double currentDistance = swerveMods.get(i).getPosition().distanceMeters;
            double deltaDistance = currentDistance - lastWheelDistances[i];
            Rotation2d wheelAngle = swerveMods.get(i).getPosition().angle;
            lastWheelDistances[i] = currentDistance;
            
            // Convert distance into x/y comps
            // Multiply by 39.37 for inches
            Vector wheelVector = new Vector(deltaDistance * 39.37, wheelAngle);
            
            // Update the Pose for the wheel
            modulePoses.get(i).SetX(modulePoses.get(i).GetXValue() + wheelVector.GetXValue());
            modulePoses.get(i).SetY(modulePoses.get(i).GetYValue() + wheelVector.GetYValue());
            modulePoses.get(i).SetAngle(wheelAngle);
            

        }

        Pose currentCenter = calculateCenter();
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        double dt = currentTime - lastTime;
        if (dt > 0) {
            double dx = currentCenter.GetXValue() - lastCenter.GetXValue();
            double dy = currentCenter.GetYValue() - lastCenter.GetYValue();
    
            double mag = Math.sqrt(dx * dx + dy * dy) / dt;
            Rotation2d angle = new Rotation2d(Math.atan2(dy, dx));

            lastVelocity = new Vector(mag, angle);
        }

        lastCenter = currentCenter;
        lastTime = currentTime;
    }

    public List<Pose> getModulePoses() {
        return new ArrayList<>(modulePoses);
    }

    public Pose getModulePose (int index) {
        return modulePoses.get(index);
    }

    public void logWheelPoses() {
        for (int i = 0; i < swerveMods.size(); i++) {
            Pose p = modulePoses.get(i);
            SmartDashboard.putNumber("Wheel " + i + " X", p.GetXValue());
            SmartDashboard.putNumber("Wheel " + i + " Y", p.GetYValue());
            SmartDashboard.putNumber("Wheel " + i + " Angle", p.GetAngleValue());
        }
    }

    public Pose getCenterFromWheel (Pose wheelPose, Vector wheelOffset) {

        Rotation2d yaw = gyro.getRotation2d();
        //Rotation2d yaw = Rotation2d.fromDegrees(lastCenter.GetAngleValue());
        Rotation2d offsetAngle = yaw.plus(wheelOffset.GetAngle());
        double offsetX = wheelOffset.GetMag() * Math.cos(offsetAngle.getRadians());
        double offsetY = wheelOffset.GetMag() * Math.sin(offsetAngle.getRadians());

        Pose centerPose = new Pose(
            wheelPose.GetXValue() - offsetX,
            wheelPose.GetYValue() - offsetY,
            yaw
        );

        return centerPose;

    }

    public Pose calculateCenter() {

        double sumX = 0;
        double sumY = 0;
        double sumCos = 0;
        double sumSin = 0;

        for (int i = 0; i < modulePoses.size(); i++) {
            Pose centerFromThisWheel = getCenterFromWheel(modulePoses.get(i), wheelOffsets.get(i));
            sumX += centerFromThisWheel.GetXValue();
            sumY += centerFromThisWheel.GetYValue();
            sumCos += Math.cos(centerFromThisWheel.GetAngleValue() * 0.0174533);
            sumSin += Math.sin(centerFromThisWheel.GetAngleValue() * 0.0174533);
        }

        double avgX = sumX / modulePoses.size();
        double avgY = sumY / modulePoses.size();
        Rotation2d avgAngle = gyro.getRotation2d();
        

        return new Pose(avgX, avgY, avgAngle);

    }

    public void setPose(Pose newPose) {
        
        for (int i = 0; i < swerveMods.size(); i++) {

            Vector offset = wheelOffsets.get(i);
            double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();

            modulePoses.get(i).SetPose(
                offset.GetXValue() + newPose.GetXValue(),
                offset.GetYValue() + newPose.GetYValue(),
                //new Rotation2d(Math.toRadians(steerAngle))
                Rotation2d.fromDegrees(newPose.GetAngleValue())
            );

            lastCenter = newPose;

            lastWheelDistances[i] = swerveMods.get(i).getPosition().distanceMeters;

        }

        calculateCenter();

    }

    public void reset() {
        for (int i = 0; i < swerveMods.size(); i++) {
            Vector offset = wheelOffsets.get(i);
            double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();

            modulePoses.get(i).SetPose(
                offset.GetXValue(),
                offset.GetYValue(),
                new Rotation2d(Math.toRadians(steerAngle))
            );

            lastWheelDistances[i] = swerveMods.get(i).getPosition().distanceMeters;
        }
        lastCenter = new Pose(0, 0, gyro.getRotation2d());
    }

    public Pose getPose() {
        //return calculateCenter();
        return lastCenter;
    }

    public Vector getVelocity() {
        return lastVelocity;
    }

    public void logCenterPose() {
        Pose center = getPose();
        Vector robotVel = getVelocity();
        SmartDashboard.putNumber("Center X", center.GetXValue());
        SmartDashboard.putNumber("Center Y", center.GetYValue());
        SmartDashboard.putNumber("Center Angle", center.GetAngleValue());

        SmartDashboard.putNumber("Speed", robotVel.GetMag());
        SmartDashboard.putNumber("Direction", robotVel.GetAngle().getDegrees());
    }

}
