// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SlewRateLimiter {

    private double maxRateLimit = 0.0;
    private double currRateLimit = 0.0;
    private double prevRateLimit = 0.0;
    private double jerkLimit = 0.0;
    private double prevVal = 0.0;
    private double prevTime = 0.0;
    private double elapsedTime = 0.0;
    private double currTime = 0.0;

    public SlewRateLimiter(double _rateLimit, double _jerkLimit) {
        maxRateLimit = _rateLimit;
        jerkLimit = _jerkLimit;
        currRateLimit = 0.0;
        prevVal = 0.0;
        prevTime = 0.0;
        currTime = 0.0;
    }

    public double CalculateSlewRate(double _input) {
        double output = 0.0;
        currTime = Timer.getFPGATimestamp();
        elapsedTime = currTime - prevTime;

        if (_input > (prevVal + currRateLimit * elapsedTime)) {
            output = prevVal + currRateLimit * elapsedTime;
            currRateLimit = Calculations.LimitOutput(currRateLimit + jerkLimit * elapsedTime, maxRateLimit);
        }
        else if (_input < (prevVal - currRateLimit * elapsedTime)) {
            output = prevVal - currRateLimit * elapsedTime;
            currRateLimit = Calculations.LimitOutput(currRateLimit + jerkLimit + elapsedTime, maxRateLimit);
        }
        else {
            output = _input;
            currRateLimit = (prevVal - _input) * elapsedTime;
        }

        prevTime = currTime;
        prevVal = output;
        return output;
    }

    public void ResetSlewRate(double _input) {
        prevVal = _input;
        prevTime = Timer.getFPGATimestamp();
    }

}
