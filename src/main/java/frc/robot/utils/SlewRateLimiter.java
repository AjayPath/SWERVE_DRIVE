// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Custom slew rate limiter with jerk control.
 * 
 * Purpose:
 * - Limits how quickly a value can change (rate limiting)
 * - Smoothly ramps the rate limit itself (jerk limiting)
 * - Prevents sudden acceleration changes that could tip the robot or strain mechanisms
 * 
 * How It Works:
 * - Rate Limit: Maximum change per second (e.g., 2.0 m/s²)
 * - Jerk Limit: How fast the rate limit can increase (e.g., 5.0 m/s³)
 * - Gradually increases acceleration from 0 to maxRateLimit
 * 
 * Example Use Case:
 * - Driver inputs full throttle instantly
 * - Instead of instant acceleration, robot smoothly ramps up
 * - Prevents wheel slip, tipping, and mechanical stress
 */
public class SlewRateLimiter {

  // ===========================================================================================
  // Rate Limiting Parameters
  // ===========================================================================================

  private double maxRateLimit;  // Maximum rate of change (units/second)
  private double currRateLimit; // Current rate of change being applied (starts at 0, ramps to max)
  private double jerkLimit;     // How fast the rate limit can increase (units/second²)

  // ===========================================================================================
  // State Tracking
  // ===========================================================================================

  private double prevVal;  // Last output value
  private double prevTime; // Timestamp of last calculation
  private double currTime; // Current timestamp
  private double elapsedTime; // Time since last calculation

  // ===========================================================================================
  // Constructor
  // ===========================================================================================

  /**
   * Creates a slew rate limiter with jerk control.
   * 
   * @param _rateLimit maximum rate of change (units per second)
   * @param _jerkLimit maximum rate of change of rate (units per second squared)
   */
  public SlewRateLimiter(double _rateLimit, double _jerkLimit) {
    maxRateLimit = _rateLimit;
    jerkLimit = _jerkLimit;
    currRateLimit = 0.0;
    prevVal = 0.0;
    prevTime = 0.0;
    currTime = 0.0;
  }

  // ===========================================================================================
  // Rate Limiting
  // ===========================================================================================

  /**
   * Applies slew rate limiting to an input value.
   * 
   * Algorithm:
   * 1. If input is increasing too fast: Limit output, gradually increase rate limit
   * 2. If input is decreasing too fast: Limit output, gradually increase rate limit
   * 3. If input is within limits: Pass through unchanged, update current rate
   * 
   * The rate limit itself ramps up smoothly (jerk limiting) to prevent sudden changes.
   * 
   * @param _input the desired input value
   * @return rate-limited output value
   */
  public double CalculateSlewRate(double _input) {
    double output = 0.0;
    currTime = Timer.getFPGATimestamp();
    elapsedTime = currTime - prevTime;

    // Input is increasing faster than allowed
    if (_input > (prevVal + currRateLimit * elapsedTime)) {
      // Limit output to maximum allowed increase
      output = prevVal + currRateLimit * elapsedTime;
      
      // Gradually increase the rate limit (jerk limiting)
      currRateLimit = Calculations.LimitOutput(
          currRateLimit + jerkLimit * elapsedTime,
          maxRateLimit);
    }
    // Input is decreasing faster than allowed
    else if (_input < (prevVal - currRateLimit * elapsedTime)) {
      // Limit output to maximum allowed decrease
      output = prevVal - currRateLimit * elapsedTime;
      
      // Gradually increase the rate limit (jerk limiting)
      currRateLimit = Calculations.LimitOutput(
          currRateLimit + jerkLimit * elapsedTime,
          maxRateLimit);
    }
    // Input is within rate limits
    else {
      // Pass through unchanged
      output = _input;
      
      // Update current rate based on actual change
      currRateLimit = Math.abs(prevVal - _input) / elapsedTime;
    }

    // Update state for next iteration
    prevTime = currTime;
    prevVal = output;
    
    return output;
  }

  // ===========================================================================================
  // Reset
  // ===========================================================================================

  /**
   * Resets the limiter to a specific value.
   * Use this when teleop starts or when you need to instantly jump to a new value.
   * 
   * @param _input the value to reset to
   */
  public void ResetSlewRate(double _input) {
    prevVal = _input;
    prevTime = Timer.getFPGATimestamp();
    currRateLimit = 0.0;
  }
}