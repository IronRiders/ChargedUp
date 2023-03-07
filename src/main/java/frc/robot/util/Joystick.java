package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;

public class Joystick extends CommandJoystick {

    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(1.0 / 0.6); // 1 / x seconds to full
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(1.0 / 0.6);
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(1 / 0.4);

    private double drivespeed = 0.65;

    public Joystick(int port) {
        super(port);
    }

    /**
     * Deadbands a value, re-scales it, and applies a power.
     * 
     * @param value Value to adjust
     * @return -1 to 1
     */
    public static double scaledPowerDeadband(double value, double exp) {
        value = MathUtil.applyDeadband(value, Constants.DEADBAND);
        return Math.signum(value) * Math.pow(Math.abs(value), exp);
    }

    public double getLeftY(double exponent) {
        return -scaledPowerDeadband(getLeftY(1), exponent);
    }

    public double getLeftX(double exponent) {
        return -scaledPowerDeadband(getLeftX(1), exponent);
    }
    
    public double getRightY(double exponent) {
        return -scaledPowerDeadband(getRightY(1), exponent);
    }

    public double getRightX(double exponent) {
        return -scaledPowerDeadband(getRightX(1), exponent);
    }

    /**
     * Applies deadband math and rate limiting to left Y to give 'forward'
     * percentage.
     * Affected by controller drivespeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getForward() {
        return forwardLimiter.calculate(getLeftY(2) * drivespeed);
    }

    /**
     * Applies deadband math and rate limiting to left X to give 'strafe'
     * percentage.
     * Affected by controller drivespeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getStrafe() {
        return strafeLimiter.calculate(getLeftX(2) * drivespeed);
    }

    /**
     * Applies deadband math and rate limiting to right X to give 'turn' percentage.
     * Not affected by controller drivespeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getTurn() {
        return turnLimiter.calculate(getRightX(2) * Constants.TURN_SPEED);
    }
}
