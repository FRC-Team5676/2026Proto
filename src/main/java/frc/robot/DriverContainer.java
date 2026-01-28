package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class DriverContainer {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    private CommandJoystick driver;
    
        public DriverContainer(CommandJoystick driver) {
            this.driver = driver;

    }

    public double getX() {
        double value = -driver.getX(); // Drive left with negative X (left)

        double multiplier = 1;
        if (driver.button(11).getAsBoolean()) {
            multiplier = 0.1;
        }

        value = MathUtil.applyDeadband(value, Constants.LinearDeadband);
        value = Math.signum(value) * Math.pow(value, 2);
        return value * Constants.MaxLinearRate * multiplier;
    }

    public double getY() {
        double value = -driver.getY(); // Drive forward with negative Y (forward)

        double multiplier = 1;
        if (driver.button(11).getAsBoolean()) {
            multiplier = 0.1;
        }

        value = MathUtil.applyDeadband(value, Constants.LinearDeadband);
        value = Math.signum(value) * Math.pow(value, 2);
        return value * Constants.MaxLinearRate * multiplier;
    }

    public double getTwist() {
        double deadband;
        double value = -driver.getTwist(); // Drive counterclockwise with negative twist (CCW)

        double multiplierButton = 1;
        if (driver.button(11).getAsBoolean()) {
            multiplierButton = 0.1;
        }

        // x = Axis 4: 1 bottom to -1 top
        // translate to 1 bottom to 2 top using formula
        // y = -0.5x + 1.5
        double multiplier = -0.5 * driver.getRawAxis(3) + 1.5;

        if (Math.signum(value) <= 0) {
            // CCW
            deadband = 0.5; // larger on this side because of joystick sensitivity on CCW rotation
        } else if (Math.signum(value) > 0) {
            // CW
            deadband = 0.0;
        } else {
            return 0;
        }

        value = MathUtil.applyDeadband(value, deadband);
        value = Math.signum(value) * Math.pow(value, 2);
        return value * Constants.MaxAngularRate * multiplier * multiplierButton;
    }
}
