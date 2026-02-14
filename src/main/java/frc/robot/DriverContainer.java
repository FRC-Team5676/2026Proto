package frc.robot;

import org.photonvision.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class DriverContainer {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    public double targetRange = 0.0;

    private CommandJoystick driver;

    // Limelight/PhotonVision
    private final PhotonCamera camera = new PhotonCamera("Camera_Module_v1");
    private final double VISION_TURN_kP = 0.05;
    private final double VISION_STRAFE_kP = 0.4;
    private final double VISION_DES_ANGLE_deg = 0.0; // Target angle offset
    private final double VISION_DES_RANGE_m = 2.0; // Target distance offset
    private double visionTurn = 0.0;
    private double visionStrafe = 0.0;

    public DriverContainer(CommandJoystick driver) {
        this.driver = driver;

        SmartDashboard.putData("Shooting", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Distance", () -> targetRange, null);
            }
        });
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

    public double getVisionTwist() {
        // Read camera data
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 9) {
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }

        // Auto-align to target when button held
        if (targetVisible) {
            visionTurn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.MaxAngularRate;
        } else {
            visionTurn = 0.0;
        }

        return visionTurn;
    }

    public double getVisionStrafe() {
        // Read camera data
        boolean targetVisible = false;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 9) {
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                                .64, 0.45, Units.degreesToRadians(0), Units.degreesToRadians(target.getPitch()));
                        targetVisible = true;
                    }
                }
            }
        }

        // Auto-align to target when button held
        if (targetVisible) {
            visionStrafe = (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Constants.MaxLinearRate;
        } else {
            visionStrafe = 0.0;
        }

        return -visionStrafe;
    }
}
