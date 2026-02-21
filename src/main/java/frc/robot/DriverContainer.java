package frc.robot;

import java.io.IOException;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.math.geometry.Pose3d;

public class DriverContainer {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    public double targetRange = 0.0;
    public double fuelRange = 0.0;
    private final SlewRateLimiter m_slewThrottle = new SlewRateLimiter(1.55);
    private final SlewRateLimiter m_slewStrafe = new SlewRateLimiter(1.55);
    private final SlewRateLimiter m_slewRot = new SlewRateLimiter(3);
    private final SlewRateLimiter m_slewVisionRot = new SlewRateLimiter(2.5);
    private final SlewRateLimiter m_slewVisionStrafe = new SlewRateLimiter(2.0);

    private CommandJoystick driver;

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded);
    

    // Limelight/PhotonVision
    private final PhotonCamera camera = new PhotonCamera("Camera_Module_v1");
    private final PhotonCamera colorCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    private final double VISION_TURN_kP = 1;
    private final double VISION_STRAFE_kP = 0.4;
    private final double VISION_DES_ANGLE_deg = 0.0; // Target angle offset
    private final double VISION_DES_RANGE_m = 2.0; // Target distance offset
    private double visionTurn = 0.0;
    private double visionStrafe = 0.0;
    private final double CAMERA_HEIGHT = 0.53; // meters

    // In your constructor or robotInit()

    // Load the 2024 field layout (adjust year as needed)
    // Define the robot-to-camera transform (adjust with your physical measurements)
    Transform3d robotToCam = new Transform3d(0.51, 0.12, 0.49, new Rotation3d(0, 0, 0));
    
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
    
    public DriverContainer() {
        SmartDashboard.putData("ShootPeriodic", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Double");

                builder.addDoubleProperty("Distance", () -> targetRange, null);
            }
        });

         SmartDashboard.putData("FuelPeriodic", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Double");

                builder.addDoubleProperty("Distance", () -> fuelRange, null);
            }
        });
    }

    public DriverContainer(CommandJoystick driver) {
        this.driver = driver;

        SmartDashboard.putData("ShootCommand", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Double");

                builder.addDoubleProperty("Distance", () -> targetRange, null);
            }
        });
    }

    public double getStrafe() {
        double value = -driver.getX(); // Drive left with negative X (left)

        double multiplier = 1;
        if (driver.button(11).getAsBoolean()) {
            multiplier = 0.1;
        }

        value = MathUtil.applyDeadband(value, Constants.LinearDeadband);
        value = Math.signum(value) * Math.pow(value, 2);
        value = value * Constants.MaxLinearRate * multiplier;
        double strafe_sl = m_slewStrafe.calculate(value);

        return value;
    }

    public double getThrottle() {
        double value = -driver.getY(); // Drive forward with negative Y (forward)

        double multiplier = 1;
        if (driver.button(11).getAsBoolean()) {
            multiplier = 0.1;
        }

        value = MathUtil.applyDeadband(value, Constants.LinearDeadband);
        value = Math.signum(value) * Math.pow(value, 2);
        value = value * Constants.MaxLinearRate * multiplier;
        double throttle_sl = m_slewThrottle.calculate(value);

        return value;
    }

    public double getRotation() {
        double value = -driver.getTwist(); // Drive counterclockwise with negative twist (CCW)

        double multiplierButton = 1;
        if (driver.button(11).getAsBoolean()) {
            multiplierButton = 0.1;
        }

        // x = Axis 4: 1 bottom to -1 top
        // translate to 1 bottom to 2 top using formula
        // y = -0.5x + 1.5
        double multiplier = -0.5 * driver.getRawAxis(3) + 1.5;

        value = MathUtil.applyDeadband(value, Constants.AngularDeadband);
        value = Math.signum(value) * Math.pow(value, 2);
        value = value * Constants.MaxAngularRate * multiplier * multiplierButton;
        double rotation_sl = m_slewRot.calculate(value);

        return value;
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
                    if (target.getFiducialId() == 25) {
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

        double rotation_sl = m_slewVisionRot.calculate(visionTurn);
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
                    if (target.getFiducialId() == 25) {
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT, 1.12, Units.degreesToRadians(0), Units.degreesToRadians(target.getPitch()));
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

        double strafe_sl = m_slewVisionStrafe.calculate(-visionStrafe);
        return visionStrafe;
    }

    public void getDistance() {
        // Read camera data
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 25) {
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT, 1.12, Units.degreesToRadians(0), Units.degreesToRadians(target.getPitch()));
                        
                    }
                }
            }
        }
        else {
            targetRange = 99.0;
        }
    }


    // for fuel: *******not complete******

    
    public void getFuelDistance() {
        // Read camera data
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 9) {
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT, 1.12, Units.degreesToRadians(0), Units.degreesToRadians(target.getPitch()));
                        
                    }
                }
            }
        }
        else {
            fuelRange = 99.0;
        }
    }
}
