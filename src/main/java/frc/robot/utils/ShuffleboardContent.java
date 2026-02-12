// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.FuelIntakeSubsystem;

/** Add your docs here. */
public class ShuffleboardContent {

        static ShuffleboardLayout boolsLayout;

        public ShuffleboardContent() {

        }

        /* 

        SmartDashboard.putData("Swerve Drive", new Sendable() {
                @Override
                public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> frontLeftModule.getAngle().getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> frontLeftModule.getVelocity(), null);

                builder.addDoubleProperty("Front Right Angle", () -> frontRightModule.getAngle().getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> frontRightModule.getVelocity(), null);

                builder.addDoubleProperty("Back Left Angle", () -> backLeftModule.getAngle().getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> backLeftModule.getVelocity(), null);

                builder.addDoubleProperty("Back Right Angle", () -> backRightModule.getAngle().getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> backRightModule.getVelocity(), null);

                builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
                }
        });

        public static void initRotateArm(ArmSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Rotate Arm");

                drLayout1.addNumber("Angle", () -> drive.getRotateArmDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getRotateArmTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getMinRotateArmDegrees())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getMaxRotateArmDegrees())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initBallScrew(BallScrewSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Ball Screw");

                drLayout1.addNumber("Distance", () -> drive.getActualUnits())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Distance", () -> drive.m_positionUnits)
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Distance", () -> drive.getUpPosition())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Distance", () -> drive.getDownPosition())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initLinearArm(ArmSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Linear Arm");

                drLayout1.addNumber("Angle", () -> drive.getLinearArmDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getLinearArmTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getExtendedLinearArmDegrees())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getRetractedLinearArmDegrees())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initClimber(ClimberSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Climber");

                drLayout1.addNumber("Angle", () -> drive.getActualDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getMinDegrees())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getMaxDegrees())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initWrist(ArmSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Wrist");

                drLayout1.addNumber("Angle", () -> drive.getWristDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getWristTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getMinWristDegrees())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getMaxWristDegrees())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initTray(TraySubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Tray");

                drLayout1.addNumber("Angle", () -> drive.getActualDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getUpPosition())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getCoralPlacePosition())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }
        */

}
