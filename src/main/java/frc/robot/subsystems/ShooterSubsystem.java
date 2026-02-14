package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  public double m_TargetRadians;

  private final int m_canId1 = 51; // Right shooter motor
  private final int m_canId2 = 52; // left shooter motor

  private final TalonFX m_driveMotor1;
  private final TalonFX m_driveMotor2;

 /*  private final double trayUpPosition = 600;
  private final double trayDownPosition = 1400;
  private final double coralPlacePosition = 2800; */

  public ShooterSubsystem() {
    // Drive Motor setup
    m_driveMotor1 = new TalonFX(m_canId1);
    m_driveMotor1.getConfigurator().apply(new TalonFXConfiguration());

    m_driveMotor2 = new TalonFX(m_canId2);
    m_driveMotor2.getConfigurator().apply(new TalonFXConfiguration());

  }

  @Override
  public void periodic() {
  }

  public void activateShooter() {
    //double shooterSpeed = 0.65; // Adjust this value as needed for the desired shooting speed
    m_driveMotor1.set(-0.67); // Negative for inverse rotation // Right
    m_driveMotor2.set(0.70); // Left
  }

  public void stopShooter() {
    m_driveMotor1.set(0);
    m_driveMotor2.set(0);
  }

  /*
  public void moveToUpPosition() {
    m_TargetRadians = trayUpPosition;
    setReferencePeriodic();
  }

  public void moveToDownPosition() {
    m_TargetRadians = trayDownPosition;
    setReferencePeriodic();
  }

  public void moveToCoralPlacePosition() {
    m_TargetRadians = coralPlacePosition;
    setReferencePeriodic();
  }

  public double getUpPosition() {
    return trayUpPosition;
  }

  public double getCoralPlacePosition() {
    return coralPlacePosition;
  }

  public void driveTray(double degrees) {
    m_TargetRadians += degrees * 10;
    setReferencePeriodic();
  }

  public double getActualDegrees() {
    return m_driveMotor.getSelectedSensorPosition();
  }

  public double getTargetDegrees() {
    return m_TargetRadians;
  }

  private void setReferencePeriodic() {
    m_TargetRadians = MathUtil.clamp(m_TargetRadians, trayUpPosition, coralPlacePosition);
    m_driveMotor.set(ControlMode.Position, m_TargetRadians);
  }
    */
}