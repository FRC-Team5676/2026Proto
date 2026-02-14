package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AimerSubsystem extends SubsystemBase {

  public double m_TargetRadians;

  private final int m_canId = 54;

  private final TalonFX m_driveMotor;

 /*  private final double trayUpPosition = 600;
  private final double trayDownPosition = 1400;
  private final double coralPlacePosition = 2800; */

  public AimerSubsystem() {
    // Drive Motor setup
    m_driveMotor = new TalonFX(m_canId);
    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration());

  }

  @Override
  public void periodic() {
  }

  public void rotateAimer(double power) {
    m_driveMotor.set(power);
  }

  public void stopAimer() {
    m_driveMotor.set(0);
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