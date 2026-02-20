package frc.robot;

import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

public class Constants {
    public static final double MaxLinearRate = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    
    public static final double LinearDeadband = 0.05; // Deadband for linear movement
    public static final double AngularDeadband = 0.1; // Deadband for angular movement
}
