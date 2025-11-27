package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double arm_appliedVolts;
    public double arm_currentAmp;
    public double arm_position;
    public double arm_goal;
    public double arm_angularVelocity;
    public boolean arm_atSetpoint;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void goToSetpoint(double target) {}
}
