package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double angularVelocity;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stopIntake() {}
}
