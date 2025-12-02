package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface wristIO {

  @AutoLog
  public static class WristIOInputs {
    public double wrist_appliedvolts;
    public double wrist_curretnamp;
    public double wrist_position;
    public double wrist_goalOne;
    public double wrist_goalTwo;
    public double wrist_goalThree;
    public double wrist_angularVelocity;
    public boolean wrist_atSetpoint;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void goToSetpoint(double target) {}
}
