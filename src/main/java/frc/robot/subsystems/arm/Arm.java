package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final PIDController pid = new PIDController(3, 0, 0.2);

  public Arm(ArmIO parameter_io) {
    this.io = parameter_io;
    pid.setTolerance(0.2);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // set variable to check setpoint to a method
    // which uses the pid object to see if the arm is actually within the setpoint range
    inputs.arm_atSetpoint = pid.atSetpoint();
    Logger.processInputs("Arm", inputs);
  }

  // create commands which we will schedule later

  public Command RawControl(double volts) {
    return run(() -> io.setVoltage(volts));
  }

  public Command goToSetpoint(double target) {
    return run(
        () -> {
          var out = pid.calculate(inputs.arm_position, target);
          Logger.recordOutput("Arm/Voltage", out);
          Logger.recordOutput("Arm/Error", pid.getError());
          io.setVoltage(out);
          Logger.recordOutput("Arm/Target", target);

        });
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0.0));
  }
}
