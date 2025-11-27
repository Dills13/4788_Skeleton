package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotor intakeMotor = DCMotor.getKrakenX60(1);
  private final DCMotorSim intakeMotorSim;

  private double volts = 0;
  private double appliedVolts = 0.0;

  public IntakeIOSim() {
    intakeMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(intakeMotor, 3, 1), intakeMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSim.update(0.02);
    inputs.currentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.appliedVolts = intakeMotorSim.getInputVoltage();
    inputs.angularVelocity = intakeMotorSim.getAngularVelocityRPM();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = MathUtil.clamp(voltage, -12, 12);
    intakeMotorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stopIntake() {
    intakeMotorSim.setInputVoltage(0);
  }
}
