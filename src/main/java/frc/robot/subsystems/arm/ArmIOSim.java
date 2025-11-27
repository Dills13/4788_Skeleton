package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmIOSim implements ArmIO {
  private final DCMotorSim armSim;
  private final DCMotor armMotor = DCMotor.getNEO(1);
  private double voltage = 0.0;

  public ArmIOSim() {
    armSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(armMotor, 0.02, 100), armMotor);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armSim.update(0.02);

    inputs.arm_appliedVolts = voltage;
    inputs.arm_currentAmp = armSim.getCurrentDrawAmps();
    inputs.arm_position = armSim.getAngularPositionRad();
    inputs.arm_angularVelocity = armSim.getAngularVelocityRPM();
    inputs.arm_goal = Math.toRadians(200);
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = MathUtil.clamp(voltage, -12, 12);
    armSim.setInputVoltage(voltage);
  }
}
