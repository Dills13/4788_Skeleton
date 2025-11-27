// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOComp;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private Intake intake;
  private Arm arm;
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Joystick keyboard = new Joystick(0);

  private final JoystickButton buttonOne = new JoystickButton(keyboard, 1);
  private final JoystickButton buttonTwo = new JoystickButton(keyboard, 2);
  private final JoystickButton buttonThree = new JoystickButton(keyboard, 3);
  private final JoystickButton buttonFour = new JoystickButton(keyboard, 4);

  public Robot() {

    // Optional Print-Outs
    System.out.println("=== Robot Constructor Start ===");
    System.out.println("RobotBase.isSimulation(): " + RobotBase.isSimulation());
    System.out.println("Current mode: " + Constants.currentMode);

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source

    // Constants.getMode();

    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        intake = new Intake(new IntakeIOComp());

        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        intake = new Intake(new IntakeIOSim());
        arm = new Arm(new ArmIOSim());

        break;
      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit logger
    Logger.start();

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.robotType) {
        case COMPBOT -> {
          intake = new Intake(new IntakeIOComp());
        }

        case DEVBOT -> {
          intake = new Intake(new IntakeIO() {});
        }

        case SIMBOT -> {
          intake = new Intake(new IntakeIOSim());
          arm = new Arm(new ArmIOSim());
        }
      }
    } else {
      intake = new Intake(new IntakeIO() {});
    }

    // Bind commands / Triggers
    // buttonOne.onTrue(intake.runIntake(6));
    // buttonTwo.onTrue(intake.stop());
    // buttonThree.onTrue(arm.goToSetpoint(Math.toRadians(500)));
    // buttonFour.onTrue(arm.stop());

    // controls to see arm jumping
    // If graphing rotational velocity, the scale of the graph with be very zoomed out as the
    // simulated motors produce large results
    buttonOne.onTrue(arm.goToSetpoint(Math.toRadians(50)));
    buttonTwo.onTrue(arm.goToSetpoint(Math.toRadians(200)));
    buttonThree.onTrue(arm.goToSetpoint(Math.toRadians(600)));
    buttonFour.onTrue(arm.stop());

    // arm.goToSetpoint(Math.toRadians(45)).schedule();
    // buttonFour.whileTrue(arm.RawControl(8));
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
