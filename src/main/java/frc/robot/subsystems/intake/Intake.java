// Copyright 2021-2024 FRC 6328
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

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
  public static final class Constants {
    public static final double GEAR_RATIO = 1.0;
  }

  private static enum State {
    STOPPED,
    INTAKE,
    EJECT,
    SYSID
  }

  private final LoggedNetworkNumber intakeInput =
      new LoggedNetworkNumber("Intake/Intake Speed", 1000.0);
  private final LoggedNetworkNumber ejectInput =
      new LoggedNetworkNumber("Intake/Eject Speed", -500.0);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final Debouncer debouncer = new Debouncer(0.2);
  private final SysIdRoutine sysId;

  @AutoLogOutput(key = "Intake/State")
  private State state = State.STOPPED;

  @AutoLogOutput(key = "Intake/Setpoint")
  private double setpoint = 0.0;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (frc.robot.Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.00185); // TODO TS: SysId
        io.configurePID(0.0001, 0.0, 0.0); // TODO TS: SysId
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    switch (state) {
      case INTAKE:
        runVelocity(intakeInput.get());
        break;
      case EJECT:
        runVelocity(ejectInput.get());
        break;
      case STOPPED:
        stopIntake();
        break;
      case SYSID:
        break;
    }
  }

  @AutoLogOutput(key = "Intake/Speed")
  private double getSpeed() {
    double result = 0;
    for (double value : inputs.velocitiesRadPerSec) {
      result += value;
    }
    result /= inputs.velocitiesRadPerSec.length;
    return Units.radiansPerSecondToRotationsPerMinute(result);
  }

  @AutoLogOutput(key = "Intake/Error")
  private double getError() {
    return getSpeed() - setpoint;
  }

  @AutoLogOutput(key = "Intake/AtSpeed")
  private boolean atSpeed() {
    return Math.abs(getError()) < 100.0;
  }

  @AutoLogOutput(key = "Intake/OnTarget")
  private boolean onTarget() {
    return debouncer.calculate(atSpeed());
  }

  /** Run open loop at the specified voltage. */
  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    setpoint = velocityRPM;
  }

  /** Stops the intake. */
  private void stopIntake() {
    state = State.STOPPED;
    io.stop();
  }

  /** Returns a command to run the intake at a set state. */
  private Command stateCommand(State state) {
    return Commands.sequence(
        runOnce(() -> this.state = state),
        Commands.waitSeconds(0.25),
        run(() -> {}).until(this::onTarget));
  }

  /** Returns a command to run the intake at intake state. */
  public Command intake() {
    return stateCommand(State.INTAKE);
  }

  /** Returns a command to run the intake at eject state. */
  public Command eject() {
    return stateCommand(State.EJECT);
  }

  /** Returns a command to stop the intake. */
  public Command stop() {
    return runOnce(this::stopIntake);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(() -> state = State.SYSID), sysId.quasistatic(direction), stop());
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(runOnce(() -> state = State.SYSID), sysId.dynamic(direction), stop());
  }
}
