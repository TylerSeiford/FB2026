package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Arm extends SubsystemBase {
  public static final class Constants {
    // https://www.reca.lc/arm?armMass=%7B%22s%22%3A8%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A8%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A60%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A150%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%20Vortex%22%7D&ratio=%7B%22magnitude%22%3A53.166%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
    public static final double GEAR_RATIO = 5.0 * 5.0 / 12.0 * 26.0;

    public static final Rotation2d MINIMUM = Rotation2d.fromDegrees(-15.0);
    public static final Rotation2d MAXIMUM = Rotation2d.fromDegrees(120.0);
  }

  private static enum State {
    STARTUP,
    INTAKE_MOVE,
    INTAKE_HOLD,
    STOW,
    SYSID
  }

  private final LoggedNetworkNumber intakeInput =
      new LoggedNetworkNumber("Arm/Intake Angle", -15.0);
  private final LoggedNetworkNumber stowInput = new LoggedNetworkNumber("Arm/Stow Angle", 120.0);

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmFeedforward ffModel;
  private final Debouncer debouncer = new Debouncer(0.2);
  private final SysIdRoutine sysId;

  @AutoLogOutput(key = "Arm/State")
  private State state = State.STARTUP;

  @AutoLogOutput(key = "Arm/Setpoint")
  private Rotation2d setpoint;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (frc.robot.Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new ArmFeedforward(0.1, 0.35, 3.74, 0.02);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new ArmFeedforward(0.0, 0.35, 3.74, 0.02);
        io.configurePID(25.0, 0.0, 0.0);
        break;
      default:
        ffModel = new ArmFeedforward(0.0, 0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    switch (state) {
      case STARTUP:
        // On init, set the setpoint to the current position
        if (setpoint == null) setAngle(inputs.position);
        break;
      case INTAKE_MOVE:
        // If we are moving to the intake position and have reached it, transition to holding
        if (onTarget()) {
          state = State.INTAKE_HOLD;
        } else {
          setAngle(Rotation2d.fromDegrees(intakeInput.get()));
        }
        break;
      case INTAKE_HOLD:
        // If we are holding the intake position and have moved away from it, transition back to
        // moving
        if (!onTarget()) {
          state = State.INTAKE_MOVE;
        }
        break;
      case STOW:
        setAngle(Rotation2d.fromDegrees(stowInput.get()));
        break;
      case SYSID:
        // TODO
        break;
      default:
        break;
    }

    double ffVolts = ffModel.calculate(inputs.position.getRadians(), 0.0);
    Logger.recordOutput("Arm/ffVolts", ffVolts);

    if (state != State.INTAKE_HOLD) {
      io.setPosition(setpoint, ffVolts);
    }
  }

  @AutoLogOutput(key = "Arm/Error")
  private Rotation2d getError() {
    return inputs.position.minus(setpoint);
  }

  @AutoLogOutput(key = "Arm/AtPosition")
  private boolean atPosition() {
    return Math.abs(getError().getDegrees()) < 1.0;
  }

  @AutoLogOutput(key = "Arm/OnTarget")
  private boolean onTarget() {
    return debouncer.calculate(atPosition());
  }

  /** Run open loop at the specified voltage. */
  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  private void setAngle(Rotation2d angle) {
    if (angle.getDegrees() > Constants.MAXIMUM.getDegrees()) setpoint = Constants.MAXIMUM;
    else if (angle.getDegrees() < Constants.MINIMUM.getDegrees()) setpoint = Constants.MINIMUM;
    else setpoint = angle;
  }

  /** Returns a command to move the arm to the specified state. */
  private Command stateCommand(State state) {
    return Commands.sequence(
        runOnce(() -> this.state = state),
        Commands.waitSeconds(0.25),
        run(() -> {}).until(this::onTarget));
  }

  /** Returns a command to move the arm to intake state. */
  public Command intake() {
    return stateCommand(State.INTAKE_MOVE);
  }

  /** Returns a command to move the arm to stow state. */
  public Command stow() {
    return stateCommand(State.STOW);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(() -> state = State.SYSID), sysId.quasistatic(direction), stow());
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(runOnce(() -> state = State.SYSID), sysId.dynamic(direction), stow());
  }
}
