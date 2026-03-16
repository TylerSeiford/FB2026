package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class DriveConductor {
  private static enum Zones {
    RED_ALLIANCE_ZONE,
    NEUTRAL_ZONE,
    BLUE_ALLIANCE_ZONE
  }

  public static enum Modes {
    HUB,
    RELAY
  }

  private final Supplier<Pose2d> robotPoseSupplier;

  public DriveConductor(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @AutoLogOutput(key = "DriveConductor/Global/Shooter")
  private Pose2d shooter() {
    return robotPoseSupplier.get().transformBy(Constants.shooterOffset);
  }

  @AutoLogOutput(key = "DriveConductor/Global/Zone")
  private Zones zone() {
    if (robotPoseSupplier.get().getX() < FieldConstants.LinesVertical.hubCenter) {
      return Zones.BLUE_ALLIANCE_ZONE;
    } else if (robotPoseSupplier.get().getX() < FieldConstants.LinesVertical.oppHubCenter) {
      return Zones.NEUTRAL_ZONE;
    } else {
      return Zones.RED_ALLIANCE_ZONE;
    }
  }

  @AutoLogOutput(key = "DriveConductor/Global/Mode")
  public Modes mode() {
    return switch (zone()) {
      case RED_ALLIANCE_ZONE -> DriveCommands.isFlipped() ? Modes.HUB : Modes.RELAY;
      case BLUE_ALLIANCE_ZONE -> DriveCommands.isFlipped() ? Modes.RELAY : Modes.HUB;
      case NEUTRAL_ZONE -> Modes.RELAY;
    };
  }

  private static final Translation2d redHub =
      FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
  private static final Translation2d blueHub = FieldConstants.Hub.topCenterPoint.toTranslation2d();

  @AutoLogOutput(key = "DriveConductor/Global/Hub")
  private Translation2d hubGlobal() {
    return DriveCommands.isFlipped() ? redHub : blueHub;
  }

  @AutoLogOutput(key = "DriveConductor/Line/Hub")
  private Pose2d[] hubLine() {
    return new Pose2d[] {shooter(), new Pose2d(hubGlobal(), Rotation2d.kZero)};
  }

  @AutoLogOutput(key = "DriveConductor/Global/Relay")
  private Translation2d relayGlobal() {
    return new Translation2d(
        (DriveCommands.isFlipped()
            ? FieldConstants.LinesVertical.oppHubCenter
            : FieldConstants.LinesVertical.hubCenter),
        (robotPoseSupplier.get().getY() > FieldConstants.LinesHorizontal.center
            ? FieldConstants.LinesHorizontal.leftBumpMiddle
            : FieldConstants.LinesHorizontal.rightBumpMiddle));
  }

  @AutoLogOutput(key = "DriveConductor/Line/Relay")
  private Pose2d[] relayLine() {
    return new Pose2d[] {shooter(), new Pose2d(relayGlobal(), Rotation2d.kZero)};
  }

  @AutoLogOutput(key = "DriveConductor/Global/Target")
  private Translation2d targetGlobal() {
    return switch (mode()) {
      case HUB -> hubGlobal();
      case RELAY -> relayGlobal();
    };
  }

  @AutoLogOutput(key = "DriveConductor/Line/Target")
  private Pose2d[] targetLine() {
    return new Pose2d[] {shooter(), new Pose2d(targetGlobal(), Rotation2d.kZero)};
  }

  private static final Pose2d relative(
      Supplier<Translation2d> targetSupplier, Supplier<Pose2d> shooterSupplier) {
    Translation2d delta = targetSupplier.get().minus(shooterSupplier.get().getTranslation());
    Rotation2d targetAngle = delta.getAngle().minus(Constants.shooterOffset.getRotation());
    return new Pose2d(delta, targetAngle);
  }

  @AutoLogOutput(key = "DriveConductor/Relative/Hub")
  public Pose2d hubRelative() {
    return relative(this::hubGlobal, this::shooter);
  }

  @AutoLogOutput(key = "DriveConductor/Relative/Relay")
  public Pose2d relayRelative() {
    return relative(this::relayGlobal, this::shooter);
  }

  @AutoLogOutput(key = "DriveConductor/Relative/Target")
  public Pose2d targetRelative() {
    return relative(this::targetGlobal, this::shooter);
  }

  public Command aimAtHub(Drive drive, Rotation2d tolerance) {
    return DriveCommands.autoDriveAtTarget(drive, () -> hubRelative().getRotation(), tolerance);
  }

  public Command driveAtHub(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.joystickDriveAtAngle(
        drive, xSupplier, ySupplier, () -> hubRelative().getRotation());
  }

  public Command aimAtRelay(Drive drive, Rotation2d tolerance) {
    return DriveCommands.autoDriveAtTarget(drive, () -> relayRelative().getRotation(), tolerance);
  }

  public Command driveAtRelay(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.joystickDriveAtAngle(
        drive, xSupplier, ySupplier, () -> relayRelative().getRotation());
  }

  public Command aimAtTarget(Drive drive, Rotation2d tolerance) {
    return DriveCommands.autoDriveAtTarget(drive, () -> targetRelative().getRotation(), tolerance);
  }

  public Command driveAtTarget(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.joystickDriveAtAngle(
        drive, xSupplier, ySupplier, () -> targetRelative().getRotation());
  }

  public static Command driveAtAngle(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Rotation2d angle) {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> DriveCommands.isFlipped() ? angle.plus(Rotation2d.fromDegrees(180)) : angle);
  }
}
