package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class TargetSupplier {
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

  public TargetSupplier(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @AutoLogOutput(key = "Superstructure/TargetSupplier/Zone")
  private Zones zone() {
    if (robotPoseSupplier.get().getX() < FieldConstants.LinesVertical.hubCenter) {
      return Zones.BLUE_ALLIANCE_ZONE;
    } else if (robotPoseSupplier.get().getX() < FieldConstants.LinesVertical.oppHubCenter) {
      return Zones.NEUTRAL_ZONE;
    } else {
      return Zones.RED_ALLIANCE_ZONE;
    }
  }

  @AutoLogOutput(key = "Superstructure/TargetSupplier/Mode")
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

  @AutoLogOutput(key = "Superstructure/TargetSupplier/Hub")
  private Translation2d hub() {
    return DriveCommands.isFlipped() ? redHub : blueHub;
  }

  @AutoLogOutput(key = "Superstructure/TargetSupplier/Relay")
  private Translation2d relay() {
    return new Translation2d(
        (DriveCommands.isFlipped()
            ? FieldConstants.LinesVertical.oppHubCenter
            : FieldConstants.LinesVertical.hubCenter),
        (robotPoseSupplier.get().getY() > FieldConstants.LinesHorizontal.center
            ? FieldConstants.LinesHorizontal.leftBumpMiddle
            : FieldConstants.LinesHorizontal.rightBumpMiddle));
  }

  @AutoLogOutput(key = "Superstructure/TargetSupplier/Target")
  public Translation2d target() {
    return switch (mode()) {
      case HUB -> hub();
      case RELAY -> relay();
    };
  }
}
