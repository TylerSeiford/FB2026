package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterSupplier {
  public static final Transform2d shooterOffset =
      new Transform2d(
          Units.inchesToMeters(-8.0), Units.inchesToMeters(12.0), Rotation2d.fromDegrees(-90.0));

  private final Supplier<Pose2d> robotPoseSupplier;

  public ShooterSupplier(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @AutoLogOutput(key = "Superstructure/ShooterSupplier/Shooter")
  public Pose2d shooter() {
    return robotPoseSupplier.get().transformBy(shooterOffset);
  }
}
