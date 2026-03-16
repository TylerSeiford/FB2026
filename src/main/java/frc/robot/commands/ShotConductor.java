package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShotConductor {
  private final Supplier<DriveConductor.Modes> driveModeSupplier;
  private final Supplier<Pose2d> relativePoseSupplier;

  private final InterpolatingDoubleTreeMap relayMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hubMap = new InterpolatingDoubleTreeMap();

  public ShotConductor(
      Supplier<DriveConductor.Modes> driveModeSupplier, Supplier<Pose2d> relativePoseSupplier) {
    this.driveModeSupplier = driveModeSupplier;
    this.relativePoseSupplier = relativePoseSupplier;

    relayMap.put(0.0, 2000.0);
    relayMap.put(4.0, 3600.0);
    relayMap.put(6.75, 5500.0);
    relayMap.put(12.0, 6750.0);

    hubMap.put(2.032, 2750.0);
    hubMap.put(2.438, 2850.0);
    hubMap.put(3.657, 3150.0);
    hubMap.put(4.572, 3650.0);
    hubMap.put(5.7, 4500.0); // TODO TS: Estimate based on quadratic fit of previous points
  }

  @AutoLogOutput(key = "ShotConductor/Distance Meters")
  private double distanceMeters() {
    return relativePoseSupplier.get().getTranslation().getNorm();
  }

  @AutoLogOutput(key = "ShotConductor/Relay Shooter RPM")
  public double relayShooterRPM() {
    return relayMap.get(distanceMeters());
  }

  @AutoLogOutput(key = "ShotConductor/Hub Shooter RPM")
  public double hubShooterRPM() {
    return hubMap.get(distanceMeters());
  }

  @AutoLogOutput(key = "ShotConductor/Target Shooter RPM")
  public double targetShooterRPM() {
    return switch (driveModeSupplier.get()) {
      case RELAY -> relayShooterRPM();
      case HUB -> hubShooterRPM();
    };
  }
}
