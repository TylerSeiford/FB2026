package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotCalculator {
  public static class ShotData {
    public final double distanceMeters;
    public final double hubRPM;
    public final double relayRPM;
    public final double timeOfFlight;

    public ShotData(double distanceMeters, double hubRPM, double relayRPM, double timeOfFlight) {
      this.distanceMeters = distanceMeters;
      this.hubRPM = hubRPM;
      this.relayRPM = relayRPM;
      this.timeOfFlight = timeOfFlight;
    }
  }

  private final InterpolatingDoubleTreeMap relayMap =
      new InterpolatingDoubleTreeMap(); // meters to RPM
  private final InterpolatingDoubleTreeMap hubMap =
      new InterpolatingDoubleTreeMap(); // meters to RPM
  private final InterpolatingDoubleTreeMap tofMap =
      new InterpolatingDoubleTreeMap(); // meters to seconds

  public ShotCalculator() {
    relayMap.put(0.0, 2000.0);
    relayMap.put(4.0, 3600.0);
    relayMap.put(6.75, 5500.0);
    relayMap.put(12.0, 6750.0);

    hubMap.put(2.2, 2850.0);
    hubMap.put(2.5, 2875.0);
    hubMap.put(3.0, 2925.0);
    hubMap.put(3.5, 3100.0);
    hubMap.put(4.0, 3300.0);
    hubMap.put(5.0, 3725.0);
    hubMap.put(6.0, 4400.0);

    tofMap.put(2.5, .61); // desmos calculated thingy
    tofMap.put(3.0, .75); // tested 4/16/26
    tofMap.put(3.5, .88); // tested 4/16/26
    tofMap.put(4.0, 1.03); // tested 4/16/26
    tofMap.put(4.5, 1.17); // desmos calculated thingy
    tofMap.put(5.0, 1.31); // desmos calculated thingy
    tofMap.put(5.5, 1.45); // desmos calculated thingy
    tofMap.put(6.0, 1.59); // desmos calculated thingy
    tofMap.put(6.5, 1.73); // desmos calculated thingy
    tofMap.put(7.0, 1.87); // desmos calculated thingy
    tofMap.put(7.5, 2.01); // desmos calculated thingy
    tofMap.put(8.0, 2.15); // desmos calculated thingy
  }

  public ShotData calculateShot(double distanceMeters) {
    return new ShotData(
        distanceMeters,
        hubMap.get(distanceMeters),
        relayMap.get(distanceMeters),
        tofMap.get(distanceMeters));
  }
}
