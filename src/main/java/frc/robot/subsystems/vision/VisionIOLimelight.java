// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final String name;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber tvSubscriber;
  private final DoubleSubscriber tidSubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber targetPoseRobotSpaceSubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private final Alert wrongNameAlert = new Alert("", AlertType.kError);
  private double lastNameCheckTime = Double.NEGATIVE_INFINITY;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.name = name;
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    tvSubscriber = table.getDoubleTopic("tv").subscribe(0.0);
    tidSubscriber = table.getDoubleTopic("tid").subscribe(-1.0);
    targetPoseRobotSpaceSubscriber =
        table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

    // The Limelight is on USB, so only the RIO can reach it. Forward its web ports (5800 stream,
    // 5801 web UI, etc.) through the RIO so the driver station can reach them at the RIO's address.
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, VisionConstants.limelightUsbIp, port);
    }

    // The Limelight's video is an MJPEG web stream (port 5800), NOT NetworkTables data. Registering
    // it with CameraServer publishes its URLs under /CameraPublisher so Elastic can show it. The
    // RIO's static IP comes first because mDNS (.local) names are unreliable on the FMS. Port 5800
    // is in the FMS team-use range (5800-5810), so it is not blocked. The Limelight publishes its
    // own /CameraPublisher/<name> entry pointing at its USB IP (unreachable from the driver
    // station), so this one needs a different name or the two overwrite each other.
    HttpCamera stream =
        new HttpCamera(
            name + "-rio",
            new String[] {
              "http://" + VisionConstants.roborioIp + ":5800", "http://roborio-1626-frc.local:5800"
            },
            HttpCameraKind.kMJPGStreamer);
    CameraServer.addCamera(stream);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the
    // last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // If we aren't hearing from the Limelight, help figure out why
    if (!inputs.connected) {
      checkForMisnamedLimelight();
    } else {
      wrongNameAlert.set(false);
    }

    // Update target observation (these were previously hard-coded to "no target")
    boolean hasTarget = inputs.connected && tvSubscriber.get() >= 1.0;
    double[] targetPose = targetPoseRobotSpaceSubscriber.get();
    double distanceMeters =
        hasTarget && targetPose.length >= 3
            ? Math.sqrt(
                targetPose[0] * targetPose[0]
                    + targetPose[1] * targetPose[1]
                    + targetPose[2] * targetPose[2])
            : Double.NaN;
    inputs.latestTargetObservation =
        new TargetObservation(
            hasTarget,
            hasTarget ? (int) tidSubscriber.get() : -1,
            distanceMeters,
            Rotation2d.fromDegrees(txSubscriber.get()),
            Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /**
   * Looks for Limelight tables in NetworkTables and raises an alert if the configured name doesn't
   * match any of them. Only runs about once a second while disconnected.
   */
  private void checkForMisnamedLimelight() {
    double now = Timer.getFPGATimestamp();
    if (now - lastNameCheckTime < 1.0) {
      return;
    }
    lastNameCheckTime = now;

    // Only count tables where a Limelight is actually publishing (our own orientation publisher
    // creates a table under the configured name even when no Limelight is there)
    var nt = NetworkTableInstance.getDefault();
    Set<String> limelightTables =
        nt.getTable("").getSubTables().stream()
            .filter(table -> table.startsWith("limelight"))
            .filter(table -> nt.getTopic("/" + table + "/tl").exists())
            .collect(Collectors.toSet());
    if (limelightTables.isEmpty()) {
      wrongNameAlert.setText(
          "No Limelight found on NetworkTables. Check that it is powered, on the robot network,"
              + " and has team number 1626 set in its web UI.");
    } else if (!limelightTables.contains(name)) {
      wrongNameAlert.setText(
          "Limelight name \""
              + name
              + "\" not found. Found: "
              + String.join(", ", limelightTables)
              + ". Update VisionConstants.limelightName.");
    } else {
      wrongNameAlert.setText(
          "Limelight \"" + name + "\" is on NetworkTables but has stopped updating.");
    }
    wrongNameAlert.set(true);
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
