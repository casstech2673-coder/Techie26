// Copyright (c) 2025-2026 Littleton Robotics
// Adapted for FRC Team 2673 (Techie26), 2026 season.
// Original: https://github.com/Mechanical-Advantage/RobotCode2026Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Changes from original:
//   - Package changed to frc.robot
//   - Lombok annotations removed (expanded manually)
//   - Constants.disableHAL path logic replaced with Filesystem.getDeployDirectory()
//   - Alliance-aware helpers (getHubForAlliance, getPassFieldBearingDeg) added
//   - kBlueHubCenter / kRedHubCenter kept for backwards compatibility

package frc.robot;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station.
 *
 * <p>Field geometry is derived from the official 2026 AprilTag layout JSON files loaded at runtime
 * from {@code deploy/apriltags/welded/}. Hub positions are computed from AprilTag 26 (blue hub) and
 * AprilTag 4 (red hub).
 *
 * <p>To configure which tag subset the vision system uses, pass the desired {@link
 * AprilTagLayoutType} to your vision pipeline (e.g. {@link AprilTagLayoutType#HUB} for hub-only
 * localization).
 */
public final class FieldConstants {

  /** Which physical field construction variant to use when loading JSON files. */
  public static final FieldType fieldType = FieldType.WELDED;

  // ── AprilTag metadata ────────────────────────────────────────────────────
  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();

  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  // ── Field dimensions (from official layout JSON) ─────────────────────────
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();

  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  // ── Fuel dimensions ──────────────────────────────────────────────────────
  public static final double fuelDiameter = Units.inchesToMeters(5.91);

  // ── Alliance-aware hub positions (backwards-compatible API) ──────────────
  /** Blue hub center as a 2D field position (for shooter distance / turret aim). */
  public static final Translation2d kBlueHubCenter =
      new Translation2d(
          AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0,
          fieldWidth / 2.0);

  /** Red hub center as a 2D field position (symmetric; derived from AprilTag 4). */
  public static final Translation2d kRedHubCenter =
      new Translation2d(
          AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0,
          fieldWidth / 2.0);

  // ── Passing direction ─────────────────────────────────────────────────────
  public static final double kBluePassFieldBearingDeg = 180.0;
  public static final double kRedPassFieldBearingDeg = 0.0;

  // ── Alliance-aware helpers ────────────────────────────────────────────────

  /**
   * Returns the hub center {@link Translation2d} for the current alliance.
   *
   * <p>Falls back to blue when the alliance has not yet been assigned by FMS (e.g. practice).
   */
  public static Translation2d getHubForAlliance() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == Alliance.Red) {
      return kRedHubCenter;
    }
    return kBlueHubCenter;
  }

  /**
   * Returns the field-relative bearing (degrees) the turret should face to lob-pass toward the
   * alliance wall.
   *
   * <p>Falls back to blue (180°) when alliance is not yet assigned.
   */
  public static double getPassFieldBearingDeg() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == Alliance.Red) {
      return kRedPassFieldBearingDeg;
    }
    return kBluePassFieldBearingDeg;
  }

  // ── Field lines ───────────────────────────────────────────────────────────

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset).
   */
  public static class LinesVertical {
    public static final double center = fieldLength / 2.0;
    public static final double starting =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
    public static final double allianceZone = starting;
    public static final double hubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;
    public static final double neutralZoneNear = center - Units.inchesToMeters(120);
    public static final double neutralZoneFar = center + Units.inchesToMeters(120);
    public static final double oppHubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;
    public static final double oppAllianceZone =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getX();
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset).
   *
   * <p>NOTE: Left/right is always from the perspective of the alliance station.
   */
  public static class LinesHorizontal {
    public static final double center = fieldWidth / 2.0;

    // Right of hub
    public static final double rightBumpStart = Hub.nearRightCorner.getY();
    public static final double rightBumpEnd = rightBumpStart - RightBump.width;
    public static final double rightBumpMiddle = (rightBumpStart + rightBumpEnd) / 2.0;
    public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
    public static final double rightTrenchOpenEnd = 0;

    // Left of hub
    public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
    public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
    public static final double leftBumpMiddle = (leftBumpStart + leftBumpEnd) / 2.0;
    public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
    public static final double leftTrenchOpenStart = fieldWidth;
  }

  // ── Field elements ────────────────────────────────────────────────────────

  /** Hub related constants (the scoring target on the alliance side of the field). */
  public static class Hub {
    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(72.0); // includes catcher at top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Alliance-side reference points
    public static final Translation3d topCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            innerHeight);
    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Hub faces (AprilTag poses on each face)
    public static final Pose2d nearFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
    public static final Pose2d farFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
    public static final Pose2d rightFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
    public static final Pose2d leftFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();

    // Opposite-side reference points
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);
  }

  /** Left Bump (the ramp obstacle to the left of the hub, alliance-side perspective). */
  public static class LeftBump {
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.oppHubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.oppHubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Right Bump (the ramp obstacle to the right of the hub, alliance-side perspective). */
  public static class RightBump {
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.oppHubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.oppHubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Left Trench (the tunnel on the left wall, alliance-side perspective). */
  public static class LeftTrench {
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
  }

  /** Right Trench (the tunnel on the right wall, alliance-side perspective). */
  public static class RightTrench {
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
  }

  /** Tower (the climb structure at the center of the field). */
  public static class Tower {
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);
    public static final double uprightHeight = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Alliance-side reference points
    public static final Translation2d centerPoint =
        new Translation2d(
            frontFaceX, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
    public static final Translation2d leftUpright =
        new Translation2d(
            frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY()
                + innerOpeningWidth / 2.0
                + Units.inchesToMeters(0.75));
    public static final Translation2d rightUpright =
        new Translation2d(
            frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY()
                - innerOpeningWidth / 2.0
                - Units.inchesToMeters(0.75));

    // Opposite-side reference points
    public static final Translation2d oppCenterPoint =
        new Translation2d(
            fieldLength - frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());
    public static final Translation2d oppLeftUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY()
                + innerOpeningWidth / 2.0
                + Units.inchesToMeters(0.75));
    public static final Translation2d oppRightUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY()
                - innerOpeningWidth / 2.0
                - Units.inchesToMeters(0.75));
  }

  /** Depot (the fuel ball storage area near the alliance wall). */
  public static class Depot {
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    public static final Translation3d depotCenter =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
    public static final Translation3d leftCorner =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
    public static final Translation3d rightCorner =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
  }

  /** Outpost (the alliance-wall scoring station). */
  public static class Outpost {
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    public static final Translation2d centerPoint =
        new Translation2d(0, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
  }

  /** Fuel Pool (the central game piece reservoir). */
  public static class FuelPool {
    public static final double width = Units.inchesToMeters(181.9);
    public static final double depth = Units.inchesToMeters(71.9);

    public static final Translation2d nearLeftCorner =
        new Translation2d(fieldLength / 2.0 - depth / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(fieldLength / 2.0 - depth / 2.0, fieldWidth / 2.0 - width / 2.0);
  }

  // ── Enums ─────────────────────────────────────────────────────────────────

  /** Which physical field construction variant to load JSON files from. */
  public enum FieldType {
    /** Competition field (HQ) — welded construction. */
    HQ("welded"),
    /** AndyMark field kit. */
    ANDYMARK("andymark"),
    /** Welded construction (same as HQ). */
    WELDED("welded");

    private final String jsonFolder;

    FieldType(String jsonFolder) {
      this.jsonFolder = jsonFolder;
    }

    public String getJsonFolder() {
      return jsonFolder;
    }
  }

  /**
   * Selects which subset of AprilTags to include in the vision layout.
   *
   * <p>Use {@link #HUB} to restrict pose estimation to only hub-mounted tags (improves accuracy
   * when targeting the hub). Use {@link #OFFICIAL} for full-field localization.
   */
  public enum AprilTagLayoutType {
    /** All tags on the field (full global localization). */
    OFFICIAL("2026-official"),
    /** No tags (disables vision updates — useful for testing). */
    NONE("2026-none"),
    /** Hub tags only (tags 18, 20, 21, 26 on blue side; 4, ... on red side). */
    HUB("2026-hub"),
    /** Outpost tags only (tags near the alliance wall outpost). */
    OUTPOST("2026-outpost"),
    /** Tower/climb structure tags only. */
    TOWER("2026-tower");

    private final String name;
    private volatile AprilTagFieldLayout layout;
    private volatile String layoutString;

    AprilTagLayoutType(String name) {
      this.name = name;
    }

    /**
     * Lazily loads and caches the {@link AprilTagFieldLayout} for this tag subset.
     *
     * <p>Reads from {@code deploy/apriltags/<fieldType>/<name>.json}. Thread-safe via
     * double-checked locking.
     */
    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        synchronized (this) {
          if (layout == null) {
            try {
              Path p =
                  Path.of(
                      Filesystem.getDeployDirectory().getPath(),
                      "apriltags",
                      fieldType.getJsonFolder(),
                      name + ".json");
              layout = new AprilTagFieldLayout(p);
              layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (IOException e) {
              throw new RuntimeException("Failed to load AprilTag layout: " + name, e);
            }
          }
        }
      }
      return layout;
    }

    /**
     * Returns the JSON string representation of the layout (for NetworkTables / AdvantageScope).
     */
    public String getLayoutString() {
      if (layoutString == null) {
        getLayout();
      }
      return layoutString;
    }
  }
}
