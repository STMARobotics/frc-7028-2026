package com.frc7028.physics.sim;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;

// https://cad.onshape.com/documents/8a691e28680da30504859fce/w/c6aa636fb23edb3f1e272fb1/e/f4e47c668796f504844c94a0 

public class RebuiltMetrics {
  public FieldMetrics RebuiltMetrics() {
    Translation3d origin = Translation3d.kZero;

    Translation3d blueOrigin = new Translation3d(Meters.of(-8.267700), Meters.of(-4.013645), Meters.zero());
    Translation3d redOrigin = new Translation3d(Meters.of(8.269288), Meters.of(3.968750), Meters.zero());

    Distance ceilingHeight = Feet.of(25);

    Region2d fieldPlane = new Region2d(blueOrigin.toTranslation2d(), redOrigin.toTranslation2d());

    Translation3d leftBottomBlueBaseVertex = new Translation3d(
        Inches.of(120.000000),
        Inches.of(23.500000),
        Inches.zero());

    Translation3d leftTopRightBaseVertex = new Translation3d(
        Inches.of(166.638780),
        Inches.of(-23.500000),
        Inches.of(59.397047));
    Region3d blueBottomBase = new Region3d(leftBottomBlueBaseVertex, leftTopRightBaseVertex);

    Region3d fieldRegion3D = new Region3d(
        blueOrigin,
        redOrigin.plus(new Translation3d(Feet.zero(), Feet.zero(), ceilingHeight)));

    ArrayList<Region3d> obstacles = new ArrayList<>();
    obstacles.add(blueBottomBase);

    ArrayList<Region3d> rejectRegions = new ArrayList<>();

    Translation3d blueTargetBottom = new Translation3d();
    Translation3d blueTargetTop = new Translation3d();

    Region3d targetRegion = new Region3d(blueTargetBottom, blueTargetTop);

    double startingHeight = Inches.of(100).baseUnitMagnitude();

    return new FieldMetrics(fieldPlane, fieldRegion3D, obstacles, rejectRegions, targetRegion, startingHeight);
  }
}