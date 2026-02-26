package com.frc7028.physics.sim;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import org.jzy3d.chart.Chart;
import org.jzy3d.chart.factories.ChartFactory;
import org.jzy3d.chart.factories.EmulGLChartFactory;
import org.jzy3d.colors.Color;
import org.jzy3d.maths.Coord3d;
import org.jzy3d.plot3d.primitives.CubeComposite;
import org.jzy3d.plot3d.primitives.LineStrip;
import org.jzy3d.plot3d.rendering.canvas.Quality;
import org.jzy3d.plot3d.rendering.view.modes.ViewBoundMode;

public class SimulatorVisualizer {
  public Chart baseChart;
  private LineStrip lineStrip;
  private ChartFactory chartFactory = new EmulGLChartFactory();

  public PointSpace trajectorySpace;

  public ArrayList<Trajectory> trajectories;

  public SimulatorVisualizer(Quality renderingQuality, Region3d field) {
    trajectories = new ArrayList<>();
    trajectorySpace = new PointSpace();
    baseChart = chartFactory.newChart(renderingQuality);

    var mouse = baseChart.addMouse();
    var keyboard = baseChart.addKeyboard();
    baseChart.open(1000, 1000);

    var canvas = baseChart.getCanvas();

    lineStrip = new LineStrip();

    baseChart.add(lineStrip);

    baseChart.getView().setBoundMode(ViewBoundMode.MANUAL);
    baseChart.getView().setBoundManual(field.toBoundingBox());

    // IMousePickingController mouse = baseChart.getMousePicking();
    // mouse.getPickingSupport().addObjectPickedListener()
  };

  public static Coord3d translationToCoord(Translation3d translation3d) {
    return new Coord3d(translation3d.getX(), translation3d.getY(), translation3d.getZ());
  }

  void drawSegment(Translation3d pointA, Translation3d pointB, Color color) {
    var strip = new LineStrip();
    strip.add(translationToCoord(pointA));
    strip.add(translationToCoord(pointB));
    strip.setColor(color);
    strip.setDisplayed(true);

    baseChart.add(strip);
  }

  LineStrip newSegment(Translation3d pointA, Translation3d pointB, Color color) {
    var strip = new LineStrip();
    strip.add(translationToCoord(pointA));
    strip.add(translationToCoord(pointB));
    strip.setColor(color);
    strip.setDisplayed(false);

    return strip;
  }

  public Trajectory newTrajectory(Color trajectoryColor) {
    Trajectory trajectory = new Trajectory(trajectorySpace, trajectoryColor);
    this.trajectories.add(trajectory);
    return trajectory;
  }

  public void insertTrajectory(Trajectory trajectory) {
    this.trajectorySpace.unfinalizedPoints.addAll(trajectory.points);
  }

  public void putRegions(ArrayList<Region3d> regions, Color regionColor) {
    regions.forEach((Region3d region) -> {
      baseChart.add(new CubeComposite(region.toBoundingBox(), regionColor, regionColor));
    });
  }

  public void putRegion(Region3d region, Color regionColor) {
    baseChart.add(new CubeComposite(region.toBoundingBox(), regionColor, regionColor));
  }

  public class PointSpace {
    ArrayList<Coord3d> unfinalizedPoints;
    Object[] points;

    public PointSpace() {
      this.unfinalizedPoints = new ArrayList<>();
    }

    public void insertCurve(Trajectory trajectory) {
      this.unfinalizedPoints.addAll(trajectory.points);
    }

    public void finalize() {
      this.points = this.unfinalizedPoints.toArray();
    }
  }

  public class Trajectory {
    List<Coord3d> points;
    PointSpace parentSpace;
    Color color;

    public Trajectory(PointSpace parentSpace, Color trajectoryColor) {
      this.points = new ArrayList<>();
      this.parentSpace = parentSpace;
      this.color = trajectoryColor;
    }

    public void step(Translation3d position) {
      this.points.add(new Coord3d(position.getX(), position.getY(), position.getZ()));
    }

    public void put() {
      LineStrip line = new LineStrip();
      line.add(points);
      line.setColor(this.color);
      line.setWidth(12);
      line.setDisplayed(true);

      baseChart.add(line);
    }
  }
}
