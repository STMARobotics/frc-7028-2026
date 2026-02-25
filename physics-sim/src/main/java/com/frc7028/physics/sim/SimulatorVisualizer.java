package com.frc7028.physics.sim;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import org.jzy3d.chart.Chart;
import org.jzy3d.chart.factories.ChartFactory;
import org.jzy3d.chart.factories.EmulGLChartFactory;
import org.jzy3d.maths.Coord3d;
import org.jzy3d.plot3d.primitives.Scatter;
import org.jzy3d.plot3d.rendering.canvas.Quality;

public class SimulatorVisualizer {
  private Chart baseChart;
  private Scatter scatterPlot;
  private ChartFactory chartFactory = new EmulGLChartFactory();

  public PointSpace trajectorySpace;

  public ArrayList<Trajectory> trajectories;

  public SimulatorVisualizer(Quality renderingQuality) {
    trajectories = new ArrayList<>();
    trajectorySpace = new PointSpace();
    baseChart = chartFactory.newChart(renderingQuality);

    baseChart.addMouse();
    baseChart.addKeyboard();
    baseChart.open();

    scatterPlot = new Scatter();

  };

  public Trajectory newTrajectory() {
    Trajectory trajectory = new Trajectory(trajectorySpace);
    this.trajectories.add(trajectory);
    return trajectory;
  }

  public void insertTrajectory(Trajectory trajectory) {
    this.trajectorySpace.unfinalizedPoints.addAll(trajectory.points);
  }

  public void putPoints() {
    scatterPlot.setData(this.trajectorySpace.unfinalizedPoints);
    scatterPlot.setDisplayed(true);
    scatterPlot.updateBounds();
    scatterPlot.setWidth(4);

    baseChart.add(scatterPlot);

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

    public Trajectory(PointSpace parentSpace) {
      this.points = new ArrayList<>();
      this.parentSpace = parentSpace;
    }

    public void step(Translation3d position) {
      this.points.add(new Coord3d(position.getX(), position.getY(), position.getZ()));
    }

    public void push() {
      this.parentSpace.insertCurve(this);
    }
  }
}
