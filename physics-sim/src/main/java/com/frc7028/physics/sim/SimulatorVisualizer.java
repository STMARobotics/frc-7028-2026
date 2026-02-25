package com.frc7028.physics.sim;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.ArrayList;
import org.jzy3d.chart.Chart;
import org.jzy3d.chart.factories.ChartFactory;
import org.jzy3d.chart.factories.EmulGLChartFactory;
import org.jzy3d.maths.doubles.Coord3D;
import org.jzy3d.plot3d.rendering.canvas.Quality;

public class SimulatorVisualizer {
  private Chart baseChart;
  private ChartFactory chartFactory = new EmulGLChartFactory();

  public ArrayList<Trajectory> trajectories;

  public SimulatorVisualizer(Quality renderingQuality) {
    trajectories = new ArrayList<>();
    baseChart = chartFactory.newChart(renderingQuality);

    baseChart.addMouse();
    baseChart.addKeyboard();
    baseChart.open();
  };

  public Trajectory newTrajectory() {
    Trajectory trajectory = new Trajectory();
    this.trajectories.add(trajectory);
    return trajectory;
  }

  public class Trajectory {
    ArrayList<Coord3D> points;

    public Trajectory() {
      this.points = new ArrayList<>();
    }

    public void step(Translation3d position) {
      this.points.add(new Coord3D(position.getX(), position.getY(), position.getZ()));
    }
  }
}
