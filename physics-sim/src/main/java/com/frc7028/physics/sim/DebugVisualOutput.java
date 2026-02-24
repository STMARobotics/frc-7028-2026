package com.frc7028.physics.sim;

import org.jzy3d.chart.Chart;
import org.jzy3d.chart.factories.EmulGLChartFactory;
import org.jzy3d.colors.Color;
import org.jzy3d.colors.ColorMapper;
import org.jzy3d.colors.colormaps.ColorMapRainbow;
import org.jzy3d.maths.Range;
import org.jzy3d.plot3d.builder.Func3D;
import org.jzy3d.plot3d.builder.SurfaceBuilder;
import org.jzy3d.plot3d.builder.concrete.OrthonormalGrid;
import org.jzy3d.plot3d.primitives.Shape;
import org.jzy3d.plot3d.rendering.canvas.Quality;

public class DebugVisualOutput {
  private Chart chart; // Declare field

  public DebugVisualOutput() {
    // 1. Define the 3D function
    Func3D func = new Func3D((x, y) -> x * Math.sin(x * y));
    Range range = new Range(-3, 3);
    int steps = 80;

    // 2. Build the surface geometry
    Shape surface = new SurfaceBuilder().orthonormal(new OrthonormalGrid(range, steps), func);
    surface.setColorMapper(new ColorMapper(new ColorMapRainbow(), surface, new Color(1, 1, 1, .5f)));
    surface.setFaceDisplayed(true);
    surface.setWireframeDisplayed(true);
    surface.setWireframeColor(Color.BLACK);

    // 3. Create the chart using Emulated GL (CPU rendering)
    EmulGLChartFactory f = new EmulGLChartFactory();
    chart = f.newChart(Quality.Advanced());

    // 4. Add surface and display
    chart.add(surface);
    chart.open(); // Opens the AWT Window
    chart.addMouse(); // Enables rotation/zoom with mouse
  }
}
