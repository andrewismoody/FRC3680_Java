using System;
using System.Collections.Generic;
using System.Linq;
using OxyPlot;
using OxyPlot.Series;
using OxyPlot.Annotations;
using OxyPlot.Axes;
using AutoJsonBuilder.Models;

namespace AutoJsonBuilder.Helpers;

public static class PlotHelper
{
	// --- new: legend hit regions and highlighting state ---
	private sealed class LegendEntry
	{
		public RectangleAnnotation? BackgroundRect; // overall legend background (shared)
		public OxyRect DataRect;                    // region in data coords
		public OxyRect ScreenRect;                  // region in screen/pixel coords (for hit-testing)
		public Series Series;                       // associated series
		public TextAnnotation LabelAnnotation;      // label annotation (for debug/reference)
	}

	// per-PlotModel legend item regions (used for hit testing)
	private static readonly Dictionary<PlotModel, List<LegendEntry>> s_legendRegions = new();

	// currently highlighted (per-plot) series (click again to restore)
	private static readonly Dictionary<PlotModel, Series?> s_highlighted = new();

	// store original series colors so we can restore after dimming (Series -> (markerFill, color))
	private static readonly Dictionary<Series, (OxyColor? markerFill, OxyColor? color)> s_originalColors = new();

	// desired dim opacity for non-selected layers
	private const double DimOpacity = 0.35;

	// per-PlotModel moving cursor indicator
	private static readonly Dictionary<PlotModel, EllipseAnnotation> s_cursorIndicators = new();

	// runtime logger (set during BuildFieldPlot)
	private static Action<string>? s_log;

	// per-PlotModel mapping: Series -> its ArrowAnnotations (for hide/show)
	private static readonly Dictionary<PlotModel, Dictionary<Series, List<ArrowAnnotation>>> s_seriesArrows = new();

	// Show (or move) a circular indicator at the PlotView-relative screen point.
	// screenX/screenY should be coordinates returned by e.GetPosition(plotView) (PlotView-relative).
	// radiusPx is the radius in pixels to draw.
	public static void ShowCursorIndicator(PlotModel pm, double screenX, double screenY, double radiusPx = 8)
	{
		if (pm == null) return;
		try
		{
			s_log?.Invoke($"ShowCursorIndicator called: screen={screenX:F1},{screenY:F1} radiusPx={radiusPx:F1}");
			var xAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Bottom) ?? pm.Axes.FirstOrDefault();
			var yAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Left) ?? pm.Axes.Skip(1).FirstOrDefault() ?? xAxis;
			var plotArea = pm.PlotArea;
			if (xAxis == null || yAxis == null || plotArea.Width <= 0 || plotArea.Height <= 0) 
			{
				s_log?.Invoke("ShowCursorIndicator: missing axis or invalid plotArea");
				return;
			}

			// Convert screen (PlotView-relative) -> data coords.
			// Axis.InverseTransform expects coordinates relative to the plot area.
			// For Y we must measure from the plot-area bottom (invert Y) on this OxyPlot build.
			double localX = screenX - plotArea.Left;
			double localY = screenY - plotArea.Top;
			// invert Y measured from top -> distance-from-bottom expected by InverseTransform
			double localY_Inverted = plotArea.Height - localY;

			double dataX = xAxis.InverseTransform(localX);
			double dataY = yAxis.InverseTransform(localY_Inverted);

			 // diagnostic: report mapping math
			s_log?.Invoke($"ShowCursorIndicator: plotArea.LeftTop={plotArea.Left:F1},{plotArea.Top:F1} localTopBased={localX:F1},{localY:F1} localBottomBased={localX:F1},{localY_Inverted:F1} -> data={dataX:F3},{dataY:F3}");

			// convert pixel radius to data units (separate X/Y scale)
			double xRange = xAxis.ActualMaximum - xAxis.ActualMinimum;
			double yRange = yAxis.ActualMaximum - yAxis.ActualMinimum;
			if (xRange <= 0 || yRange <= 0) return;
			double pxPerUnitX = plotArea.Width / xRange;
			double pxPerUnitY = plotArea.Height / yRange;
			if (pxPerUnitX <= 0 || pxPerUnitY <= 0) return;

			double widthData = (radiusPx * 2.0) / pxPerUnitX;
			double heightData = (radiusPx * 2.0) / pxPerUnitY;

			// Robust correction: re-project computed data back to PlotView pixels and compare to input mouse point.
			// If there is a consistent pixel offset, convert that pixel delta back into data units (via axis transforms)
			// and apply it to the data point. This handles constant offsets (left/top) and render-space quirks.
			try
			{
				// back-projection: data -> plotView-relative pixels
				var backX = plotArea.Left + xAxis.Transform(dataX);
				var backY = plotArea.Top  + yAxis.Transform(dataY);
				var dx = screenX - backX;
				var dy = screenY - backY;
				// log the reprojection delta
				s_log?.Invoke($"ShowCursorIndicator: reproj.back={backX:F1},{backY:F1} deltaPx={dx:F1},{dy:F1}");

				// If there's a measurable delta, compute corrected data coords by feeding the
				// back-projection + delta through the inverse transforms. This keeps axis math consistent.
				if (Math.Abs(dx) > 0.5 || Math.Abs(dy) > 0.5)
				{
					// corrected X: inverse-transform of (xAxis.Transform(dataX) + dx)
					double tx = xAxis.Transform(dataX) + dx;
					double correctedDataX = xAxis.InverseTransform(tx);

					// corrected Y: compute inverted-local used by InverseTransform for Y and adjust by dy.
					// local (top-based) for current dataY is yAxis.Transform(dataY); we adjust that by dy then invert.
					double currentTopY = yAxis.Transform(dataY);
					double adjustedTopY = currentTopY + dy;
					double adjustedInverted = plotArea.Height - adjustedTopY;
					double correctedDataY = yAxis.InverseTransform(adjustedInverted);

					s_log?.Invoke($"ShowCursorIndicator: corrected data from ({dataX:F4},{dataY:F4}) -> ({correctedDataX:F4},{correctedDataY:F4})");
					dataX = correctedDataX;
					dataY = correctedDataY;
				}
			}
			catch { /* best-effort */ }

			s_log?.Invoke($"ShowCursorIndicator: pxPerUnit={pxPerUnitX:F3},{pxPerUnitY:F3} widthData={widthData:F3},heightData={heightData:F3}");
			// Get or create indicator
			if (!s_cursorIndicators.TryGetValue(pm, out var ann) || ann == null)
			{
				ann = new EllipseAnnotation
				{
					X = dataX,
					Y = dataY,
					Width = widthData,
					Height = heightData,
					Fill = OxyColor.FromArgb(80, 255, 0, 0),
					Stroke = OxyColors.Red,
					StrokeThickness = 1,
					Layer = AnnotationLayer.AboveSeries
				};
				s_cursorIndicators[pm] = ann;
				pm.Annotations.Add(ann);
			}
			else
			{
				// update existing
				ann.X = dataX;
				ann.Y = dataY;
				ann.Width = widthData;
				ann.Height = heightData;
			}

			// request redraw
			try { pm.InvalidatePlot(false); } catch { }
		}
		catch { /* best-effort */ }
	}

	// Remove any existing cursor indicator for this PlotModel.
	public static void ClearCursorIndicator(PlotModel pm)
	{
		if (pm == null) return;
		try
		{
			if (s_cursorIndicators.TryGetValue(pm, out var ann) && ann != null)
			{
				try { pm.Annotations.Remove(ann); } catch { }
				s_cursorIndicators.Remove(pm);
				try { pm.InvalidatePlot(false); } catch { }
			}
		}
		catch { /* best-effort */ }
	}

	// Build a PlotModel showing fixtures (grouped by type) and targets (layered by module).
	// - doc: document model
	// - evaluatedLookup: params/evaluated lookup used when resolving translation numeric entries (local best-effort)
	// - resolvedFixtures: fixture positions produced by the external resolver (key -> (x,y,z,rotDeg?))
	// - palette: color palette to assign to groups/modules (wraps)
	// - log: callback for diagnostic logging
	public static PlotModel BuildFieldPlot(
		AutoDefinitionModel doc,
		IDictionary<string, object?> evaluatedLookup,
		Dictionary<string, (double x, double y, double z, double? rot)> resolvedFixtures,
		OxyColor[] palette,
		Action<string> log)
	{
		try
		{
			// keep a runtime copy of the logger so mouse-time helpers can emit diagnostic lines
			s_log = log;
			var pm = new PlotModel { Title = "Field fixtures & targets" };
			pm.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "X" });
			pm.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "Y" });

			 // Apply the common legend style (floating inside the plot, semi-transparent background, compact padding).
			 // Note: the built-in legend does not support rounded corners or user-drag out of the box.
			 // For full rounded/drag behaviour you can replace this with a custom Annotation-based legend.
			ApplyLegendStyle(pm, collapsed: false);

			// local helper to coerce "raw" objects into doubles (strings / ParamValue wrappers / numeric types)
			bool TryResolveDouble(object? raw, out double v)
			{
				v = 0;
				try
				{
					if (raw == null) return false;
					switch (raw)
					{
						case double d: v = d; return true;
						case float f: v = Convert.ToDouble(f); return true;
						case int i: v = Convert.ToDouble(i); return true;
						case long l: v = Convert.ToDouble(l); return true;
						case decimal dec: v = Convert.ToDouble(dec); return true;
						case string s:
							if (double.TryParse(s, System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var parsed)) { v = parsed; return true; }
							if (evaluatedLookup != null && evaluatedLookup.TryGetValue(s, out var ev)) return TryResolveDouble(ev, out v);
							return false;
					}
					// wrapper with Raw
					var t = raw.GetType();
					var rawProp = t.GetProperty("Raw");
					if (rawProp != null)
					{
						var inner = rawProp.GetValue(raw);
						return TryResolveDouble(inner, out v);
					}
				}
				catch { /* best-effort */ }
				return false;
			}

			static double ConvertLengthToMeters(double value, string? units)
			{
				if (string.IsNullOrWhiteSpace(units)) return value * 0.0254;
				var u = units.Trim().ToLowerInvariant();
				if (u == "m" || u == "meter" || u == "meters" || u == "metre" || u == "metres") return value;
				if (u == "in" || u == "inch" || u == "inches") return value * 0.0254;
				if (u == "ft" || u == "foot" || u == "feet") return value * 0.3048;
				return value * 0.0254;
			}

			static double? NormalizeRotationToDegrees(double? value, string? units)
			{
				if (value == null) return null;
				if (string.IsNullOrWhiteSpace(units)) return value * 180.0 / Math.PI;
				var u = units.Trim().ToLowerInvariant();
				if (u.Contains("deg")) return value;
				return value * 180.0 / Math.PI;
			}

			// --- fixtures ---
			var fixtures = doc.Fixtures ?? new List<FixtureSchemaModel>();
			var groups = fixtures.Where(f => f != null).GroupBy(f => f.Type ?? "(none)").ToList();
			// precompute total layers so we can evenly space hues and avoid color duplication
			var moduleNamesList = (doc.Modules ?? Enumerable.Empty<string>()).Select(m => string.IsNullOrWhiteSpace(m) ? "(no-module)" : m).Distinct(StringComparer.OrdinalIgnoreCase).ToList();
			int totalLayers = Math.Max(1, groups.Count + moduleNamesList.Count);
			int colorIdx = 0;

			// helper: get a distinct color for layer index (fallback to palette if needed)
			static OxyColor GetDistinctColor(int idx, int total, OxyColor[] palette)
			{
				// if total is small and palette contains enough entries, prefer palette for familiarity
				if (palette != null && palette.Length >= total && idx < palette.Length) return palette[idx];
				// otherwise generate evenly spaced hues
				var hue = (idx * 360.0) / Math.Max(1, total);
				return HsvToOxyColor(hue, 0.6, 0.9);
			}
			
			// HSV -> OxyColor (0..360 hue, 0..1 sat, 0..1 val)
			static OxyColor HsvToOxyColor(double h, double s, double v)
			{
				h = (h % 360 + 360) % 360;
				double c = v * s;
				double hp = h / 60.0;
				double x = c * (1 - Math.Abs((hp % 2) - 1));
				double r1 = 0, g1 = 0, b1 = 0;
				if (0 <= hp && hp < 1) { r1 = c; g1 = x; b1 = 0; }
				else if (1 <= hp && hp < 2) { r1 = x; g1 = c; b1 = 0; }
				else if (2 <= hp && hp < 3) { r1 = 0; g1 = c; b1 = x; }
				else if (3 <= hp && hp < 4) { r1 = 0; g1 = x; b1 = c; }
				else if (4 <= hp && hp < 5) { r1 = x; g1 = 0; b1 = c; }
				else { r1 = c; g1 = 0; b1 = x; }
				double m = v - c;
				byte r = (byte)Math.Round((r1 + m) * 255.0);
				byte g = (byte)Math.Round((g1 + m) * 255.0);
				byte b = (byte)Math.Round((b1 + m) * 255.0);
				return OxyColor.FromRgb(r, g, b);
			}

			var collectedArrows = new List<ArrowAnnotation>();
			var localSeriesArrows = new Dictionary<Series, List<ArrowAnnotation>>();
			// length for arrow heads in meters (used for both fixture and target arrows)
			double arrowLengthMeters = 12.0 * 0.0254;

			foreach (var g in groups)
			{
				var type = g.Key;
				var color = GetDistinctColor(colorIdx++, totalLayers, palette);
				var scatter = new ScatterSeries
				{
					MarkerType = MarkerType.Circle,
					MarkerFill = color,
					MarkerSize = 6,
					Title = type
				};
				foreach (var f in g)
				{
					try
					{
						var key = $"{f.Type}:{f.Index}";
						if (resolvedFixtures.TryGetValue(key, out var pos))
						{
							scatter.Points.Add(new ScatterPoint(pos.x, pos.y));
							if (pos.rot.HasValue)
							{
								var theta = pos.rot.Value * Math.PI / 180.0;
								var ex = pos.x + Math.Cos(theta) * arrowLengthMeters;
								var ey = pos.y + Math.Sin(theta) * arrowLengthMeters;
								var a = new ArrowAnnotation { StartPoint = new DataPoint(pos.x, pos.y), EndPoint = new DataPoint(ex, ey), Color = color, HeadLength = 12, HeadWidth = 8 };
								// stash arrow under this series (do not add to pm.Annotations yet)
								if (!localSeriesArrows.TryGetValue(scatter, out var list)) { list = new List<ArrowAnnotation>(); localSeriesArrows[scatter] = list; }
								list.Add(a);
							}
						}
						else
						{
							log?.Invoke($"PlotHelper: skipped fixture {f.Type}:{f.Index} (no resolved position)");
						}
					}
					catch { /* best-effort per fixture */ }
				}
				if (scatter.Points.Count > 0) pm.Series.Add(scatter);
			}

			// --- targets by module ---
			var moduleSeries = new Dictionary<string, ScatterSeries>(StringComparer.OrdinalIgnoreCase);
			var moduleColors = new Dictionary<string, OxyColor>(StringComparer.OrdinalIgnoreCase);
			int modIdx = 0;
			foreach (var m in moduleNamesList)
			{
				var name = string.IsNullOrWhiteSpace(m) ? "(no-module)" : m;
				var color = GetDistinctColor(colorIdx++, totalLayers, palette);
				moduleColors[name] = color;
				moduleSeries[name] = new ScatterSeries { MarkerType = MarkerType.Diamond, MarkerFill = color, MarkerSize = 7, Title = name };
				modIdx++;
			}
			ScatterSeries GetOrCreate(string module)
			{
				if (moduleSeries.TryGetValue(module, out var s)) return s;
				var idx = moduleSeries.Count;
				var color = GetDistinctColor(colorIdx++, totalLayers, palette);
				s = new ScatterSeries { MarkerType = MarkerType.Diamond, MarkerFill = color, MarkerSize = 7, Title = module };
				moduleSeries[module] = s;
				return s;
			}

			int plottedTargets = 0;
			foreach (var t in doc.Targets ?? Enumerable.Empty<TargetSchemaModel>())
			{
				var moduleName = string.IsNullOrWhiteSpace(t.Module) ? "(no-module)" : t.Module;
				var s = GetOrCreate(moduleName);

				bool added = false;
				double tx = 0, ty = 0, tz = 0;
				double? trot = null;

				if (t.Fixture != null)
				{
					try
					{
						var key = $"{t.Fixture.Type}:{t.Fixture.Index}";
						if (resolvedFixtures.TryGetValue(key, out var pos))
						{
							tx = pos.x; ty = pos.y; tz = pos.z; trot = pos.rot;
							added = true;
						}
					}
					catch { /* best-effort */ }
				}

				if (!added && t.Translation != null)
				{
					try
					{
						var tr = t.Translation;
						if (tr.Position != null && tr.Position.Count >= 2)
						{
							if (TryResolveDouble(tr.Position[0], out var rx) && TryResolveDouble(tr.Position[1], out var ry))
							{
								var units = tr.PositionUnits;
								tx = ConvertLengthToMeters(rx, units);
								ty = ConvertLengthToMeters(ry, units);

								if (tr.Position.Count >= 3 && TryResolveDouble(tr.Position[2], out var rz))
									tz = ConvertLengthToMeters(rz, units);
								else
									tz = 0.0;

								if (tr.Rotation != null && TryResolveDouble(tr.Rotation, out var rrot))
									trot = NormalizeRotationToDegrees(rrot, tr.RotationUnits);

								added = true;
							}
						}
					}
					catch { /* best-effort */ }
				}

				if (added)
				{
					s.Points.Add(new ScatterPoint(tx, ty));
					plottedTargets++;
					if (trot.HasValue)
					{
						var theta = trot.Value * Math.PI / 180.0;
						var ex = tx + Math.Cos(theta) * arrowLengthMeters;
						var ey = ty + Math.Sin(theta) * arrowLengthMeters;
						var color = moduleColors.TryGetValue(moduleName, out var c) ? c : OxyPlot.OxyColors.Black;
						var a = new ArrowAnnotation { StartPoint = new DataPoint(tx, ty), EndPoint = new DataPoint(ex, ey), Color = color, HeadLength = 12, HeadWidth = 8 };
						if (!localSeriesArrows.TryGetValue(s, out var list)) { list = new List<ArrowAnnotation>(); localSeriesArrows[s] = list; }
						list.Add(a);
					}
				}
			}

			// add module series (only those with points)
			foreach (var ms in moduleSeries.Values)
				if (ms.Points.Count > 0) pm.Series.Add(ms);

			// Add all collected arrows to the PlotModel annotations and persist mapping for runtime hide/show
			if (localSeriesArrows.Count > 0)
			{
				foreach (var kv in localSeriesArrows)
				{
					foreach (var a in kv.Value)
					{
						pm.Annotations.Add(a);
					}
				}
				s_seriesArrows[pm] = localSeriesArrows;
			}

			// add arrows
			// (arrows already added above from localSeriesArrows)

			 // Build a guaranteed-visible annotation legend (fallback in case built-in legend doesn't show).
			TryAddAnnotationLegend(pm);

			// Updated handler to rescale arrow heads
			pm.Updated += (s, e) =>
			{
				try
				{
					var xAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Bottom) ?? pm.Axes.FirstOrDefault();
					var yAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Left) ?? pm.Axes.Skip(1).FirstOrDefault() ?? xAxis;
					var plotArea = pm.PlotArea;
					if (xAxis == null || yAxis == null || plotArea.Width <= 0 || plotArea.Height <= 0) return;
					double xRange = xAxis.ActualMaximum - xAxis.ActualMinimum;
					double yRange = yAxis.ActualMaximum - yAxis.ActualMinimum;
					if (xRange <= 0 || yRange <= 0) return;

					double pxPerUnitX = plotArea.Width / xRange;
					double pxPerUnitY = plotArea.Height / yRange;

					foreach (var ann in pm.Annotations.OfType<ArrowAnnotation>())
					{
						try
						{
							var dx = ann.EndPoint.X - ann.StartPoint.X;
							var dy = ann.EndPoint.Y - ann.StartPoint.Y;
							var pixelLen = Math.Sqrt((dx * pxPerUnitX) * (dx * pxPerUnitX) + (dy * pxPerUnitY) * (dy * pxPerUnitY));
							var newHeadLength = Math.Max(6, pixelLen * 0.35);
							var newHeadWidth = Math.Max(4, pixelLen * 0.25);
							if (Math.Abs(ann.HeadLength - newHeadLength) > 0.5 || Math.Abs(ann.HeadWidth - newHeadWidth) > 0.5)
							{
								ann.HeadLength = newHeadLength;
								ann.HeadWidth = newHeadWidth;
							}
						}
						catch { /* per-annotation */ }
					}
				}
				catch { /* best-effort */ }
			};

			pm.IsLegendVisible = true;
			log?.Invoke($"PlotHelper: built plot (fixtures { (doc.Fixtures?.Count ?? 0) }, targets plotted {plottedTargets})");
			return pm;
		}
		catch (Exception ex)
		{
			log?.Invoke($"PlotHelper.BuildFieldPlot failed: {ex.Message}");
			return new PlotModel { Title = "Plot failed" };
		}
	}

	// Create a simple annotation-based legend and add it to the PlotModel.
	// This ensures a visible legend even when built-in legend support is limited on the OxyPlot build in use.
	private static void TryAddAnnotationLegend(PlotModel pm)
	{
		if (pm == null) return;

		// collect titled series and their display colors (prefers ScatterSeries.MarkerFill; falls back to black)
		var items = pm.Series
					  .Where(s => !string.IsNullOrWhiteSpace(s.Title))
					  .Select(s =>
					  {
						  var color = OxyColors.Black;
						  if (s is ScatterSeries ss && ss.MarkerFill != null) color = ss.MarkerFill;
						  else
						  {
							  // try to inspect common series color properties
							  var prop = s.GetType().GetProperty("Color");
							  if (prop != null && prop.GetValue(s) is OxyColor oc) color = oc;
						  }
						  return (Title: s.Title ?? "", Color: color);
					  })
					  .ToList();

		if (items.Count == 0) return;

		// Remove any previous custom-legend annotations we added (identify by TextAnnotation text marker)
		// ElementCollection<T> doesn't expose RemoveAll; collect then remove to avoid modifying during enumeration.
		var toRemove = new List<Annotation>();
		toRemove.AddRange(pm.Annotations.OfType<TextAnnotation>().Where(ta => ta.Text != null && ta.Text.StartsWith("__legend_marker__")));
		// background rectangle we used: semi-transparent white + dim gray stroke (most likely unique)
		toRemove.AddRange(pm.Annotations.OfType<RectangleAnnotation>().Where(r => r.Fill != null && r.Fill.A == 200 && r.Stroke == OxyColors.DimGray));
		// swatches: RectangleAnnotations previously added use Transparent stroke and AboveSeries layer — remove those too
		toRemove.AddRange(pm.Annotations.OfType<RectangleAnnotation>().Where(r => r.Stroke == OxyColors.Transparent && r.Layer == AnnotationLayer.AboveSeries));
		// now remove collected annotations
		foreach (var a in toRemove.Distinct().ToList())
		{
			try { pm.Annotations.Remove(a); } catch { /* best-effort */ }
		}

		// Create background rectangle annotation (we'll set coords in Updated event when axes are known)
		var bg = new RectangleAnnotation
		{
			Fill = OxyColor.FromArgb(200, 255, 255, 255),
			Stroke = OxyColors.DimGray,
			StrokeThickness = 1.5, // stronger border for the legend box
			Layer = AnnotationLayer.AboveSeries
			// Note: CornerRadius not always available; skip for cross-version compatibility.
		};
		pm.Annotations.Add(bg);

		// create per-item swatch + label text annotations and keep references
		var swatches = new List<RectangleAnnotation>();
		var labels = new List<TextAnnotation>();
		foreach (var it in items)
		{
			// swatch: flat color chip with no border
			var sw = new RectangleAnnotation { Fill = it.Color, Stroke = OxyColors.Transparent, StrokeThickness = 0, Layer = AnnotationLayer.AboveSeries };
			// label: plain text with transparent background and no stroke so there's no per-label border
			var ta = new TextAnnotation
			{
				Text = "__legend_marker__" + it.Title,
				FontWeight = FontWeights.Bold,
				Layer = AnnotationLayer.AboveSeries,
				Background = OxyColors.Undefined,   // ensure no boxed background
				TextColor = OxyColors.Black,        // explicit color for contrast against bg
				Stroke = OxyColors.Undefined,       // remove any outline stroke
				StrokeThickness = 0                 // ensure zero thickness (no border)
				// Padding not set for max compatibility across OxyPlot versions
			};
			// we will set positions later
			pm.Annotations.Add(sw);
			pm.Annotations.Add(ta);
			swatches.Add(sw);
			labels.Add(ta);
		}

		 // Build legend entry bookkeeping for hit-testing and store it per plot model.
		// We'll populate data-space rectangles inside the Updated handler below.
		s_legendRegions[pm] = new List<LegendEntry>();
		for (int i = 0; i < items.Count; i++)
		{
			// associate each label with its series by title (best-effort)
			var title = items[i].Title;
			var series = pm.Series.FirstOrDefault(s => string.Equals(s.Title, title, StringComparison.Ordinal)) ?? pm.Series.ElementAtOrDefault(i);
			if (series == null) continue;
			// OxyRect.Empty is not available on all OxyPlot builds — use a default empty rect
			s_legendRegions[pm].Add(new LegendEntry { BackgroundRect = bg, DataRect = new OxyRect(0, 0, 0, 0), ScreenRect = new OxyRect(0, 0, 0, 0), Series = series, LabelAnnotation = labels[i] });
		}

		// Position/update legend on layout changes
		pm.Updated += (s, e) =>
		{
			try
			{
				// Find primary axes
				var xAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Bottom) ?? pm.Axes.FirstOrDefault();
				var yAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Left) ?? pm.Axes.Skip(1).FirstOrDefault() ?? xAxis;
				if (xAxis == null || yAxis == null) return;

				// get plot area and ensure it's valid before doing screen transforms
				var plotArea = pm.PlotArea;
				if (plotArea.Width <= 0 || plotArea.Height <= 0) return;

				var xMax = xAxis.ActualMaximum;
				var xMin = xAxis.ActualMinimum;
				var yMax = yAxis.ActualMaximum;
				var yMin = yAxis.ActualMinimum;
				var xRange = xMax - xMin;
				var yRange = yMax - yMin;
				if (xRange <= 0 || yRange <= 0) return;

				 // layout constants (fractions of axis ranges)
				const double widthFrac = 0.22;        // width fraction of x range
				const double rightMarginFrac = 0.02;  // right margin fraction
				const double topMarginFrac = 0.02;    // top margin fraction
				const double perItemFrac = 0.06;      // per-item vertical fraction of y range (increased to avoid overlap)
				const double swatchWidthFrac = 0.03;
				const double swatchPadFrac = 0.01;

				var legendWidth = xRange * widthFrac;
				var legendRight = xMax - xRange * rightMarginFrac;
				var legendLeft = legendRight - legendWidth;
				var legendTop = yMax - yRange * topMarginFrac;
				// compute total legend height based on per-item fraction (guarantees non-overlapping item boxes)
				var perItemDataHeight = yRange * perItemFrac;
				var totalLegendHeight = (perItemDataHeight * items.Count) + (yRange * 0.01); // small extra padding
				var legendBottom = legendTop - totalLegendHeight;

				// set background rectangle in data coords
				bg.MinimumX = legendLeft;
				bg.MaximumX = legendRight;
				bg.MinimumY = legendBottom;
				bg.MaximumY = legendTop;

				 // ensure we have entries
				if (!s_legendRegions.TryGetValue(pm, out var regions) || regions == null) return;

				// place swatches and labels and update each LegendEntry.DataRect
				for (int i = 0; i < items.Count && i < swatches.Count && i < labels.Count && i < regions.Count; i++)
				{
					var sw = swatches[i];
					var ta = labels[i];
					var entry = regions[i];

					// compute per-item top/bottom using explicit per-item height
					var itemTop = legendTop - i * perItemDataHeight - yRange * 0.005;
					var itemBottom = itemTop - perItemDataHeight + yRange * 0.005;

					// swatch horizontally near left of legend, label to its right
					var swLeft = legendLeft + xRange * (swatchPadFrac);
					var swRight = swLeft + xRange * swatchWidthFrac;
					var labelX = swRight + xRange * (swatchPadFrac * 0.6);
					var labelY = (itemTop + itemBottom) * 0.5;

					// set swatch bounds (centered vertically within per-item region)
					sw.MinimumX = swLeft;
					sw.MaximumX = swRight;
					sw.MinimumY = itemBottom;
					sw.MaximumY = itemTop;

					// set label in the middle of the item region
					ta.Text = items[i].Title;
					ta.TextPosition = new DataPoint(labelX, labelY);
					ta.TextHorizontalAlignment = OxyPlot.HorizontalAlignment.Left;
					ta.TextVerticalAlignment = OxyPlot.VerticalAlignment.Middle;
					ta.FontSize = 12;

					// set DataRect as the full per-item region (click anywhere in the item)
					entry.DataRect = new OxyRect(legendLeft, itemBottom, legendRight - legendLeft, itemTop - itemBottom);

					// compute and store screen-space rectangle for robust hit-testing:
					// convert the data-space legend bounds for this item to screen coords via axis.Transform
					try
					{
						var leftScreen = xAxis.Transform(legendLeft);
						var rightScreen = xAxis.Transform(legendRight);
						var topScreen = yAxis.Transform(itemTop);
						var bottomScreen = yAxis.Transform(itemBottom);
						// normalize to rectangle with positive width/height
						var sx = Math.Min(leftScreen, rightScreen);
						var sy = Math.Min(topScreen, bottomScreen);
						var swPix = Math.Abs(rightScreen - leftScreen);
						var shPix = Math.Abs(bottomScreen - topScreen);
						// Transform returns coordinates relative to the plot area; convert to PlotView-relative screen coords
						entry.ScreenRect = new OxyRect(sx + plotArea.Left, sy + plotArea.Top, swPix, shPix);

						// diagnostic: log each legend item's computed screen rect
						s_log?.Invoke($"LegendEntry[{i}] Title='{items[i].Title}' ScreenRect={entry.ScreenRect.Left:F1},{entry.ScreenRect.Top:F1} {entry.ScreenRect.Width:F1}x{entry.ScreenRect.Height:F1}");
					}
					catch
					{
						entry.ScreenRect = new OxyRect(0, 0, 0, 0);
					}
				}
			}
			catch { /* best-effort */ }
		};
	}

	// Try to handle a mouse click (screen coords relative to the PlotView) on a legend item.
	// Returns true if the helper handled the click (updated plot model); caller should call InvalidatePlot(false).
	public static bool TryHandleLegendClick(PlotModel pm, double screenX, double screenY)
	{
		// Legend click handling intentionally disabled.
		try { s_log?.Invoke($"TryHandleLegendClick: disabled (click@{screenX:F1},{screenY:F1})"); } catch { }
		return false;
	}

	// Restore all series opacity to fully opaque and clear highlight state.
	public static void RestoreLegendHighlight(PlotModel pm)
	{
		if (pm == null) return;
		RestoreAllSeriesColors(pm);
		s_highlighted[pm] = null;
	}

	// Set series "opacity" by altering its color(s) alpha channel (stores original colors first).
	private static void SetSeriesOpacity(Series series, double opacity)
	{
		if (series == null) return;
		opacity = Math.Clamp(opacity, 0.0, 1.0);

		// helper to apply alpha to an OxyColor
		static OxyColor ApplyAlpha(OxyColor c, double alpha)
		{
			byte a = (byte)Math.Round(alpha * 255.0);
			return OxyColor.FromArgb(a, c.R, c.G, c.B);
		}

		// capture original colors once
		if (!s_originalColors.ContainsKey(series))
		{
			OxyColor? markerFill = null;
			OxyColor? color = null;
			if (series is ScatterSeries ss) markerFill = ss.MarkerFill;
			// try reflection-based retrieval for common color properties
			var propMarker = series.GetType().GetProperty("MarkerFill");
			if (propMarker != null)
			{
				try
				{
					var v = propMarker.GetValue(series);
					if (v is OxyColor mv) markerFill = mv;
				}
				catch { /* ignore */ }
			}
			var propColor = series.GetType().GetProperty("Color");
			if (propColor != null)
			{
				try
				{
					var v = propColor.GetValue(series);
					if (v is OxyColor cv) color = cv;
				}
				catch { /* ignore */ }
			}
			s_originalColors[series] = (markerFill, color);
		}

		var orig = s_originalColors[series];

		// Apply for ScatterSeries.MarkerFill when present
		if (series is ScatterSeries sss)
		{
			OxyColor baseC;
			if (orig.markerFill.HasValue) baseC = orig.markerFill.Value;
			else baseC = sss.MarkerFill;
			sss.MarkerFill = ApplyAlpha(baseC, opacity);
		}

		// Try to set MarkerFill property by reflection if available (covers other series types)
		var markerProp = series.GetType().GetProperty("MarkerFill");
		if (markerProp != null)
		{
			try
			{
				// compute base color: original if present, otherwise read current prop, otherwise fallback to black
				OxyColor baseC = OxyColors.Black;
				if (orig.markerFill.HasValue) baseC = orig.markerFill.Value;
				else
				{
					var v = markerProp.GetValue(series);
					if (v is OxyColor mv) baseC = mv;
				}
				markerProp.SetValue(series, ApplyAlpha(baseC, opacity));
			}
			catch { /* ignore */ }
		}

		// Try to set Color property by reflection
		var colorProp = series.GetType().GetProperty("Color");
		if (colorProp != null)
		{
			try
			{
				OxyColor baseC = OxyColors.Black;
				if (orig.color.HasValue) baseC = orig.color.Value;
				else
				{
					var v = colorProp.GetValue(series);
					if (v is OxyColor cv) baseC = cv;
				}
				colorProp.SetValue(series, ApplyAlpha(baseC, opacity));
			}
			catch { /* ignore */ }
		}
	}

	// Restore original captured series colors (or make fully opaque if original absent).
	private static void RestoreAllSeriesColors(PlotModel pm)
	{
		if (pm == null) return;
		foreach (var s in pm.Series)
		{
			try
			{
				if (s_originalColors.TryGetValue(s, out var orig))
				{
					// restore marker fill if present
					if (s is ScatterSeries ss && orig.markerFill.HasValue) ss.MarkerFill = orig.markerFill.Value;
					var markerProp = s.GetType().GetProperty("MarkerFill");
					if (markerProp != null && orig.markerFill.HasValue)
					{
						try { markerProp.SetValue(s, orig.markerFill.Value); } catch { /* ignore */ }
					}
					// restore color if present
					var colorProp = s.GetType().GetProperty("Color");
					if (colorProp != null && orig.color.HasValue)
					{
						try { colorProp.SetValue(s, orig.color.Value); } catch { /* ignore */ }
					}
				}
				else
				{
					// No stored original: attempt to make series opaque by setting alpha to 255 on any color props
					var markerProp = s.GetType().GetProperty("MarkerFill");
					if (markerProp != null)
					{
						try
						{
							var v = markerProp.GetValue(s);
							if (v is OxyColor mf) markerProp.SetValue(s, OxyColor.FromArgb(255, mf.R, mf.G, mf.B));
						}
						catch { /* ignore */ }
					}
					var colorProp = s.GetType().GetProperty("Color");
					if (colorProp != null)
					{
						try
						{
							var v = colorProp.GetValue(s);
							if (v is OxyColor c) colorProp.SetValue(s, OxyColor.FromArgb(255, c.R, c.G, c.B));
						}
						catch { /* ignore */ }
					}
				}
			}
			catch { /* best-effort */ }
		}
	}

	// Centralize legend appearance and a simple "collapsed" mode.
	// NOTE: Many OxyPlot builds expose LegendPlacement/LegendPosition/etc on PlotModel,
	// but some package/target frameworks used here do not. To remain compatible we only
	// toggle the built-in legend visibility. For rounded corners, semi-transparency and
	// user-draggable behavior implement a custom Annotation-based legend (TODO).
	public static void ApplyLegendStyle(PlotModel pm, bool collapsed)
	{
		if (pm == null) return;
		// Keep this portable: show the built-in legend and avoid setting properties that may not exist.
		pm.IsLegendVisible = true;

		// TODO: If you need rounded corners / custom background / draggable legend implement:
		// - A RectangleAnnotation for the background (with semi-transparent fill)
		// - TextAnnotations for title + items
		// - Mouse handlers via PlotController to support dragging / collapse toggle
		// I can provide a sample Annotation-based legend implementation if you'd like.
	}

	// Toggle between showing and hiding the legend (portable).
	public static void ToggleLegendCollapsed(PlotModel pm)
	{
		if (pm == null) return;
		pm.IsLegendVisible = !pm.IsLegendVisible;
		pm.InvalidatePlot(false);
	}

	// Show/hide arrow annotations associated with a series on the given PlotModel.
	public static void SetSeriesAnnotationsVisibility(PlotModel pm, Series series, bool visible)
	{
		if (pm == null || series == null) return;
		try
		{
			if (!s_seriesArrows.TryGetValue(pm, out var map) || map == null) return;
			if (!map.TryGetValue(series, out var arrows) || arrows == null) return;
			if (visible)
			{
				foreach (var a in arrows)
				{
					if (!pm.Annotations.Contains(a)) pm.Annotations.Add(a);
				}
			}
			else
			{
				foreach (var a in arrows)
				{
					if (pm.Annotations.Contains(a)) pm.Annotations.Remove(a);
				}
			}
			try { pm.InvalidatePlot(false); } catch { }
		}
		catch { /* best-effort */ }
	}
}
