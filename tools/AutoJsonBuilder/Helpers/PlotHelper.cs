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
			var pm = new PlotModel { Title = "Field fixtures & targets" };
			pm.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "X" });
			pm.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "Y" });

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
			int colorIdx = 0;
			var collectedArrows = new List<ArrowAnnotation>();
			foreach (var g in groups)
			{
				var type = g.Key;
				var color = palette[colorIdx % palette.Length];
				colorIdx++;
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
								var arrowLengthMeters = 12.0 * 0.0254;
								var ex = pos.x + Math.Cos(theta) * arrowLengthMeters;
								var ey = pos.y + Math.Sin(theta) * arrowLengthMeters;
								collectedArrows.Add(new ArrowAnnotation { StartPoint = new DataPoint(pos.x, pos.y), EndPoint = new DataPoint(ex, ey), Color = color, HeadLength = 12, HeadWidth = 8 });
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
			foreach (var m in doc.Modules ?? Enumerable.Empty<string>())
			{
				var name = string.IsNullOrWhiteSpace(m) ? "(no-module)" : m;
				var color = palette[modIdx % palette.Length];
				moduleColors[name] = color;
				moduleSeries[name] = new ScatterSeries { MarkerType = MarkerType.Diamond, MarkerFill = color, MarkerSize = 7, Title = name };
				modIdx++;
			}
			ScatterSeries GetOrCreate(string module)
			{
				if (moduleSeries.TryGetValue(module, out var s)) return s;
				var idx = moduleSeries.Count;
				var color = palette[idx % palette.Length];
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
						var arrowLengthMeters = 12.0 * 0.0254;
						var ex = tx + Math.Cos(theta) * arrowLengthMeters;
						var ey = ty + Math.Sin(theta) * arrowLengthMeters;
						var color = moduleColors.TryGetValue(moduleName, out var c) ? c : OxyPlot.OxyColors.Black;
						collectedArrows.Add(new ArrowAnnotation { StartPoint = new DataPoint(tx, ty), EndPoint = new DataPoint(ex, ey), Color = color, HeadLength = 12, HeadWidth = 8 });
					}
				}
			}

			// add module series (only those with points)
			foreach (var ms in moduleSeries.Values)
				if (ms.Points.Count > 0) pm.Series.Add(ms);

			// add arrows
			foreach (var a in collectedArrows) pm.Annotations.Add(a);

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
}
