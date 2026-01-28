using System;
using System.Collections.Generic;
using System.Linq;
using OxyPlot;
using OxyPlot.Series;
using OxyPlot.Annotations;
using OxyPlot.Axes;
using AutoJsonBuilder.Models;
using System.Threading.Tasks;
using System.Windows; // used for Application.Current.Dispatcher
using System.IO;
using System.Text.Json;

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

	// NEW: per-PlotModel mapping: Series -> other Annotations (ImageAnnotation, etc.) to support toggling the background 'map'
	private static readonly Dictionary<PlotModel, Dictionary<Series, List<Annotation>>> s_seriesExtraAnnotations = new();

	// pending field image info (queued until Updated handler can place them with stable axes/plotArea)
	private sealed class FieldImageInfo
	{
		public byte[] Bytes = Array.Empty<byte>();
		public string? PngPath;
		public int? OriginX;
		public int? OriginY;
		public int? ExtentW;
		public int? ExtentH;
		public double? FieldWidthMeters;
		public double? FieldHeightMeters;
		public bool Added = false;
	}
	private static readonly Dictionary<PlotModel, FieldImageInfo> s_pendingFieldImages = new();

	// per-PlotModel mapping used to apply the desired initial view once (avoids OxyPlot autoscale clobbering)
	private static readonly Dictionary<PlotModel, (double xMin, double xMax, double yMin, double yMax)> s_initialViews
		= new();

	// mark which PlotModels already had their initial view applied
	private static readonly HashSet<PlotModel> s_initialViewApplied = new();

	// Preserve last-applied axis ranges so we can detect which axis the user changed.
	private static readonly Dictionary<PlotModel, (double xMin, double xMax, double yMin, double yMax)> s_prevRanges
		= new();

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

	// Convert length to meters based on units.
	private static double ConvertLengthToMeters(double value, string? units)
	{
		if (string.IsNullOrWhiteSpace(units)) return value * 0.0254;
		var u = units.Trim().ToLowerInvariant();
		if (u == "m" || u == "meter" || u == "meters" || u == "metre" || u == "metres") return value;
		if (u == "in" || u == "inch" || u == "inches") return value * 0.0254;
		if (u == "ft" || u == "foot" || u == "feet") return value * 0.3048;
		return value * 0.0254;
	}

	// Normalize rotation to degrees based on units.
	private static double? NormalizeRotationToDegrees(double? value, string? units)
	{
		if (value == null) return null;
		if (string.IsNullOrWhiteSpace(units)) return value * 180.0 / Math.PI;
		var u = units.Trim().ToLowerInvariant();
		if (u.Contains("deg")) return value;
		return value * 180.0 / Math.PI;
	}

	// --- new: color helpers used to pick distinct layer colors ---
	private static OxyColor GetDistinctColor(int idx, int total, OxyColor[] palette)
	{
		if (palette != null && palette.Length >= total && idx >= 0 && idx < palette.Length) return palette[idx];
		var hue = (idx * 360.0) / Math.Max(1, total);
		return HsvToOxyColor(hue, 0.6, 0.9);
	}

	private static OxyColor HsvToOxyColor(double h, double s, double v)
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
			 // Default view: origin at lower-left and X axis spans 0..9 meters (make X authoritative)
			// Set AbsoluteMinimum/AbsoluteMaximum so OxyPlot autoscale won't override the X span.
			var xAxis = new LinearAxis
			{
				Position = AxisPosition.Bottom,
				Title = "X",
				Minimum = 0,
				Maximum = 9 // initial default view; do NOT set AbsoluteMinimum/AbsoluteMaximum so user can zoom out past 9
			};
			var yAxis = new LinearAxis
			{
				Position = AxisPosition.Left,
				Title = "Y",
				Minimum = 0,
				Maximum = 9 // initial guess; the Updated handler will compute the isotropic Y range and apply it
			}; // initial square view; Updated handler will adjust to preserve aspect
			pm.Axes.Add(xAxis);
			pm.Axes.Add(yAxis);

			 // Apply the common legend style (floating inside the plot, semi-transparent background, compact padding).
			 // Note: the built-in legend does not support rounded corners or user-drag out of the box.
			 // For full rounded/drag behaviour you can replace this with a custom Annotation-based legend.
			ApplyLegendStyle(pm, collapsed: false);

			// --- fixtures ---
			var fixtures = doc.Fixtures ?? new List<FixtureSchemaModel>();
			var groups = fixtures.Where(f => f != null).GroupBy(f => f.Type ?? "(none)").ToList();
			// precompute total layers so we can evenly space hues and avoid color duplication
			var moduleNamesList = (doc.Modules ?? Enumerable.Empty<string>()).Select(m => string.IsNullOrWhiteSpace(m) ? "(no-module)" : m).Distinct(StringComparer.OrdinalIgnoreCase).ToList();
			int totalLayers = Math.Max(1, groups.Count + moduleNamesList.Count);
			int colorIdx = 0;

			// --- moved-in: try to add background field image from FieldDefs/{season} ---
			// We'll track image data-space extents so the initial view can include the image.
			double? imageMinX = null, imageMaxX = null, imageMinY = null, imageMaxY = null;

			try
			{
				var season = (doc?.Season?.Trim()) ?? "2025";
				var defsDir = Path.Combine(AppContext.BaseDirectory ?? ".", "FieldDefs", season);
				s_log?.Invoke($"PlotHelper: image load: season='{season}', defsDir='{defsDir}'");
				if (!Directory.Exists(defsDir))
				{
					s_log?.Invoke($"PlotHelper: image load: FieldDefs directory not found: '{defsDir}'");
				}
				else
				{
					var pngPath = Directory.EnumerateFiles(defsDir, "*.png", SearchOption.TopDirectoryOnly).FirstOrDefault();
					if (string.IsNullOrWhiteSpace(pngPath))
					{
						s_log?.Invoke($"PlotHelper: image load: no PNG found in '{defsDir}'");
					}
					else
					{
						s_log?.Invoke($"PlotHelper: image load: found PNG '{pngPath}'");
						// prefer defn/definition json names but accept any .json
						var jsonPath = Directory.EnumerateFiles(defsDir, "*.json", SearchOption.TopDirectoryOnly)
											  .FirstOrDefault(f => Path.GetFileName(f).IndexOf("defn", StringComparison.OrdinalIgnoreCase) >= 0)
									   ?? Directory.EnumerateFiles(defsDir, "*.json", SearchOption.TopDirectoryOnly).FirstOrDefault();
						if (string.IsNullOrWhiteSpace(jsonPath))
						{
							s_log?.Invoke($"PlotHelper: image load: no JSON found in '{defsDir}' (continuing without JSON)");
						}
						else
						{
							s_log?.Invoke($"PlotHelper: image load: found JSON '{jsonPath}'");
						}

						int? originX = null, originY = null;
						int? extentW = null, extentH = null;

						if (!string.IsNullOrWhiteSpace(jsonPath))
						{
							try
							{
								var js = File.ReadAllText(jsonPath);
								using var jd = JsonDocument.Parse(js);
								if (jd.RootElement.TryGetProperty("origin", out var o) && o.ValueKind == JsonValueKind.Array && o.GetArrayLength() >= 2)
								{
									if (o[0].ValueKind == JsonValueKind.Number) originX = o[0].GetInt32();
									if (o[1].ValueKind == JsonValueKind.Number) originY = o[1].GetInt32();
								}
								if (jd.RootElement.TryGetProperty("extent", out var ex) && ex.ValueKind == JsonValueKind.Array && ex.GetArrayLength() >= 2)
								{
									if (ex[0].ValueKind == JsonValueKind.Number) extentW = ex[0].GetInt32();
									if (ex[1].ValueKind == JsonValueKind.Number) extentH = ex[1].GetInt32();
								}
								s_log?.Invoke($"PlotHelper: parsed JSON origin={originX?.ToString() ?? "null"},{originY?.ToString() ?? "null"} extent={extentW?.ToString() ?? "null"}x{extentH?.ToString() ?? "null"}");
							}
							catch (Exception je)
							{
								s_log?.Invoke($"PlotHelper: failed to parse field JSON '{jsonPath}': {je.Message}");
							}
						}

						// Attempt to obtain field-size (meters) from evaluatedLookup or from doc via reflection.
						double? fieldWidthMeters = null;
						double? fieldHeightMeters = null;
						try
						{
							// prefer evaluated lookup (e.g. params or top-level resolved values)
							if (evaluatedLookup != null)
							{
								object? fv = null;
								foreach (var key in new[] { "fieldsize", "fieldSize", "field_size", "fieldSizeMeters" })
								{
									if (evaluatedLookup.TryGetValue(key, out var v)) { fv = v; break; }
								}
								if (fv != null)
								{
									if (fv is double dd) { fieldWidthMeters = dd; fieldHeightMeters = dd; }
									else if (fv is float ff) { fieldWidthMeters = Convert.ToDouble(ff); fieldHeightMeters = fieldWidthMeters; }
									else if (fv is int ii) { fieldWidthMeters = Convert.ToDouble(ii); fieldHeightMeters = fieldWidthMeters; }
									else if (fv is IList<object> list && list.Count >= 2)
									{
										if (double.TryParse(list[0]?.ToString() ?? "", out var a)) fieldWidthMeters = a;
										if (double.TryParse(list[1]?.ToString() ?? "", out var b)) fieldHeightMeters = b;
									}
									else if (fv is System.Text.Json.JsonElement je)
									{
										if (je.ValueKind == JsonValueKind.Number && je.TryGetDouble(out var d)) { fieldWidthMeters = d; fieldHeightMeters = d; }
										else if (je.ValueKind == JsonValueKind.Array && je.GetArrayLength() >= 2)
										{
											if (je[0].ValueKind == JsonValueKind.Number) fieldWidthMeters = je[0].GetDouble();
											if (je[1].ValueKind == JsonValueKind.Number) fieldHeightMeters = je[1].GetDouble();
										}
									}
								}
								s_log?.Invoke($"PlotHelper: evaluatedLookup gave fieldWidthMeters={fieldWidthMeters?.ToString() ?? "null"} fieldHeightMeters={fieldHeightMeters?.ToString() ?? "null"}");
							}
						}
						catch { /* best-effort */ }

						// fallback: try doc reflection for common property names if evaluatedLookup didn't yield a size
						if (!fieldWidthMeters.HasValue)
						{
							try
							{
								var names = new[] { "FieldSize", "FieldSizeMeters", "FieldDimensions", "Field" };
								foreach (var n in names)
								{
									var prop = doc?.GetType().GetProperty(n);
									if (prop == null) continue;
									var val = prop.GetValue(doc);
									if (val == null) continue;
									if (val is double dv) { fieldWidthMeters = dv; fieldHeightMeters = dv; break; }
									if (val is IList<object> lst && lst.Count >= 2)
									{
										if (double.TryParse(lst[0]?.ToString() ?? "", out var a)) fieldWidthMeters = a;
										if (double.TryParse(lst[1]?.ToString() ?? "", out var b)) fieldHeightMeters = b;
										break;
									}
									if (double.TryParse(val.ToString(), out var pd)) { fieldWidthMeters = pd; fieldHeightMeters = pd; break; }
								}
								s_log?.Invoke($"PlotHelper: reflection gave fieldWidthMeters={fieldWidthMeters?.ToString() ?? "null"} fieldHeightMeters={fieldHeightMeters?.ToString() ?? "null"}");
							}
							catch { /* best-effort */ }
						}

						 // --- new: normalize suspiciously large field sizes (common case: user supplied inches without units) ---
						if (fieldWidthMeters.HasValue)
						{
							try
							{
								var rawFw = fieldWidthMeters.Value;
								// Heuristic: values larger than 50 are unlikely to be meters for typical fields.
								// Treat them as inches and convert to meters.
								if (rawFw > 50.0)
								{
									s_log?.Invoke($"PlotHelper: fieldWidthMeters={rawFw} seems large; assuming units in inches and converting to meters.");
									fieldWidthMeters = ConvertLengthToMeters(rawFw, "in");
									if (fieldHeightMeters.HasValue)
										fieldHeightMeters = ConvertLengthToMeters(fieldHeightMeters.Value, "in");
									else if (extentH.HasValue && extentW.HasValue && extentW.Value > 0)
										fieldHeightMeters = fieldWidthMeters * ((double)extentH.Value / extentW.Value);

									s_log?.Invoke($"PlotHelper: normalized field size -> {fieldWidthMeters:F3}m x {fieldHeightMeters:F3}m (assumed inches)");
								}
							}
							catch { /* best-effort */ }
						}
						// --- end normalization ---

						// read PNG bytes and queue placement for Updated handler (avoid placing before axes/layout stable)
						try
						{
							s_log?.Invoke($"PlotHelper: queueing PNG bytes from '{pngPath}' for deferred placement");
							var bytes = File.ReadAllBytes(pngPath);
							var finfo = new FieldImageInfo
							{
								Bytes = bytes,
								PngPath = pngPath,
								OriginX = originX,
								OriginY = originY,
								ExtentW = extentW,
								ExtentH = extentH,
								FieldWidthMeters = fieldWidthMeters,
								FieldHeightMeters = fieldHeightMeters,
								Added = false
							};
							// store per-PlotModel; will be applied in Updated once plotArea/axes are stable
							s_pendingFieldImages[pm] = finfo;
							s_log?.Invoke($"PlotHelper: queued field image for PlotModel '{pm.Title}' (will be placed in Updated handler).");
						}
						catch (Exception ie)
						{
							s_log?.Invoke($"PlotHelper: failed to read PNG '{pngPath}': {ie.Message}");
						}
					}
				}
			}
			catch (Exception ex)
			{
				s_log?.Invoke($"PlotHelper: field image load error: {ex.Message}");
			}
			// --- end moved-in image logic ---

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
								var a = new ArrowAnnotation { StartPoint = new DataPoint(pos.x, pos.y), EndPoint = new DataPoint(ex, ey), Color = color, HeadLength = 0.4, HeadWidth = 0.3 };
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
						var a = new ArrowAnnotation { StartPoint = new DataPoint(tx, ty), EndPoint = new DataPoint(ex, ey), Color = color, HeadLength = 4, HeadWidth = 3 };
						if (!localSeriesArrows.TryGetValue(s, out var list)) { list = new List<ArrowAnnotation>(); localSeriesArrows[s] = list; }
						list.Add(a);
					}
				}
			}

			// add module series (only those with points)
			foreach (var ms in moduleSeries.Values)
				if (ms.Points.Count > 0) pm.Series.Add(ms);

			 // Compute plotted data extents so we can choose a sensible initial view.
			// Default requirement: origin at lower-left and X span should show 9 meters.
			double dataMinX = double.PositiveInfinity, dataMaxX = double.NegativeInfinity;
			double dataMinY = double.PositiveInfinity, dataMaxY = double.NegativeInfinity;
			foreach (var ss in pm.Series.OfType<ScatterSeries>())
			{
				foreach (var p in ss.Points)
				{
					if (double.IsNaN(p.X) || double.IsNaN(p.Y)) continue;
					dataMinX = Math.Min(dataMinX, p.X);
					dataMaxX = Math.Max(dataMaxX, p.X);
					dataMinY = Math.Min(dataMinY, p.Y);
					dataMaxY = Math.Max(dataMaxY, p.Y);
				}
			}

			// If no data found, default to 0..9 both axes.
			if (double.IsInfinity(dataMinX))
			{
				dataMinX = 0; dataMaxX = 9;
				dataMinY = 0; dataMaxY = 9;
			}

			// Ensure the visible X range includes 0..9 (origin lower-left by default).
			var xMin = Math.Min(0.0, dataMinX);
			var xMax = Math.Max(9.0, dataMaxX);
			var yMin = Math.Min(0.0, dataMinY);
			var yMax = Math.Max(9.0, dataMaxY);

			 // Defer applying the initial zoom until the first Updated callback so OxyPlot's
			 // internal autoscale won't immediately override our requested view.
			 // Store desired view and the Updated handler will apply it once.
			s_initialViews[pm] = (xMin, xMax, yMin, yMax);

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

			 // Ensure a "map" series exists when a background image is pending so the legend/filters include it.
			// This must run before TryAddAnnotationLegend so the created series appears in the legend list.
			try
			{
				if (s_pendingFieldImages.TryGetValue(pm, out var pending) && pending != null)
				{
					const string mapTitle = "map";
					if (!pm.Series.Any(s => string.Equals(s.Title, mapTitle, StringComparison.OrdinalIgnoreCase)))
					{
						var ms = new ScatterSeries
						{
							Title = mapTitle,
							MarkerType = MarkerType.None,
							MarkerSize = 0,
							MarkerFill = OxyColors.Transparent
						};
						// insert at front so legend shows it near top
						pm.Series.Insert(0, ms);

						// ensure extra-annotation mapping exists so RegisterAnnotationUnderMap will find it
						if (!s_seriesExtraAnnotations.TryGetValue(pm, out var mapDict) || mapDict == null)
						{
							mapDict = new Dictionary<Series, List<Annotation>>();
							s_seriesExtraAnnotations[pm] = mapDict;
						}
						if (!mapDict.ContainsKey(ms)) mapDict[ms] = new List<Annotation>();
					}
				}
			}
			catch
			{
				// tolerant — legend inclusion is a nicety
			}

			 // Build a guaranteed-visible annotation legend (fallback in case built-in legend doesn't show).
			TryAddAnnotationLegend(pm);

			// Updated handler to lock X and Y scales together
			pm.Updated += (s, e) =>
			{
				try
				{
					var xAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Bottom) ?? pm.Axes.FirstOrDefault();
					var yAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Left) ?? pm.Axes.Skip(1).FirstOrDefault() ?? xAxis;
					var plotArea = pm.PlotArea;
					if (xAxis == null || yAxis == null || plotArea.Width <= 0 || plotArea.Height <= 0) return;

					 // Diagnostic: report plot area & axes extents early for debugging pixel math
					try
					{
						s_log?.Invoke($"Updated:start plotArea={plotArea.Width:F1}x{plotArea.Height:F1} LeftTop={plotArea.Left:F1},{plotArea.Top:F1} axesCount={pm.Axes.Count}");
					}
					catch { }

					// Apply the initial view once
					if (!s_initialViewApplied.Contains(pm) && s_initialViews.TryGetValue(pm, out var initial))
					{
						try
						{
							var desiredXMin = initial.xMin;
							var desiredXMax = initial.xMax;
							var desiredXRange = Math.Max(1e-6, desiredXMax - desiredXMin);

							// Compute the corresponding Y range for isotropic scaling
							var desiredYRange = desiredXRange * (plotArea.Height / Math.Max(1.0, plotArea.Width));
							var desiredYMin = 0.0;
							var desiredYMax = desiredYMin + desiredYRange;

							xAxis.Zoom(desiredXMin, desiredXMax);
							yAxis.Zoom(desiredYMin, desiredYMax);

							// Schedule a tiny imperceptible nudge asynchronously on the UI dispatcher.
							// Doing this after a short delay avoids racing with OxyPlot's internal layout
							// passes; the follow-up Updated will run with stable ActualMinimum/Maximum.
							try
							{
								var eps = Math.Max(1e-12, desiredXRange * 1e-6); // tiny fraction of X range
								// fire-and-forget delayed nudge
								_ = Task.Run(async () =>
								{
									try
									{
										await Task.Delay(50).ConfigureAwait(false); // small delay to let layout settle
                                                                                    // marshal back to UI thread to call Axis.Zoom / InvalidatePlot
                                        (Application.Current?.Dispatcher)?.Invoke(() =>
                                            {
                                                try
                                                {
                                                    // nudge then restore
                                                    xAxis.Zoom(desiredXMin + eps, desiredXMax + eps);
                                                    xAxis.Zoom(desiredXMin, desiredXMax);
                                                    // force a full update (updateData = true) so transforms/layout are recomputed
                                                    // — this matches the effect of a real mouse interaction which forces OxyPlot
                                                    // to run its update pass and produce stable ActualMinimum/Maximum values.
                                                    pm.InvalidatePlot(true);
                                                }
                                                catch { /* best-effort */ }
                                            });
                                    }
									catch { /* best-effort */ }
								});
							}
							catch { /* best-effort nudge - ignore failures */ }

							// mark applied and remove stored desired view
							s_initialViewApplied.Add(pm);
							s_initialViews.Remove(pm);
							s_log?.Invoke($"Applied isotropic initial view X={desiredXMin:F3}..{desiredXMax:F3}, Y={desiredYMin:F3}..{desiredYMax:F3} (nudge applied)");

							// Defer image placement until after initial view applied and layout is stable:
							if (s_pendingFieldImages.TryGetValue(pm, out var pending) && pending != null && !pending.Added)
							{
								try
								{
									// construct OxyImage now and place using the same math previously used
									var oxyImg = new OxyImage(pending.Bytes);
									// pick branch based on metadata
									if (pending.ExtentW.HasValue && pending.ExtentW.Value > 0 && pending.FieldWidthMeters.HasValue)
									{
										double fWm = pending.FieldWidthMeters.Value;
										double fHm = pending.FieldHeightMeters ?? (fWm * (pending.ExtentH.HasValue && pending.ExtentW.HasValue ? ((double)pending.ExtentH.Value / pending.ExtentW.Value) : 1.0));
										double pixelSizeMeters = fWm / (double)pending.ExtentW.Value;

										 // compute top-left in data coords (convert origin Y from bottom->top)
										double topLeftDataX = -((pending.OriginX ?? 0) * pixelSizeMeters);
										double originYPx = pending.OriginY ?? 0;
										double extentHPx = oxyImg.Height; // safe fallback
										double topLeftDataY = ((extentHPx - originYPx) * pixelSizeMeters);

										 // Explicit image physical size (entire image converted to meters)
										double imgWidthMeters = pixelSizeMeters * (double)oxyImg.Width;
										double imgHeightMeters = pixelSizeMeters * (double)oxyImg.Height;

										s_log?.Invoke($"PlotHelper(Updated): placing deferred image scaled -> imageMeters={imgWidthMeters:F3}m x {imgHeightMeters:F3}m pixel->m={pixelSizeMeters:F6} originPx=({pending.OriginX},{pending.OriginY}) topLeftData=({topLeftDataX:F3},{topLeftDataY:F3})");

										var imgAnn = new ImageAnnotation
										{
											ImageSource = oxyImg,
											Opacity = 1.0,
											Interpolate = false,
											Layer = AnnotationLayer.BelowSeries,
											HorizontalAlignment = OxyPlot.HorizontalAlignment.Left,
											VerticalAlignment = OxyPlot.VerticalAlignment.Top,
											X = new OxyPlot.PlotLength(topLeftDataX, OxyPlot.PlotLengthUnit.Data),
											Y = new OxyPlot.PlotLength(topLeftDataY, OxyPlot.PlotLengthUnit.Data),
											// use full image size converted to meters (not any subset)
											Width = new OxyPlot.PlotLength(imgWidthMeters, OxyPlot.PlotLengthUnit.Data),
											Height = new OxyPlot.PlotLength(imgHeightMeters, OxyPlot.PlotLengthUnit.Data)
										};
										pm.Annotations.Add(imgAnn);
										s_log?.Invoke($"PlotHelper(Updated): added deferred image annotation (data bounds) for '{pending.PngPath}' (image size used)");
										RegisterAnnotationUnderMap(pm, imgAnn);

										// Diagnostic: add a small visible marker at data (0,0) so we can confirm where the JSON-origin maps in plot coords.
										try
										{
											var originMarker = new EllipseAnnotation
											{
												X = 0.0,
												Y = 0.0,
												Width = Math.Max(imgWidthMeters, 1.0) * 0.005,  // tiny relative marker
												Height = Math.Max(imgHeightMeters, 1.0) * 0.005,
												Fill = OxyColor.FromArgb(200, 255, 0, 0),
												Stroke = OxyColors.DarkRed,
												StrokeThickness = 1,
												Layer = AnnotationLayer.AboveSeries
											};
											pm.Annotations.Add(originMarker);

											// Also log where data (0,0) projects on the PlotView surface (plotArea-relative pixels).
											try
											{
												var data0x_px = plotArea.Left + xAxis.Transform(0.0);
												var data0y_px = plotArea.Top + yAxis.Transform(0.0);
												s_log?.Invoke($"PlotHelper(Updated): diagnostic: data origin (0,0) -> plotView-relative px = ({data0x_px:F1},{data0y_px:F1})");
												s_log?.Invoke($"PlotHelper(Updated): image data bounds -> left={topLeftDataX:F3} right={topLeftDataX + imgWidthMeters:F3} top={topLeftDataY:F3} bottom={topLeftDataY - imgHeightMeters:F3}");
											}
											catch { /* best-effort diagnostic */ }
										}
										catch { /* ignore marker failures */ }
									}
									else if (pending.ExtentW.HasValue && pending.ExtentH.HasValue)
									{
										var useW = pending.ExtentW.Value;
										var useH = pending.ExtentH.Value;
										// originY in JSON is bottom-based — compute top pixel index (from top) = extentH - originY
										var originYPx = pending.OriginY ?? 0;
										var topPx = useH - originYPx;
										var imgAnn = new ImageAnnotation
										{
											ImageSource = oxyImg,
											Opacity = 1.0,
											Interpolate = false,
											Layer = AnnotationLayer.BelowSeries,
											HorizontalAlignment = OxyPlot.HorizontalAlignment.Left,
											VerticalAlignment = OxyPlot.VerticalAlignment.Top,
											X = new OxyPlot.PlotLength(-(pending.OriginX ?? 0), OxyPlot.PlotLengthUnit.Data),
											Y = new OxyPlot.PlotLength(topPx, OxyPlot.PlotLengthUnit.Data),
											Width = new OxyPlot.PlotLength(useW, OxyPlot.PlotLengthUnit.Data),
											Height = new OxyPlot.PlotLength(useH, OxyPlot.PlotLengthUnit.Data)
										};
										pm.Annotations.Add(imgAnn);
										RegisterAnnotationUnderMap(pm, imgAnn);

										var left = -(pending.OriginX ?? 0);
										var right = left + useW;
										var top = topPx;
										var bottom = top - useH;

										imageMinX = imageMinX.HasValue ? Math.Min(imageMinX.Value, left) : left;
										imageMaxX = imageMaxX.HasValue ? Math.Max(imageMaxX.Value, right) : right;
										imageMinY = imageMinY.HasValue ? Math.Min(imageMinY.Value, bottom) : bottom;
										imageMaxY = imageMaxY.HasValue ? Math.Max(imageMaxY.Value, top) : top;

										s_log?.Invoke($"PlotHelper(Updated): added deferred pixel-units image annotation for '{pending.PngPath}' (originPx bottom-based converted to topPx={topPx})");

										// Diagnostic: add small marker at data (0,0) for pixel-units branch as well.
										try
										{
											var originMarker2 = new EllipseAnnotation
											{
												X = 0.0,
												Y = 0.0,
												Width = Math.Max(useW, 1.0) * 0.005,
												Height = Math.Max(useH, 1.0) * 0.005,
												Fill = OxyColor.FromArgb(200, 255, 0, 0),
												Stroke = OxyColors.DarkRed,
												StrokeThickness = 1,
												Layer = AnnotationLayer.AboveSeries
											};
											pm.Annotations.Add(originMarker2);
											try
											{
												var data0x_px = plotArea.Left + xAxis.Transform(0.0);
												var data0y_px = plotArea.Top + yAxis.Transform(0.0);
												s_log?.Invoke($"PlotHelper(Updated): diagnostic: data origin (0,0) -> plotView-relative px = ({data0x_px:F1},{data0y_px:F1})");
												s_log?.Invoke($"PlotHelper(Updated): image pixel-units bounds -> left={left} right={right} top={top} bottom={bottom}");
											}
											catch { }
										}
										catch { }
									}
									else
									{
										var ia = new ImageAnnotation
										{
											ImageSource = new OxyImage(pending.Bytes),
											Opacity = 1.0,
											Interpolate = false,
											Layer = AnnotationLayer.BelowSeries,
											X = new OxyPlot.PlotLength(0, OxyPlot.PlotLengthUnit.Data),
											Y = new OxyPlot.PlotLength(0, OxyPlot.PlotLengthUnit.Data),
											Width = new OxyPlot.PlotLength(1, OxyPlot.PlotLengthUnit.Data),
											Height = new OxyPlot.PlotLength(1, OxyPlot.PlotLengthUnit.Data),
											HorizontalAlignment = OxyPlot.HorizontalAlignment.Center,
											VerticalAlignment = OxyPlot.VerticalAlignment.Middle
										};
										pm.Annotations.Add(ia);
										RegisterAnnotationUnderMap(pm, ia);
										s_log?.Invoke($"PlotHelper(Updated): added deferred fallback image annotation for '{pending.PngPath}'");

										// Diagnostic marker for fallback case
										try
										{
											var originMarker3 = new EllipseAnnotation { X = 0, Y = 0, Width = 0.2, Height = 0.2, Fill = OxyColors.Red, Layer = AnnotationLayer.AboveSeries };
											pm.Annotations.Add(originMarker3);
											var data0x_px = plotArea.Left + xAxis.Transform(0.0);
											var data0y_px = plotArea.Top + yAxis.Transform(0.0);
											s_log?.Invoke($"PlotHelper(Updated): diagnostic fallback: data origin (0,0) -> plotView px ({data0x_px:F1},{data0y_px:F1})");
										}
										catch { }
									}
									pending.Added = true;
									// request a redraw so the image appears now that placement is set
									try { pm.InvalidatePlot(false); } catch { }
								}
								catch (Exception ie)
								{
									s_log?.Invoke($"PlotHelper(Updated): failed to place deferred image: {ie.Message}");
								}
							}

							// Bail out of this Updated invocation so subsequent Updated (triggered by the Zoom/nudge)
							// sees stable axis ActualMinimum/Maximum and arrow sizing can run safely.
							return;
						}
						catch { /* best-effort */ }
					}

					// Get current axis ranges
					double curXMin = xAxis.ActualMinimum, curXMax = xAxis.ActualMaximum;
					double curYMin = yAxis.ActualMinimum, curYMax = yAxis.ActualMaximum;
					double xRange = curXMax - curXMin;
					double yRange = curYMax - curYMin;
					if (xRange <= 0 || yRange <= 0) return;

					 // Compute pixels-per-unit for both axes
					double pxPerUnitX = plotArea.Width / xRange;
					double pxPerUnitY = plotArea.Height / yRange;

					 // Diagnostic: report computed px/unit and axis ranges
					try
					{
						s_log?.Invoke($"Updated: xRange={xRange:F3} yRange={yRange:F3} pxPerUnitX={pxPerUnitX:F3} pxPerUnitY={pxPerUnitY:F3}");
					}
					catch { }

					// Adjust the axis with the smaller px/unit to match the other
					if (Math.Abs(pxPerUnitX - pxPerUnitY) > 1e-3)
					{
						if (pxPerUnitX > pxPerUnitY)
						{
							// X is "zoomed out" more — adjust Y to match X scale
							double newYRange = xRange * (plotArea.Height / plotArea.Width);
							double yCenter = (curYMax + curYMin) * 0.5;
							yAxis.Zoom(yCenter - newYRange * 0.5, yCenter + newYRange * 0.5);
							s_log?.Invoke($"Adjusted Y to match X scale: Y={yCenter - newYRange * 0.5:F3}..{yCenter + newYRange * 0.5:F3}");
						}
						else
						{
							// Y is "zoomed out" more — adjust X to match Y scale
							double newXRange = yRange * (plotArea.Width / plotArea.Height);
							double xCenter = (curXMax + curXMin) * 0.5;
							xAxis.Zoom(xCenter - newXRange * 0.5, xCenter + newXRange * 0.5);
							s_log?.Invoke($"Adjusted X to match Y scale: X={xCenter - newXRange * 0.5:F3}..{xCenter + newXRange * 0.5:F3}");
						}
					}

					// Resize arrow heads to match current pixel scaling
					{
						// Recompute px/unit in case a Zoom above changed ranges
						xRange = xAxis.ActualMaximum - xAxis.ActualMinimum;
						yRange = yAxis.ActualMaximum - yAxis.ActualMinimum;
						pxPerUnitX = plotArea.Width / Math.Max(1e-9, xRange);
						pxPerUnitY = plotArea.Height / Math.Max(1e-9, yRange);
					}

					int logLimit = 5;
					int logged = 0;
					foreach (var ann in pm.Annotations.OfType<ArrowAnnotation>())
					{
						try
						{
							// Transform start/end data points to PlotView-relative pixels (use plotArea offset + axis.Transform).
							var p1x = plotArea.Left + xAxis.Transform(ann.StartPoint.X);
							var p1y = plotArea.Top  + yAxis.Transform(ann.StartPoint.Y);
							var p2x = plotArea.Left + xAxis.Transform(ann.EndPoint.X);
							var p2y = plotArea.Top  + yAxis.Transform(ann.EndPoint.Y);
							var pixelDx = p2x - p1x;
							var pixelDy = p2y - p1y;
							var pixelLen = Math.Sqrt(pixelDx * pixelDx + pixelDy * pixelDy);

							 // Diagnostic: log first few arrow pixel transforms / lengths
							if (logged < logLimit)
							{
								try
								{
									s_log?.Invoke($"Arrow[{logged}] data start=({ann.StartPoint.X:F3},{ann.StartPoint.Y:F3}) end=({ann.EndPoint.X:F3},{ann.EndPoint.Y:F3}) px start=({p1x:F1},{p1y:F1}) end=({p2x:F1},{p2y:F1}) lenPx={pixelLen:F1}");
								}
								catch { }
								logged++;
							}

							// scale arrow heads down to ~1/3 of previous visual size (adjustable factors)
							var newHeadLength = Math.Max(3.0, pixelLen * 0.12);
							var newHeadWidth  = Math.Max(2.0, pixelLen * 0.08);

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
		// Exclude the internal "map" series from the legend.
		var items = pm.Series
					  .Where(s => !string.IsNullOrWhiteSpace(s.Title) && !string.Equals(s.Title, "map", StringComparison.OrdinalIgnoreCase))
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
				FontWeight = OxyPlot.FontWeights.Bold,
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
	// Now also handles generic annotations (ImageAnnotation) registered under a series.
	public static void SetSeriesAnnotationsVisibility(PlotModel pm, Series series, bool visible)
	{
		if (pm == null || series == null) return;
		try
		{
			// remove/add ArrowAnnotation lists (existing behavior)
			if (s_seriesArrows.TryGetValue(pm, out var map) && map != null && map.TryGetValue(series, out var arrows) && arrows != null)
			{
				if (visible)
				{
					foreach (var a in arrows)
						if (!pm.Annotations.Contains(a)) pm.Annotations.Add(a);
				}
				else
				{
					foreach (var a in arrows)
						if (pm.Annotations.Contains(a)) pm.Annotations.Remove(a);
				}
			}

			// NEW: handle other annotations (images, rectangles, etc.)
			if (s_seriesExtraAnnotations.TryGetValue(pm, out var extraMap) && extraMap != null && extraMap.TryGetValue(series, out var anns) && anns != null)
			{
				if (visible)
				{
					foreach (var a in anns)
					{
						if (!pm.Annotations.Contains(a)) pm.Annotations.Add(a);
					}
				}
				else
				{
					foreach (var a in anns)
					{
						if (pm.Annotations.Contains(a)) pm.Annotations.Remove(a);
					}
				}
			}

			try { pm.InvalidatePlot(false); } catch { }
		}
		catch { /* best-effort */ }
	}

	// Register a non-arrow annotation (e.g. ImageAnnotation) under the special "map" series so it can be toggled.
	private static void RegisterAnnotationUnderMap(PlotModel pm, Annotation ann)
	{
		if (pm == null || ann == null) return;
		try
		{
			const string mapTitle = "map";
			// find or create the map series
			var mapSeries = pm.Series.FirstOrDefault(s => string.Equals(s.Title, mapTitle, StringComparison.OrdinalIgnoreCase));
			if (mapSeries == null)
			{
				// create an inert ScatterSeries used only for legend/filtering
				var ms = new ScatterSeries
				{
					Title = mapTitle,
					MarkerType = MarkerType.None,
					MarkerSize = 0,
					MarkerFill = OxyColors.Transparent
				};
				// insert at front so legend shows it near top
				pm.Series.Insert(0, ms);
				mapSeries = ms;
			}

			// ensure mapping dict exists for this PlotModel
			if (!s_seriesExtraAnnotations.TryGetValue(pm, out var mapDict) || mapDict == null)
			{
				mapDict = new Dictionary<Series, List<Annotation>>();
				s_seriesExtraAnnotations[pm] = mapDict;
			}

			if (!mapDict.TryGetValue(mapSeries, out var list) || list == null)
			{
				list = new List<Annotation>();
				mapDict[mapSeries] = list;
			}

			// avoid duplicates
			if (!list.Contains(ann)) list.Add(ann);
		}
		catch { /* tolerant */ }
	}
}
