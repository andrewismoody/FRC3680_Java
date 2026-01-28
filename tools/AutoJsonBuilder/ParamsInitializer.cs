using System;
using System.Collections.Generic;
using AutoJsonBuilder.Models;

namespace AutoJsonBuilder
{
	// Helper that ensures required params exist and inserts defaults/expressions when missing.
	public static class ParamsInitializer
	{
		public static void EnsureParams(AutoDefinitionModel doc)
		{
			if (doc == null) throw new ArgumentNullException(nameof(doc));

			doc.Params ??= new Dictionary<string, ParamValue>(StringComparer.OrdinalIgnoreCase);

			void AddArrayIfMissing(string name, IList<object> value)
			{
				if (!doc.Params.ContainsKey(name))
					doc.Params[name] = new ParamValue { Raw = value };
			}
			void AddScalarIfMissing(string name, object value)
			{
				if (!doc.Params.ContainsKey(name))
					doc.Params[name] = new ParamValue { Raw = value };
			}

			// fieldSize default [690, 317]
			AddArrayIfMissing("fieldSize", new List<object> { 690, 317 });

			// fieldCenter always use expressions based on fieldSize
			AddArrayIfMissing("fieldCenter", new List<object> {
				"$fieldSize.X / 2",
				"$fieldSize.Y / 2"
			});

			// frameSize default [24,24]
			AddArrayIfMissing("frameSize", new List<object> { 24, 24 });

			// frameCenter based on frameSize
			AddArrayIfMissing("frameCenter", new List<object> {
				"$frameSize.X / 2",
				"$frameSize.Y / 2"
			});

			// bumperWidth scalar default 0
			AddScalarIfMissing("bumperWidth", 0);

			// robotSize based on frameSize and bumperWidth (use expressions)
			AddArrayIfMissing("robotSize", new List<object> {
				"$frameSize.X + $bumperWidth * 2",
				"$frameSize.Y + $bumperWidth * 2"
			});

			// robotCenter based on robotSize
			AddArrayIfMissing("robotCenter", new List<object> {
				"$robotSize.X / 2",
				"$robotSize.Y / 2"
			});

			// startArea default [90,30]
			AddArrayIfMissing("startArea", new List<object> { 90, 30 });

			// redStartTranslation default [ "$fieldCenter.X", "$fieldCenter.Y", 0 ]
			AddArrayIfMissing("redStartTranslation", new List<object> {
				"$fieldCenter.X",
				"$fieldCenter.Y",
				0
			});

			// redStartRotation default [0,0,180]
			AddArrayIfMissing("redStartRotation", new List<object> { 0, 0, 180 });

			// startPadding scalar default 0
			AddScalarIfMissing("startPadding", 0);

			// robotOffset based on robotCenter and startPadding
			AddArrayIfMissing("robotOffset", new List<object> {
				"$robotCenter.X + $startPadding",
				"$robotCenter.Y + $startPadding * 2"
			});

			// blueStartY1, blueStartY2, blueStartY3 scalar defaults 0
			AddScalarIfMissing("blueStartY1", 0);
			AddScalarIfMissing("blueStartY2", 0);
			AddScalarIfMissing("blueStartY3", 0);

			// blueStartPosition expression (X as complex expression, Y as fieldCenter.Y)
			AddArrayIfMissing("blueStartPosition", new List<object> {
				"$fieldCenter.X - $startArea.X + ($frameSize.X/2 + $bumperWidth/2 + (($startArea.X/2 - ($frameSize.X/2 + $bumperWidth/2)) ) )",
				"$fieldCenter.Y"
			});
		}
	}
}
