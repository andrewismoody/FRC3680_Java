using System;
using System.Collections.Generic;
using AutoJsonBuilder.Models;

namespace AutoJsonBuilder.Helpers;

internal static class PoseHelper
{
	public static string EnsureListHas(IList<string> list, string fallback)
	{
		if (list.Count == 0) list.Add(fallback);
		return list[0];
	}

	public static void EnsurePoseBackingLists(AutoDefinitionModel doc)
	{
		if (doc is null) return;
		EnsureListHas(doc.Groups, "group1");
		EnsureListHas(doc.Locations, "location1");
		EnsureListHas(doc.Positions, "position1");
		EnsureListHas(doc.Actions, "action1");
	}

	public static void EnsurePoseNames(AutoDefinitionModel doc)
	{
		if (doc?.Poses is null || doc.Poses.Count == 0) return;

		var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

		string NextPoseName()
		{
			var idx = 1;
			while (seen.Contains($"pose{idx}")) idx++;
			return $"pose{idx}";
		}

		for (var i = 0; i < doc.Poses.Count; i++)
		{
			var p = doc.Poses[i];
			var name = p.Name?.Trim();

			if (string.IsNullOrWhiteSpace(name))
			{
				var assigned = NextPoseName();
				p.Name = assigned;
				seen.Add(assigned);
				continue;
			}

			if (!seen.Add(name))
			{
				var baseName = name;
				var suffix = 1;
				string candidate;
				do
				{
					candidate = $"{baseName}_{suffix++}";
				} while (seen.Contains(candidate));
				p.Name = candidate;
				seen.Add(candidate);
			}
		}
	}
}
