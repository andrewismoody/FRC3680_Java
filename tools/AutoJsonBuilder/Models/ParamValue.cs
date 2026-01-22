using System;
using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace AutoJsonBuilder.Models;

// Small holder for a parameter value which may be a number, a string (expression), or an array.
// Raw holds a double, string, or List<object> (elements double|string).
[JsonConverter(typeof(ParamValueJsonConverter))]
public sealed class ParamValue
{
	public object? Raw { get; set; }

	public ParamValue() { }
	public ParamValue(object? raw) { Raw = raw; }

	public double? AsDouble()
	{
		if (Raw is double d) return d;
		if (Raw is float f) return Convert.ToDouble(f);
		if (Raw is int i) return Convert.ToDouble(i);
		if (Raw is long l) return Convert.ToDouble(l);
		if (Raw is string s && double.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var v)) return v;
		return null;
	}

	public IEnumerable<object>? AsArray() => Raw as IEnumerable<object>;

	public override string ToString() => Raw?.ToString() ?? "(null)";
}

internal sealed class ParamValueJsonConverter : JsonConverter<ParamValue>
{
	public override ParamValue Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
	{
		if (reader.TokenType == JsonTokenType.Null) return new ParamValue(null);

		if (reader.TokenType == JsonTokenType.Number)
		{
			var d = reader.GetDouble();
			return new ParamValue(d);
		}

		if (reader.TokenType == JsonTokenType.String)
		{
			var s = reader.GetString();
			return new ParamValue(s);
		}

		if (reader.TokenType == JsonTokenType.StartArray)
		{
			var list = new List<object>();
			while (reader.Read())
			{
				if (reader.TokenType == JsonTokenType.EndArray) break;
				if (reader.TokenType == JsonTokenType.Number) list.Add(reader.GetDouble());
				else if (reader.TokenType == JsonTokenType.String) list.Add(reader.GetString() ?? "");
				else if (reader.TokenType == JsonTokenType.Null) list.Add(null!);
				else
				{
					// skip/ignore nested objects but consume tokens
					reader.Skip();
				}
			}
			return new ParamValue(list);
		}

		// fallback: consume token and store its raw string
		var raw = reader.GetString();
		return new ParamValue(raw);
	}

	public override void Write(Utf8JsonWriter writer, ParamValue value, JsonSerializerOptions options)
	{
		var raw = value?.Raw;
		if (raw is null) { writer.WriteNullValue(); return; }

		switch (raw)
		{
			case double d: writer.WriteNumberValue(d); break;
			case float f: writer.WriteNumberValue(Convert.ToDouble(f)); break;
			case int i: writer.WriteNumberValue(i); break;
			case long l: writer.WriteNumberValue(l); break;
			case string s: writer.WriteStringValue(s); break;
			case IEnumerable<object> seq:
				writer.WriteStartArray();
				foreach (var e in seq)
				{
					if (e is double ed) writer.WriteNumberValue(ed);
					else if (e is float ef) writer.WriteNumberValue(Convert.ToDouble(ef));
					else if (e is int ei) writer.WriteNumberValue(ei);
					else if (e is long el) writer.WriteNumberValue(el);
					else if (e is string es) writer.WriteStringValue(es);
					else writer.WriteNullValue();
				}
				writer.WriteEndArray();
				break;
			default:
				writer.WriteStringValue(raw.ToString());
				break;
		}
	}
}
