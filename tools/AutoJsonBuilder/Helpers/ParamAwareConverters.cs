using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace AutoJsonBuilder.Helpers;

internal static class ParamAwareConverters
{
    // Build params map from root JsonElement (if present). Uses forgiving parsing like FlexibleDoubleDictionaryConverter.
    public static Dictionary<string, double> BuildParamsMap(JsonElement root)
    {
        var map = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
        if (!root.TryGetProperty("params", out var pElem) || pElem.ValueKind != JsonValueKind.Object) return map;

        foreach (var prop in pElem.EnumerateObject())
        {
            map[prop.Name] = ParseNumericElement(prop.Value);
        }
        return map;
    }

    private static double ParseNumericElement(JsonElement el)
    {
        switch (el.ValueKind)
        {
            case JsonValueKind.Number:
                return el.GetDouble();
            case JsonValueKind.String:
                {
                    var s = el.GetString();
                    if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var d)) return d;
                    return 0d;
                }
            case JsonValueKind.Array:
                {
                    foreach (var it in el.EnumerateArray())
                    {
                        if (it.ValueKind == JsonValueKind.Number) return it.GetDouble();
                        if (it.ValueKind == JsonValueKind.String)
                        {
                            var s = it.GetString();
                            if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var d)) return d;
                        }
                    }
                    return 0d;
                }
            case JsonValueKind.Null:
            default:
                return 0d;
        }
    }

    // Public helper: resolve "$name" or "${name}" using params map; returns true if resolved
    public static bool TryResolveParamString(string s, IDictionary<string, double>? paramsMap, out double value)
    {
        value = 0d;
        if (string.IsNullOrWhiteSpace(s) || paramsMap is null) return false;

        if (s.StartsWith("${") && s.EndsWith("}")) s = s.Substring(2, s.Length - 3);
        else if (s.StartsWith("$")) s = s.Substring(1);

        if (paramsMap.TryGetValue(s, out value)) return true;

        // Not a direct param name — maybe an expression. Heuristic: contains operator or parentheses.
        if (s.IndexOfAny(new[] { '+', '-', '*', '/', '(', ')', '^' }) >= 0)
        {
            // ask Java evaluator to compute (non-blocking call with timeout inside)
            var (ok, val) = AutoExprInterop.TryEvaluate(s, paramsMap);
            if (ok) { value = val; return true; }
        }

        return false;
    }
}

// Converter for double that can resolve param references using the provided params map.
internal sealed class ParamAwareDoubleConverter : JsonConverter<double>
{
    private readonly IDictionary<string, double> _params;

    public ParamAwareDoubleConverter(IDictionary<string, double> paramsMap) => _params = paramsMap ?? new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);

    public override double Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
    {
        if (reader.TokenType == JsonTokenType.Number) return reader.GetDouble();

        if (reader.TokenType == JsonTokenType.String)
        {
            var s = reader.GetString() ?? "";
            if (ParamAwareConverters.TryResolveParamString(s, _params, out var resolved)) return resolved;
            if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var d)) return d;
            return 0d;
        }

        if (reader.TokenType == JsonTokenType.StartArray)
        {
            // read until first numeric-like element
            while (reader.Read())
            {
                if (reader.TokenType == JsonTokenType.EndArray) break;
                if (reader.TokenType == JsonTokenType.Number) return reader.GetDouble();
                if (reader.TokenType == JsonTokenType.String)
                {
                    var s = reader.GetString() ?? "";
                    if (ParamAwareConverters.TryResolveParamString(s, _params, out var resolved)) return resolved;
                    if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var d)) return d;
                }
                // otherwise keep scanning
            }
            return 0d;
        }

        // null or other -> fallback
        if (reader.TokenType == JsonTokenType.Null) return 0d;
        return 0d;
    }

    public override void Write(Utf8JsonWriter writer, double value, JsonSerializerOptions options) => writer.WriteNumberValue(value);
}

// Converter for List<double> that accepts arrays with numbers/strings/param refs or single number/string.
internal sealed class ParamAwareDoubleListConverter : JsonConverter<List<double>>
{
    private readonly IDictionary<string, double> _params;

    public ParamAwareDoubleListConverter(IDictionary<string, double> paramsMap) => _params = paramsMap ?? new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);

    public override List<double> Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
    {
        var list = new List<double>();

        if (reader.TokenType == JsonTokenType.StartArray)
        {
            while (reader.Read())
            {
                if (reader.TokenType == JsonTokenType.EndArray) break;
                if (reader.TokenType == JsonTokenType.Number) list.Add(reader.GetDouble());
                else if (reader.TokenType == JsonTokenType.String)
                {
                    var s = reader.GetString() ?? "";
                    if (ParamAwareConverters.TryResolveParamString(s, _params, out var resolved)) list.Add(resolved);
                    else if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var d)) list.Add(d);
                    else list.Add(0d);
                }
                else if (reader.TokenType == JsonTokenType.Null) list.Add(0d);
                // ignore nested objects etc.
            }
            return list;
        }

        // accept single number or string as singleton list
        if (reader.TokenType == JsonTokenType.Number) { list.Add(reader.GetDouble()); return list; }
        if (reader.TokenType == JsonTokenType.String)
        {
            var s = reader.GetString() ?? "";
            if (ParamAwareConverters.TryResolveParamString(s, _params, out var resolved)) list.Add(resolved);
            else if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var d)) list.Add(d);
            else list.Add(0d);
            return list;
        }

        // other: return empty list
        return list;
    }

    public override void Write(Utf8JsonWriter writer, List<double> value, JsonSerializerOptions options)
    {
        writer.WriteStartArray();
        foreach (var d in value) writer.WriteNumberValue(d);
        writer.WriteEndArray();
    }
}
