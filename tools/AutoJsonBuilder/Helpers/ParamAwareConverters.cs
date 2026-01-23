using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace AutoJsonBuilder.Helpers;

internal static class ParamAwareConverters
{
    // Build params map preserving heterogeneous content:
    // - numeric scalars -> double
    // - textual values -> string (preserve expressions)
    // - arrays -> List<object> where elements are double or string
    public static Dictionary<string, object> BuildParamsMap(JsonElement root)
    {
        var map = new Dictionary<string, object>(StringComparer.OrdinalIgnoreCase);
        if (!root.TryGetProperty("params", out var pElem) || pElem.ValueKind != JsonValueKind.Object) return map;

        foreach (var prop in pElem.EnumerateObject())
        {
            var val = ParseElementPreserve(prop.Value);
            map[prop.Name] = val;
        }
        return map;
    }

    private static object ParseElementPreserve(JsonElement el)
    {
        switch (el.ValueKind)
        {
            case JsonValueKind.Number:
                return el.GetDouble();
            case JsonValueKind.String:
                // preserve textual expression as-is
                return el.GetString() ?? "";
            case JsonValueKind.Array:
                var list = new List<object>();
                foreach (var it in el.EnumerateArray())
                {
                    if (it.ValueKind == JsonValueKind.Number) list.Add(it.GetDouble());
                    else if (it.ValueKind == JsonValueKind.String) list.Add(it.GetString() ?? "");
                    else if (it.ValueKind == JsonValueKind.Null) list.Add(null!);
                    // ignore nested objects for params
                }
                return list;
            case JsonValueKind.Null:
            default:
                return 0d;
        }
    }

    // Resolve "$name" or "${name}" or compute simple expression using the params map.
    // Returns true if a numeric value could be determined.
    public static bool TryResolveParamString(string s, IDictionary<string, object>? paramsMap, out double value)
    {
        value = 0d;
        if (string.IsNullOrWhiteSpace(s) || paramsMap is null) return false;

        // Delegate all expression evaluation to the external expression server (no local parsing).
        // Important: send the string exactly as-is (do not strip leading '$' or modify the expression).
        try
        {
            var (ok, val) = AutoExprInterop.TryEvaluate(s, paramsMap);
            if (ok) { value = val; return true; }
        }
        catch
        {
            // fall through to numeric fallback
        }

        // Fallback: if the token is a plain numeric literal, parse it locally.
        if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var d))
        {
            value = d;
            return true;
        }

        return false;
    }
}

// Converter for double that can resolve param references using the provided params map.
internal sealed class ParamAwareDoubleConverter : JsonConverter<double>
{
    private readonly IDictionary<string, object> _params;

    public ParamAwareDoubleConverter(IDictionary<string, object> paramsMap) => _params = paramsMap ?? new Dictionary<string, object>(StringComparer.OrdinalIgnoreCase);

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
    private readonly IDictionary<string, object> _params;

    public ParamAwareDoubleListConverter(IDictionary<string, object> paramsMap) => _params = paramsMap ?? new Dictionary<string, object>(StringComparer.OrdinalIgnoreCase);

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
