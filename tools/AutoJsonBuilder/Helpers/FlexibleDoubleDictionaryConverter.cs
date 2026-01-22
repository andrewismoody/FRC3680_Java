using System;
using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace AutoJsonBuilder.Helpers;

internal sealed class FlexibleDoubleDictionaryConverter : JsonConverter<Dictionary<string, double>>
{
    public override Dictionary<string, double> Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
    {
        if (reader.TokenType != JsonTokenType.StartObject)
            throw new JsonException("Expected start of object for dictionary.");

        var dict = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);

        while (reader.Read())
        {
            if (reader.TokenType == JsonTokenType.EndObject) return dict;
            if (reader.TokenType != JsonTokenType.PropertyName) throw new JsonException();

            var propName = reader.GetString() ?? "";

            // Move to value
            if (!reader.Read()) throw new JsonException();

            double val = 0d; // default fallback

            switch (reader.TokenType)
            {
                case JsonTokenType.Number:
                    val = reader.GetDouble();
                    break;

                case JsonTokenType.String:
                    {
                        var s = reader.GetString();
                        if (!double.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out val))
                        {
                            // fallback to 0 on unparseable string
                            val = 0d;
                        }
                    }
                    break;

                case JsonTokenType.StartArray:
                    {
                        // scan array for the first numeric-looking element (number or numeric-string)
                        bool found = false;
                        while (reader.Read() && !found)
                        {
                            if (reader.TokenType == JsonTokenType.EndArray) break;

                            if (reader.TokenType == JsonTokenType.Number)
                            {
                                val = reader.GetDouble();
                                found = true;
                                // consume rest of array
                                while (reader.TokenType != JsonTokenType.EndArray && reader.Read()) { }
                                break;
                            }
                            else if (reader.TokenType == JsonTokenType.String)
                            {
                                var s = reader.GetString();
                                if (double.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var parsed))
                                {
                                    val = parsed;
                                    found = true;
                                    while (reader.TokenType != JsonTokenType.EndArray && reader.Read()) { }
                                    break;
                                }
                                // otherwise continue scanning; don't throw
                            }
                            else
                            {
                                // ignore other element types and continue scanning
                            }
                        }
                        // if nothing numeric found, val remains 0
                    }
                    break;

                case JsonTokenType.Null:
                    val = 0d;
                    break;

                default:
                    // unsupported token -> fallback to 0 rather than throwing
                    val = 0d;
                    break;
            }

            dict[propName] = val;
        }

        throw new JsonException("Unexpected end when reading dictionary.");
    }

    public override void Write(Utf8JsonWriter writer, Dictionary<string, double> value, JsonSerializerOptions options)
    {
        writer.WriteStartObject();
        foreach (var kvp in value)
        {
            writer.WritePropertyName(kvp.Key);
            writer.WriteNumberValue(kvp.Value);
        }
        writer.WriteEndObject();
    }
}
