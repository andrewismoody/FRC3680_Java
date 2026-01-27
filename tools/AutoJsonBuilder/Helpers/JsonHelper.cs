using System.Text.Json;
using AutoJsonBuilder.Models;

namespace AutoJsonBuilder.Helpers;

public static class JsonHelper
{
	// Camel-case options (compact)
	public static readonly JsonSerializerOptions CamelOptions = new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase };

	// Camel-case indented
	public static readonly JsonSerializerOptions CamelOptionsIndented = new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase, WriteIndented = true };

	public static string SerializeCamel(object obj) => JsonSerializer.Serialize(obj, CamelOptions);

	public static string SerializeCamelIndented(object obj) => JsonSerializer.Serialize(obj, CamelOptionsIndented);

	// Deserialize with case-insensitive property matching (used by OpenJson flow)
	public static T? Deserialize<T>(string json)
	{
		var opts = new JsonSerializerOptions { PropertyNameCaseInsensitive = true };
		return JsonSerializer.Deserialize<T>(json, opts);
	}
}
