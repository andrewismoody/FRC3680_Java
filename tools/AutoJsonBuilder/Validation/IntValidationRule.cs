using System.Globalization;
using System.Windows.Controls;

namespace AutoJsonBuilder.Validation;

public sealed class IntValidationRule : ValidationRule
{
    public override ValidationResult Validate(object value, CultureInfo cultureInfo)
    {
        var s = value?.ToString()?.Trim() ?? "";
        if (s.Length == 0) return ValidationResult.ValidResult; // allow empty while typing

        return int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out _)
            ? ValidationResult.ValidResult
            : new ValidationResult(false, "Must be an integer");
    }
}
