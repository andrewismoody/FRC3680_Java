using System;
using System.Globalization;
using System.Windows.Data;

namespace AutoJsonBuilder.Converters;

public sealed class InverseBooleanConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        => value is bool b ? !b : value;

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        => value is bool b ? !b : value;
}
