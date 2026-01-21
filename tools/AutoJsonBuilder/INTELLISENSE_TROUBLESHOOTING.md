# IntelliSense: `InitializeComponent` missing (WPF)

Symptom
- IntelliSense reports `InitializeComponent` (generated WPF partial) as missing while the project builds successfully.

Confirmed minimal fix
1. Uninstall or disable the **C# Dev Kit** (Roslyn-based) extension.
2. Install and enable only the official **C#** extension (`ms-dotnettools.csharp`).
3. In the C# extension settings, enable **OmniSharp** (force the C# extension to use OmniSharp as the language server).
4. Reload the VS Code window (Developer: Reload Window).

Notes
- After these steps, IntelliSense should correctly resolve generated `obj/* .g.cs` files (including `InitializeComponent`).
- If you still see problems, run `dotnet restore` and `dotnet build`, then reload VS Code.

Example (optional)
```
code --install-extension ms-dotnettools.csharp
```

This troubleshooting step was verified by removing C# Dev Kit and enabling OmniSharp in the C# extension; no other changes were necessary.
