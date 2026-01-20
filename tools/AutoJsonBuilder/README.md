# AutoJsonBuilder

WPF utility for creating/editing the team “season JSON” file and writing it to the robot project deploy directory.

## Prerequisites

- Windows (WPF)
- .NET SDK 8.x

Verify:
- `dotnet --info`

## Build

From this folder:

```powershell
dotnet restore
dotnet build -c Release
```

## Run

### Option A: Run from source
```powershell
dotnet run -c Release
```

### Option B: Run the built executable
After building, run the generated exe from:

- `bin\Release\net8.0-windows\AutoJsonBuilder.exe`

## Using AutoJsonBuilder (create a season JSON from scratch)

1. Launch **AutoJsonBuilder**.
2. Create a new season file:
   - Use the app’s **New** / **New Season** / **New File** action (wording depends on the UI).
3. Fill out season metadata and content:
   - Enter the season/year and any required top-level fields.
   - Add events, matches, autos/paths, or any other season-specific sections your robot code expects.
4. Validate your JSON inside the tool (if the UI offers validation/errors, resolve them before exporting).
5. Save/export to a file:
   - Use **Save** / **Save As** / **Export JSON** (wording depends on the UI).

## Writing the JSON to the robot deploy directory

The robot code reads deploy assets from:

- `src/main/deploy`

So you want the final JSON to end up in (relative to the repo root):

- `./src/main/deploy/<your-season-file>.json`

### Recommended workflow

1. In AutoJsonBuilder, choose **Save As** (or equivalent).
2. Browse to your robot project’s deploy directory:
   - `D:\Repos\FRC3680-Java_2025\src\main\deploy`
3. Save the file with the name your robot code expects (example):
   - `season.json` (or your team’s configured name)

### Confirm it’s deployed

When you build/deploy the robot code, everything under `src/main/deploy` is copied to the robot’s “deploy” area.

If you’re unsure of the expected filename/path, search the robot codebase for:
- `getDeployDirectory()`
- references to `.json`
- whatever loader reads the season config

## Notes / Troubleshooting

- If the app fails to start, confirm you’re on Windows and using .NET 8.
- If the robot doesn’t “see” the file, double-check:
  - the JSON is inside `src/main/deploy`
  - the filename matches what the robot code loads
  - the JSON is valid (no trailing commas, correct schema for the season)
