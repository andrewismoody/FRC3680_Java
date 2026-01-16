# Auto Season JSON Authoring Guide

This folder contains JSON Schemas used to define **autonomous (auto) behavior** for a given FRC season. The root schema is:

- `auto.schema.json` — the top-level “season auto definition” file.

This README explains **how to construct a JSON file** that validates against `auto.schema.json`, what each required component does, and why the structure is intentionally modular.

---

## 1) What you are building

A season auto definition file is a single JSON object that contains:

- **Metadata** (season/version/description)
- A set of **vocabularies** (lists of allowed labels)
- A set of **reusable definitions** (poses, fixtures, events, targets)
- A set of **executable plans** (sequences)

The schema requires all of the following top-level properties:

```jsonc
{
  "$schema": "https://…/auto.schema.json",
  "season": "YYYY Game Name",
  "version": "x.y.z",
  "description": "Human-readable summary",

  "poses": [ /* pose objects */ ],
  "groups": [ /* string labels */ ],
  "locations": [ /* string labels */ ],
  "positions": [ /* string labels */ ],
  "actions": [ /* string labels */ ],

  "fixtures": [ /* fixture objects */ ],
  "events": [ /* event objects */ ],
  "targets": [ /* target objects */ ],
  "sequences": [ /* sequence objects */ ]
}
```

> Note: The detailed shape of `pose`, `fixture`, `event`, `target`, and `sequence` entries is defined in the referenced schemas:
> - `pose.schema.json`
> - `fixture.schema.json`
> - `event.schema.json`
> - `target.schema.json`
> - `sequence.schema.json`

This README focuses on **how the top-level file is organized and why**.

---

## 2) Authoring workflow (recommended)

1. **Decide your naming conventions**
   - Pick stable labels for groups/locations/positions/actions (they will be referenced across many definitions).
2. **Define vocabularies first**
   - Fill `groups`, `locations`, `positions`, `actions`.
3. **Define reusable “world facts”**
   - Add `fixtures` (things on the field you may interact with).
4. **Define reusable “robot intents/poses”**
   - Add `poses` that reference the vocabularies (and will be referenced by events/actions).
5. **Define event triggers**
   - Add `events` that represent observable moments/conditions and map them to targets.
6. **Define targets**
   - Add `targets` that represent objectives/goal states to pursue.
7. **Assemble sequences**
   - Add `sequences` that specify the ordered (and/or conditional) execution plan for auto.

This order minimizes backtracking: later objects tend to reference earlier definitions.

---

## 3) Top-level fields (what and why)

### `$schema` (optional but strongly recommended)
**What it is:** A string identifying which JSON Schema this file expects to validate against.

**Why it matters:** Editors and CI validators can automatically validate your file, autocomplete fields, and catch mistakes early. It also makes the file self-describing.

---

### `season` (required)
**What it is:** A season identifier (string).

**Why it matters:** Prevents mixing definitions across different games (field layout, fixtures, scoring logic) and provides a clear compatibility boundary for the code consuming the file.

---

### `version` (required)
**What it is:** A semantic (or otherwise consistent) version string.

**Why it matters:** Enables controlled evolution:
- You can change definitions while preserving older versions for past events.
- Software can enforce compatibility rules if needed.

---

### `description` (required)
**What it is:** Human-readable summary of what this definition set is intended for.

**Why it matters:** Auto definitions become “infrastructure.” The description is the cheapest way to avoid confusion when multiple people/competitions/configs exist.

---

## 4) Vocabularies (label lists)

These are arrays of strings:

- `groups`
- `locations`
- `positions`
- `actions`

All are **required** and must contain at least one entry.

### Why vocabularies exist
They provide a constrained dictionary of allowed labels to be referenced elsewhere (especially by `poses`). This is valuable because it:

- **Prevents drift**: “LeftGrid” vs “left_grid” vs “Left Grid” won’t silently multiply into separate meanings.
- **Enables validation**: schemas/code can verify a label belongs to the allowed set.
- **Improves reuse**: poses and sequences can reference a stable set of concepts.

### What each typically represents
(Your exact meaning is defined by your season + the referenced schemas.)

- **`groups`**: High-level categorization (e.g., alliance side, game-specific region, phase, “source vs. processor”).
- **`locations`**: Named field areas or anchor points (e.g., “Reef”, “HumanPlayerStation”, “Centerline”).
- **`positions`**: Sub-location positions within a location (e.g., “Left”, “Right”, “Bay3”, “SlotA”).
- **`actions`**: Named behaviors or operations used in poses/events (e.g., “Intake”, “ScoreHigh”, “Align”, “Wait”).

> Keep these labels stable. Renaming a label can break every reference to it.

---

## 5) `poses` (required)

**What it is:** An array of pose definitions (`pose.schema.json`).

**Why it is necessary:**
- Poses create a **shared language** for “where/how the robot should be” or “what configuration/intent is being referenced.”
- Events and sequences can refer to poses instead of duplicating the same coordinate/intent data repeatedly.

**What it enables:**
- Reusing the same pose in multiple sequences.
- Separating *what* a pose is from *when* it is used.
- Swapping or tuning poses without rewriting sequences.

**How it connects:**
- Poses typically reference the vocabulary labels (`groups`, `locations`, `positions`, `actions`) to stay consistent and validateable.

---

## 6) `fixtures` (required)

**What it is:** An array describing field fixtures (`fixture.schema.json`) — game elements or field features you might interact with.

**Why it is necessary:**
- Fixtures anchor auto logic to **real, named things** on the field.
- They allow events/targets to refer to game elements by ID/definition instead of hardcoding geometry/meaning in many places.

**What it enables:**
- Clear modeling of interactions: e.g., “approach fixture X”, “score at fixture Y”, “avoid fixture Z”.
- Season portability: each season’s fixtures differ, but the rest of the planning structure can remain consistent.

---

## 7) `events` (required)

**What it is:** An array of event definitions (`event.schema.json`) — triggers or conditions that can occur during auto.

**Why it is necessary:**
- Auto is not just a static timeline. The robot must react to conditions (reached a pose, acquired a game piece, timer threshold, sensor state, etc.).
- Events provide the “reactive layer” that turns a plan into a robust behavior.

**What it enables:**
- Branching logic (do X when condition Y becomes true).
- Synchronization (don’t proceed until a prerequisite is satisfied).
- Cleaner sequences: sequences reference named events instead of embedding raw conditions everywhere.

---

## 8) `targets` (required)

**What it is:** An array of target definitions (`target.schema.json`) — objectives that can be pursued.

**Why it is necessary:**
- Targets define the **intent/outcome** independent of the low-level steps.
- They provide a consistent interface between “what we want” and “how we get there.”

**What it enables:**
- Swapping strategies: keep the same target but change the sequence or constraints.
- Better composition: events can trigger pursuit/selection of targets.

---

## 9) `sequences` (required)

**What it is:** An array of sequence definitions (`sequence.schema.json`) — the actual executable flow for auto.

**Why it is necessary:**
- Sequences are where you put the “story” of auto: ordering, dependencies, branching, and which targets/actions to pursue.

**What it enables:**
- Multiple autos in one season file (e.g., “Safe 2-piece”, “Center 3-piece”, “Taxi + Score”).
- Strategy toggles without copying low-level definitions: sequences reuse poses/events/targets.

---

## 10) Why the schema is “complex” (and why that’s good)

At first glance, requiring all of these tables feels heavy. The complexity exists to provide **control without duplication**:

1. **Separation of concerns**
   - Vocabularies define allowed words.
   - Poses define reusable states/places.
   - Fixtures define the field.
   - Events define triggers.
   - Targets define goals.
   - Sequences define execution.

   This keeps each concept small, testable, and independently editable.

2. **Reusability**
   - A single pose/fixture/target can be used across many sequences.
   - You avoid copy/paste JSON that diverges over time.

3. **Validation and safety**
   - With schemas, tooling can catch:
     - missing required sections,
     - malformed objects,
     - inconsistent structure.
   - The required arrays (minItems = 1) ensure the file isn’t “empty but valid.”

4. **Strategic flexibility**
   - Real autos need both:
     - defined structure (so they’re reliable), and
     - configurable behavior (so they’re adaptable).
   - Modular definitions allow you to change strategy at the sequence level while preserving stable primitives.

5. **Scalability across a season**
   - Early events: simple “score + leave” autos.
   - Mid-season: multi-piece, conditional, field-dependent routines.
   - The same schema supports both without redesign.

---

## 11) Minimal starter template (fill in referenced object structures)

Use this as a starting point and then fill in each array with objects that match the referenced schemas:

```jsonc
{
  "$schema": "auto.schema.json",
  "season": "2025 ExampleSeason",
  "version": "1.0.0",
  "description": "Auto definitions for the 2025 season",

  "groups": ["ExampleGroup"],
  "locations": ["ExampleLocation"],
  "positions": ["ExamplePosition"],
  "actions": ["ExampleAction"],

  "poses": [
    // Must conform to pose.schema.json
  ],
  "fixtures": [
    // Must conform to fixture.schema.json
  ],
  "events": [
    // Must conform to event.schema.json
  ],
  "targets": [
    // Must conform to target.schema.json
  ],
  "sequences": [
    // Must conform to sequence.schema.json
  ]
}
```

---

## 12) Practical tips

- Treat label lists (`groups/locations/positions/actions`) like **enums**: stable, intentional, and reviewed.
- Prefer referencing named definitions (pose/fixture/event/target IDs) instead of duplicating raw details inside sequences.
- Keep sequences compact; push reusable details into the definition sections.
