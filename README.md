To contribute to this Repo:
## Creating a New Branch
1. Clone this repo to your local machine:

  `git clone https://github.com/andrewismoody/FRC3680_Java.git`
  
3. Create a new local branch:

  `git checkout -B My-New-Branch`
  
5. Make your code changes
6. Test your code changes
7. Add your code changes to the commit:

  `git add .`
  
9. Commit your code changes to your local branch:

  `git commit`
  
11. Enter meaningful comments that describe what changes you made, what they are expected to do, and why.
12. Push your changes to the remote repo:

`git push -u origin My-New-Branch`

14. Submit a pull request to have your branch merged into the master branch

## Rebasing
If someone else has merged changes into master after you started working on your branch, you may need to incorporate those changes into your branch before you can merge.  To do this, you will need to:
1. Fetch the current version of the master branch from the remote repo:

  `git fetch origin master:master`
  
2. Rebase your branch onto the updated master branch:

  `git rebase -i master`
  
3. Resolve any conflicts that appear during your rebase
4. Re-push your changes to the remote repo:

  `git push --force`

# Auto JSON schema — high level

This README describes the top-level sections of the auto JSON document and what each section is used for. The UI Selection Details also shows these short descriptions above the JSON for any selected node.

- season
  - The season identifier for this auto definition (string).
- version
  - The document version (string).
- description
  - A free-text description of this file's purpose.
- params
  - Named parameters used by expressions. A parameter can be a numeric value, a string/expression, or a short array. Parameters are referenced by other fields (timings, offsets, etc.) and used in computed expressions.
- poses
  - Definitions of named poses. Each pose contains: group, location, index, position and action. Poses are referenced by events and targets to describe where something should happen.
- modules
  - A list of module labels (logical subsystems) that targets can reference.
- groups / travelGroups / locations / positions / actions
  - Lists of labels referenced by poses. `travelGroups` can be used for waypoint-like groups distinct from positioning groups.
- fixtures
  - Field fixtures: physical field points that can be used as targets or reference points. A fixture has a type, index and either an explicit translation or a derivation from other fixtures.
- events
  - Events describe triggers (time-based or await/pose-based) that cause targets to be executed.
- targets
  - Targets are the actionable items commanded during auto. A target may specify a translation, reference a fixture, provide a measurement, or request a module state.
- sequences
  - Ordered lists of event names that form higher-level actions.

Notes
- The selection details panel in the tool will show the above description followed by the JSON for the selected node so you can see what the value is and why it exists.
- Use the sections to organize behavior: poses and fixtures define locations; events and targets trigger actions at those locations; params hold reusable numeric or expression values.

