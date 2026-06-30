# Non_grapic_simulator — Router Arduino Due sketches

This folder contains the Router-side simulator sketches for the Elcano trike
project plus a shared physics library.

```
Non_grapic_simulator/
├── README.md                              ← you are here
├── libraries/
│   └── simulator_physics/                 ← shared physics, single source of truth
│       ├── library.properties
│       └── src/simulator_physics.h
├── simulator_stage1/                      ← standalone Router sketch (Minhee)
│   └── simulator_stage1.ino
└── simulator_closed_loop/                 ← bridge closed-loop test sketch (Toprak)
    └── simulator_closed_loop.ino
```

## Required one-time setup (per developer)

Both sketches `#include <simulator_physics.h>` from the in-repo Arduino
library at `libraries/simulator_physics/`. For the Arduino IDE to find that
library, you must point its **Sketchbook Location** at this folder.

**Without this step the sketches will not compile** — the IDE will report
`fatal error: simulator_physics.h: No such file or directory`.

### Arduino IDE 2.x

1. **File → Preferences** (on macOS: **Arduino IDE → Settings**)
2. Set the **Sketchbook location** field to the full path of *this folder*
   (i.e. wherever your local clone of the repo puts `Non_grapic_simulator/`).
   Examples:
   ```
   Windows : C:\path\to\Bridge\Non_grapic_simulator
   macOS   : /Users/<you>/path/to/Bridge/Non_grapic_simulator
   Linux   : /home/<you>/path/to/Bridge/Non_grapic_simulator
   ```
3. Click **OK**
4. **Quit Arduino IDE completely**, then reopen it. (Some versions need the
   restart for library indexing to pick up the new path.)

### Arduino IDE 1.x

Same idea — **File → Preferences → Sketchbook location**.

## Install third-party libraries (closed-loop sketch only)

`simulator_closed_loop.ino` uses the **due_can** library to broadcast CAN
frames. `simulator_stage1.ino` does not need it.

Side effect of changing the Sketchbook Location: the Arduino IDE no longer
sees libraries that were installed in your *old* sketchbook (e.g. the default
`Documents/Arduino/libraries/due_can/`). Reinstall the dependencies into the
new sketchbook:

1. With Sketchbook Location set to this folder, open Arduino IDE
2. **Tools → Manage Libraries…**
3. Search for `due_can` and install (by Collin Kidder)

The library will land at `libraries/due_can/`. It's `.gitignore`d so it
won't get committed.

## Verify the setup worked

Before clicking Verify on either sketch, open **Sketch → Include Library**.
You should see `simulator_physics` listed under "Contributed libraries" or
"Custom libraries". If it's there, includes will resolve.

If it isn't there:

- The Sketchbook Location path is probably wrong. Open Preferences and confirm
  it points to **this folder** (the one containing both `libraries/` and the
  two sketch folders), not to `libraries/` itself and not to a sketch folder.
- Try restarting the IDE again.

## Why this folder layout?

Arduino IDE only searches a few places for `#include "..."` and
`#include <...>`: the active sketch folder, installed library `src/` folders,
the IDE-bundled libraries, and the Arduino core. It does **not** traverse up
into parent directories. Putting `simulator_physics.h` directly at the
project root would not work — sketches in subfolders wouldn't find it.

The library wrapper (`libraries/simulator_physics/`) is how Arduino IDE
exposes a single shared file to multiple sketches. The wrapper folders are
the Arduino convention, not decoration.

## Switching back to your normal sketchbook

If you do Elcano work in the morning and other Arduino projects in the
evening, you'll need to flip Sketchbook Location back to your default
(usually `Documents/Arduino`) for those projects. Arduino IDE 2.x supports
multiple windows — you can keep one window pointed here for Elcano and
another pointed at your default sketchbook for everything else.
