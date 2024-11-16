# Installation

## Depot installation (recommended)

To install the depot, first add the depot to your project

```bash
pros c add-depot https://github.com/alexDickhans/loco-lib/raw/refs/heads/depot/stable.json
```

Then add the locolib library to your project

```bash
pros c apply locolib
```

You're all set, take a look at the [Annotated Example](example.md) next!

## Manual installation

If you want to install manually, download the zip file from
the [most recent release](https://github.com/alexDickhans/loco-lib/releases/latest).

Then fetch the locolib zip

```bash
pros c fetch locolib@VERSION.zip
```

Apply the libcommand zip file to your project

```bash
cd YOUR_PROJECT_BASE
pros c apply libcommand
```

Add includes to your projects `main.h`

```c++
#include "localization/particleFilter.h"
```

## Next steps

You're all set, take a look at the [Annotated Example](example.md) next!
