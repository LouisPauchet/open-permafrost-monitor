# Contributing

## Workflow

1. Branch from `main` (`fix/…`, `feat/…`, `docs/…`).
2. Make the change. Keep comments explaining *why*, which is the style of
   this codebase.
3. `pio run -e walter` and `pio test -e native` must pass.
4. Test on hardware where behaviour changes; see
   [Build and test](building-and-testing.md#testing-on-hardware).
5. **Update the docs in the same pull request.** See below.
6. Open a pull request against `main`.

Report bugs and doc errors as GitHub issues. Include the console output
around the problem.

## Keeping docs in sync with code

The docs are only useful if they match the firmware. When you change one
of these, update the matching page:

| Code | Page |
|---|---|
| `include/pins.h` | [Wiring](../build/wiring.md) |
| `src/field_service.cpp`, `src/device_config.cpp` | [Configuration menu](../field/configuration.md), [Configuration model](../how-it-works/configuration-model.md) |
| `src/position_log.cpp` | [Positions CSV](../data/positions-csv.md), [MQTT](../data/mqtt.md) |
| `src/upload_client.cpp` | [Upload backend API](../data/upload-api.md) |
| `src/main.cpp` timers and fix cycle | [GNSS fix cycle](../how-it-works/scheduler.md), [Power](../how-it-works/power.md) |
| `src/sd_storage.cpp`, file naming | [SD card layout](../data/sd-layout.md), [Storage](../how-it-works/storage.md) |
| `src/miniseed_*`, `geophone_timing.h` | [MiniSEED](../data/miniseed.md), [Timing](../how-it-works/timing.md) |
| Console messages | [First boot](../build/first-boot.md), [Troubleshooting](../field/troubleshooting.md) |

## Editing this site

The site is [MkDocs](https://www.mkdocs.org/) with the
[Material](https://squidfunk.github.io/mkdocs-material/) theme. Pages are
Markdown in `docs/`, and the navigation is in `mkdocs.yml`.

```sh
pip install -r requirements-docs.txt
mkdocs serve            # live preview at http://127.0.0.1:8000
mkdocs build --strict   # what CI runs: fails on broken links
```

- **Diagrams** use [Mermaid](https://mermaid.js.org/) in a fenced
  ` ```mermaid ` block. They render on the site and on GitHub. Prefer a
  diagram over a paragraph for flows, states and sequences.
- **Admonitions:** `!!! warning "Title"`, `!!! tip`, `!!! danger`, etc.
- **Tabs:** `=== "VS Code"` / `=== "Command line"`.
- Copy console messages and menu labels **verbatim** from the source, so
  people can search for them.
- Pushes to `main` that touch the docs are published to GitHub Pages
  automatically (`.github/workflows/docs.yml`).
