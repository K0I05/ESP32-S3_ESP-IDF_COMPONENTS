```markdown
# Contributing to ESP32-S3_ESP-IDF_COMPONENTS

Thank you for your interest in contributing! This repository collects ESP-IDF components and examples targeting the Espressif ESP32-S3. Contributions — whether bug reports, documentation improvements, new components, or fixes — are welcome and appreciated.

Please read these guidelines to make the contribution process smooth and predictable for everyone.

---

## Table of contents

- Repository scope
- Code of Conduct
- Primary development environment
- Getting started
- Development workflow
- Adding a new component
- Building & testing
- Issues
- Pull requests
- Commit messages & branch naming
- Style, linting & formatting
- Documentation & examples
- Security & sensitive data
- License & copyright
- Maintainers & reviews
- Thank you

---

## Repository scope

This repository is intended for:
- Reusable ESP-IDF components targeted to the ESP32-S3.
- Small example projects that demonstrate using those components.
- Utilities and helper scripts related to the components.

This repository is *not* intended for:
- Full product firmware that contains private or company-specific code.
- Large monolithic applications not meant to be reused as components.

If your contribution doesn't fit, consider opening an issue so we can advise.

---

## Code of Conduct

This project follows the Contributor Covenant Code of Conduct. Be respectful and inclusive. If you witness or experience unacceptable behavior, please open a private issue or contact a maintainer.

(If you want to add a formal CODE_OF_CONDUCT.md, please open a PR.)

---

## Primary development environment

The recommended and primary development environment for contributors is:

- Visual Studio Code (VS Code) with the PlatformIO extension (PlatformIO IDE / PlatformIO for VS Code).

Why this is the primary environment:
- PlatformIO offers an integrated workflow for building, flashing, and monitoring embedded projects directly from VS Code.
- It simplifies toolchain and environment management across platforms (Linux/macOS/Windows).
- It supports the Espressif platforms and can work with the ESP-IDF framework used by this repository's components.

Note: Native ESP-IDF tooling (idf.py, ESP-IDF VS Code extension) remains fully supported as an alternative — instructions for that workflow are included below. If you prefer native ESP-IDF, you can still build and test examples in the repo.

---

## Getting started

Prerequisites (general)
- VS Code (latest stable).
- PlatformIO extension for VS Code (recommended).
- PlatformIO Core (installed automatically by the extension; you can also install pio-cli separately).
- For native ESP-IDF alternative: Python 3.8+, ESP-IDF, and idf.py toolchain (see repository README for recommended ESP-IDF version).

Quickstart using VS Code + PlatformIO (recommended)

1. Install VS Code:
   - https://code.visualstudio.com/

2. Install PlatformIO extension:
   - Open VS Code → Extensions → search "PlatformIO IDE" (or "PlatformIO") → Install.
   - The extension will install PlatformIO Core (pio) automatically.

3. Open the repository:
   - File → Open Folder → select the repository root.

4. Create or configure a PlatformIO project for an example:
   - Option A — New project:
     - PlatformIO Home → New Project
     - Board: choose an ESP32-S3 board (for example, "esp32s3-devkitc-1")
     - Framework: Espressif IDF
     - Location: create the project at a convenient location, then copy or reference examples from this repo.
   - Option B — Use an example inside this repo:
     - Use a PlatformIO project that references the repo's components directory (see platformio.ini example below).

5. Example platformio.ini (place in the example folder or project root)
```ini
[env:esp32s3-devkitc-1]
platform = espressif32
board = esp32s3-devkitc-1
framework = espidf
monitor_speed = 115200

; If you want to use the components/ directory in this repository:
lib_extra_dirs = ../components

; Optional: build flags, custom flash args, etc.
; build_flags = -D SOME_MACRO=1
```

6. Build / Upload / Monitor:
   - Use the PlatformIO toolbar in VS Code (Build, Upload, Monitor), or
   - Use the CLI in the project folder:
     - pio run
     - pio run -t upload
     - pio device monitor

Notes on using components from this repo:
- PlatformIO with framework = espidf will respect the ESP-IDF component layout when the components directory is referenced (lib_extra_dirs or by opening the repo as the project root and placing a project under examples/ that uses the components/ path).
- Make sure each component has a correct CMakeLists.txt, Kconfig (if needed), and headers in include/.

Alternative: Native ESP-IDF (if you prefer)
1. Install ESP-IDF as described in the repository README and Espressif docs.
2. From an example folder:
   - idf.py set-target esp32s3
   - idf.py menuconfig
   - idf.py build
   - idf.py -p /dev/ttyUSB0 flash monitor

---

## Development workflow

1. Create a branch for your work:
   - Naming: `feature/<short-description>`, `fix/<short-description>`, or `docs/<short-description>`
   - Example: `feature/adc-driver-support` or `fix/cmake-memory-leak`

2. Keep changes focused per branch/PR. Prefer multiple small PRs to one large one.

3. Run builds and linters locally (using PlatformIO or idf.py) before opening a PR.

We use GitHub pull requests for code review and merging into the `main` branch.

---

## Adding a new component

When adding a new component to components/:

Recommended layout:
components/
  my_component/
    CMakeLists.txt
    Kconfig
    include/
      my_component.h
    src/
      my_component.c
    README.md
    examples/
      simple_example/
        main/main.c
        CMakeLists.txt
        sdkconfig.defaults

Required files
- `CMakeLists.txt`: follow ESP-IDF component conventions.
- `Kconfig`: add configuration options if needed.
- `README.md`: purpose, API overview, configuration notes, example usage, supported ESP-IDF versions.
- Public headers: place in `include/` and document functions/types.

Examples
- Include a small example demonstrating initialization and core usage.
- Provide `sdkconfig.defaults` for example-specific Kconfig defaults.

Third-party code
- If adding third-party code, include license notes and attribution in the component README.

---

## Building & testing

PlatformIO workflow
- Build: Use PlatformIO Build (VS Code) or pio run.
- Upload: PlatformIO Upload or pio run -t upload.
- Monitor: PlatformIO Monitor or pio device monitor.

Native ESP-IDF workflow
- Use `idf.py build`, `idf.py flash`, `idf.py monitor`.

CI
- PRs should pass configured CI checks (formatting, build for at least one target, basic tests).
- If you add new CI configuration, ensure it is deterministic and documents expectations.

Hardware testing
- When a change requires hardware verification, mention it in the PR and the reviewer will help coordinate testing.

Debugging
- Include logs from PlatformIO Monitor or `idf.py monitor`, and any gdb traces if relevant.
- Provide steps to reproduce and full environment details (board, toolchain version, wiring).

---

## Issues

Before opening an issue:
- Search existing issues to avoid duplicates.
- Provide minimal reproduction steps or a small example that reproduces the problem.
- Include:
  - ESP-IDF version and commit (if relevant)
  - PlatformIO version (if applicable) or idf.py/toolchain versions
  - Board/SoC (ESP32-S3)
  - Build logs and error messages
  - Hardware details (board revision, peripherals, wiring)

Issue templates
- Bug report: include reproduction steps, logs, and expected vs actual behavior.
- Feature request: describe the use-case, API proposal (if any), and backward compatibility concerns.

---

## Pull requests

Please follow this PR checklist when opening a pull request.

PR checklist
- [ ] Fork + branch created with clear name.
- [ ] Descriptive title and detailed description explaining what, why, and how.
- [ ] Link to any related issue(s) using #issue-number or full URL.
- [ ] All CI checks pass.
- [ ] Code compiles and examples build for the intended ESP-IDF version (or PlatformIO environment).
- [ ] Documentation and README updated where applicable.
- [ ] No secrets, passwords, or private keys included.

PR description should include:
- Summary of changes.
- How the change was tested (PlatformIO build, idf.py build, hardware tested on which board).
- Any backward compatibility breaks or API changes.
- Migration notes if a public API changed.

Review process
- Maintainers will review and may request changes.
- Respond to comments and update the PR as needed.
- Large or breaking changes should be discussed in an issue first.

Merging
- PRs are squash-merged by default unless the maintainers prefer preserving history.

---

## Commit messages & branch naming

Follow a conventional commit-like style for clarity. Example:
- feat(component): add non-blocking API for xxx
- fix(component): correct null-pointer on init
- docs(component): document new API options
- chore: update CI matrix

Branch naming:
- `feature/<short-desc>`, `fix/<short-desc>`, `docs/<short-desc>`, `test/<short-desc>`

---

## Style, linting & formatting

- C/C++:
  - Follow ESP-IDF coding style.
  - Use clang-format if configured. If a repository .clang-format exists, apply it.
- Python:
  - Use black and isort for formatting in scripts.
- Shell scripts:
  - POSIX-compatible shell where possible; lint with shellcheck.
- Kconfig/CMake:
  - Keep configuration readable and minimal. Use descriptive help text.

Run formatters and linters before opening PRs.

---

## Documentation & examples

- Update the component README for any API or behavior change.
- Examples should be small and annotated.
- If adding a new feature, add a short documentation section with sample code and expected behavior.
- Prefer Markdown and keep README sections: Overview, Requirements, Quickstart, API, Example, License.

---

## Security & sensitive data

- Do not commit secrets (API keys, private keys, passwords). If secrets are accidentally committed, open an issue and notify maintainers; we will advise remediation steps.
- For security vulnerabilities, contact maintainers privately (open a private GitHub issue or use the security contact listed in the repo). Do not disclose vulnerabilities in public issues or PRs.

---

## License & copyright

- All contributions are expected to be compatible with the repository license. Check the LICENSE file at the repo root.
- When contributing third-party code, ensure licensing is compatible and include attribution in the component README.

---

## Maintainers & reviews

- Maintainers are listed in the repository metadata (or README). They review PRs and triage issues.
- Maintainers may request changes or tests prior to merging.
- The maintainers reserve the right to close issues/PRs that are out of scope.

---

## Thank you

Thanks for helping make ESP32-S3_ESP-IDF_COMPONENTS better. Contributions keep this project useful for the community — every bug fix, documentation improvement, or new component matters.

If you want me to, I can open a branch and push this updated CONTRIBUTING.md and create a PR; tell me and I will create the branch and PR for you.



Copyright (c) 2025 Eric Gionet (<gionet.c.eric@gmail.com>)
