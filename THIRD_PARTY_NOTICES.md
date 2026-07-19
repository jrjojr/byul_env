# Third-Party Notices

This document records third-party components, tools, libraries, assets, or external materials used by the BYUL project.

The BYUL project license applies only to original BYUL project code, documentation, project-specific API design, original naming, and original worldbuilding material authored by ByulPapa, unless otherwise stated.

Third-party components remain under their respective licenses.

## Current Status

No bundled third-party source code or assets are listed here yet.

The project may reference external tools or libraries during development, testing, or building. When a third-party component is bundled, copied, vendored, or redistributed in this repository, it should be recorded below.

## Notice Template

Use this format when adding a third-party component:

```text
Name:
Source:
Version or commit:
License:
Purpose:
Files or directories:
Notes:
```

## Components

Name: Simple DirectMedia Layer (SDL)
Source: https://github.com/libsdl-org/SDL
Version or commit: release-3.2.18 / 68bfcb6c5419f51104e74e72ea0f8d405a4615b0
License: zlib License
Purpose: Cross-platform window, input, and OpenGL context support for GPU Compute Tester
Files or directories: external/SDL3-3.2.18/source
Notes: Git submodule pinned to the official SDL release tag. Existing MSVC and
MinGW development packages remain under external/SDL3-3.2.18.
