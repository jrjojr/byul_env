# Contributing to BYUL

Thank you for your interest in BYUL.

BYUL is source-available, but it is not an open source project. Contributions are accepted only under the contribution terms below.

## License Reminder

This repository is licensed under the Byul World Source-Available Non-Commercial License v1.0.

- Personal learning, academic research, technical review, feedback, and non-commercial evaluation are permitted.
- Commercial use, redistribution, resale, public mirroring, sublicensing, and packaging as a library, SDK, plugin, or service are prohibited without prior written permission.
- See [`LICENSE`](../../LICENSE) for the full terms.

## Contribution Grant

Unless a separate written agreement exists, by submitting a contribution, patch, pull request, issue suggestion, or code proposal to this repository, you grant ByulPapa a perpetual, worldwide, royalty-free, non-exclusive license to use, modify, reproduce, distribute, sublicense, and relicense that contribution as part of the BYUL project.

This grant allows ByulPapa to maintain, modify, redistribute, and separately license the BYUL project, including contributions that are accepted into the project.

## Contributor Responsibility

By submitting a contribution, you confirm that:

- you have the right to submit it;
- it is your own original work, or you have permission to contribute it;
- it does not knowingly violate third-party copyright, patent, trademark, trade secret, license, or other rights;
- any third-party code, data, or asset included in the contribution is clearly identified.

## Code Style Notes

- Public headers should keep C ABI compatibility.
- Public functions should use `BYUL_API` when they are part of the exported API.
- Public API declarations should remain inside `extern "C"` blocks where applicable.
- Do not expose C++ STL types or C++ classes in public C ABI headers.
- Objects created by BYUL should be destroyed through BYUL-provided destroy/free functions.
- Do not throw C++ exceptions across the DLL boundary.

## Third-Party Materials

If a contribution includes or depends on third-party materials, update [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) with the relevant source, version, license, and purpose.

## Commercial Licensing

For commercial collaboration, paid project integration, business evaluation, commercial game use, simulation use, SDK/library packaging, or separate licensing inquiries, contact:

byuldev@outlook.kr
