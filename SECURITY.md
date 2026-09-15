# Security policy

## Supported versions

Stratum is pre-1.0. Only the latest tagged release receives fixes.

## Reporting a vulnerability

Report privately through GitHub's
[security advisory form](https://github.com/haptixxx-dev/Stratum/security/advisories/new).
Do not open a public issue for a vulnerability.

Expect an acknowledgement within seven days.

## Scope

Stratum parses untrusted input: `.osm` and `.pbf` files, heightmap images and
3D model files, all of which may come from anywhere. Parser crashes,
out-of-bounds reads and unbounded allocations reachable from a malformed input
file are in scope and are the most likely class of real vulnerability here.

Out of scope: crashes that need a deliberately corrupted internal scene file
you generated yourself, and anything requiring local write access to the
install directory.
