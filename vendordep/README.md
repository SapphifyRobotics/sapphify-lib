# Vendordep

Files for the pull request to
[`wpilibsuite/vendor-json-repo`](https://github.com/wpilibsuite/vendor-json-repo). The pull
request needs **both** of these, not just the first:

| File | Goes to |
|---|---|
| `SapphifyLib-2027.0.0-alpha-1.json` | `2027_alpha5/` in the bundle directory |
| `metadata-entry.json` | appended as one entry to `2027_alpha5_metadata.json` |

The `uuid` must be identical in both. It is what WPILib keys the library on, which is also why
neither the uuid nor the `name` can change after teams have installed it.

## The `website` field

It is the link teams follow from the WPILib dependency manager, and the first thing a reviewer
clicks. Of the 22 entries in `2027_alpha5_metadata.json`, **17 point at a documentation site** —
`docs.ctr-electronics.com`, `docs.revrobotics.com/brushless/revlib/revlib-overview`,
`docs.reduxrobotics.com/reduxlib`, `docs.photonvision.org`, `docs.advantagekit.org`. REVLib and
ReduxLib deep-link to the library's own page inside their docs. The remaining five point at a
GitHub repository (Studica, Choreo, maple-sim) or a company home page (AndyMark, Grapple).

**None of them points at a product page.** The entry describes the library, which covers every
device, so a single product's marketing page would be the wrong shape even if ROTEM is currently
the only device.

This entry therefore points at the GitHub organisation for now — the precedent Studica and Choreo
set, and honest, because that is where the specification, the source and the examples actually
live. It moves to `docs.sapphify.com` once the documentation site exists, which is a metadata-only
change and does not disturb the uuid.

An `instructions` field is optional; only 2 of the 22 use one. Add it when there is a real
getting-started page to point at.
