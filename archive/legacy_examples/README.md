# Legacy examples

## `exam-6.2-spp.cpp`

**Status:** archived / legacy

### Reason

- Uses an obsolete `SPPCode::full_solve()` calling convention.
- The reachable Git history does not contain the old implementation needed to
  recover the original parameter semantics.
- Adapting it to the current API would require assumptions about system and
  observation selection and historical output behavior, which could change the
  experiment semantics.
- It is therefore excluded from the active build and Windows migration
  validation.

Do not restore this example to the active build without first defining its
intended SPP configuration and output semantics.
