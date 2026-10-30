# gz_terramechanics (vendored)

Gazebo Harmonic system plugin that applies Bekker-Wong terramechanics wheel-soil forces
(sinkage, slip-dependent traction, compaction resistance, lateral shear) to a four-wheel
rover. Used by the sim in `qpl_rover` (see its README, "Sand simulation").

Upstream: https://github.com/unitsSpaceLab/gz-terramechanics, commit `3c06ce3`
(University of Trieste Space Lab: Simone Cottiga, Khalid Bourr). Licensed GPL-3.0, see
`LICENSE`. The upstream `package.xml` said Apache-2.0, but the repo's `LICENSE` file is
GPL-3.0, so that is what applies here.

## Changes from upstream

- `wheel_params.yaml` takes `reversed_axis`, one bool per wheel. Upstream decided which
  wheels spin backwards when driving forward by looking for `fr`/`br` in the link name,
  which also matches `front_left_wheel`. Without the list it falls back to that rule.
- A stopped wheel no longer produces NaN forces: the lateral shear displacement and the
  shear stress direction divided by zero when omega and the shear velocity were zero, and
  the slip angle was atan(0/0). Any non-finite wrench is now skipped with a warning
  instead of being applied.
- Builds with ament_cmake: links OpenMP (it uses `omp.h` but didn't link it), installs
  the library and its example configs, and an environment hook puts `lib/` on
  `GZ_SIM_SYSTEM_PLUGIN_PATH` when the workspace is sourced.

`config/` holds upstream's example configs (for their Archimede rover). Our rover's are in
`qpl_rover/config/terramechanics/`.
