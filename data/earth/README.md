# Earth data for DSKY-matic launch visualization

These binary files are preprocessed GSHHG (Global Self-consistent,
Hierarchical, High-resolution Geography) shoreline / lake data, loaded
at runtime by `dskymatic-studio`'s `MapView` to render a stylized
Earth with coastline art, filled continents, and lake "holes" in
continental interiors.

| File | Format | Contents |
|------|--------|----------|
| `gshhs_i_land.bin`      | triangles  | 1,147 continental land polygons from GSHHG Level 1, earcut-triangulated and max-edge-subdivided to 2° arc |
| `gshhs_i_lakes.bin`     | triangles  | 1,293 inland lake polygons from GSHHG Level 2, same treatment |
| `gshhs_i_coastline.bin` | polylines  | Level 1 shoreline rings for outline overlay |

Binary layouts and preprocessing steps are documented in
`scripts/earth/gen_earth_data.py`.

## Regenerating from source

1. Download `gshhg-shp-2.3.7.zip` from
   http://www.soest.hawaii.edu/pwessel/gshhg/ (as of 2026-11, NOAA
   no longer hosts it and SOEST at U-Hawaii is the primary mirror).
2. Unzip into `data/earth/gshhg-source/` (so that
   `data/earth/gshhg-source/GSHHS_shp/i/GSHHS_i_L1.shp` exists).
3. Create a Python venv with `pyshp`, `mapbox_earcut`, `numpy`, and
   optionally `matplotlib` (for `--preview`):
   ```
   python3 -m venv /tmp/earth-venv
   /tmp/earth-venv/bin/pip install pyshp mapbox_earcut numpy matplotlib
   ```
4. Run the preprocessor:
   ```
   /tmp/earth-venv/bin/python scripts/earth/gen_earth_data.py \
       --gshhg-root data/earth/gshhg-source \
       --out-dir   data/earth \
       --preview
   ```

Default parameters (intermediate resolution, 2° max triangle edge,
0.3° minimum polygon bbox span) give ~7.8 MB of binaries at a quality
appropriate for the launch visualization's render scale.

## License / attribution

GSHHG is released into the public domain by Paul Wessel and Walter
Smith; see their license statement at
https://www.soest.hawaii.edu/pwessel/gshhg/.  No attribution is
required but credit is appreciated — the dataset is described in:

> Wessel, P., and W. H. F. Smith, A Global Self-consistent,
> Hierarchical, High-resolution Shoreline Database, J. Geophys.
> Res., 101(B4), 8741-8743, 1996.
