# Food appearance meshes

These original, procedurally generated COLLADA assets replace colored block
**visuals** in the sandwich scene:

- `bread_slice.dae`: loaf-slice outline, browned crust, rounded edge and porous
  cream crumb. Used by both independent bread bodies.
- `tomato_slice.dae`: rounded red skin, cut flesh, six gel compartments, pith
  and individual seeds. Opposing grasp lands are slightly flattened.
- `cheese_slice.dae`: rounded pale-yellow cheese with modeled openings and
  shaded internal walls.

The meshes are in meters, Z-up, centered on their existing ingredient links.
All visual vertices fit within the previously verified bounding dimensions.
There are no downloaded assets, texture dependencies or animation tracks.
Gazebo moves each visual with its existing dynamic ingredient body.

The scene scales each mesh along Z to match its rigid collision and planning
box. Each bread slice is 11 mm thick, the tomato is 6 mm and the cheese is
5 mm. These are design choices for this simulation. Together they form a
33 mm stack. Masses scale with volume and the generator computes box inertia
for the new dimensions. The crumb, seeds and cheese holes are visual details.

Regenerate the assets from the project root after editing the generator:

```bash
python3 -B experiments/sandwich_codex_experiment/scripts/food_visuals.py
python3 -B experiments/sandwich_codex_experiment/scripts/scene.py
```

`ensure_assets()` builds missing meshes automatically. Existing meshes are kept
until explicitly regenerated. `outputs/food_visual_update/asset_validation.json`
records dimensions/checksums; `physics_comparison.json` proves equality of the
nonvisual SDF before and after the appearance update. Historical recordings and
source snapshots remain unchanged.
