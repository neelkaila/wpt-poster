# Adaptive Wireless Power Transfer: project website

Static site for the EE2025 Team 49 project. Serve it with GitHub Pages: Settings → Pages → Branch `main`, folder `/ (root)`.

| File | What it is |
|---|---|
| `index.html` | The page: text, figures and the layout of the three simulations |
| `js/physics.js` | Circuit, decision rule, coupled-mode solver and mode-centred search (port of the Python project code) |
| `js/sims.js` | The interactive simulations (charging bay, resonance lab, results explorer) |
| `js/data.js` | Generated data: dense M(x, y, gap, tilt) tables for both pads, circuit parameters, and the recorded 1,485-pose results. Regenerate with `wpt_sim/web/build_data_js.py`; do not edit by hand |
| `img/` | Figures from the simulation |

To edit text, open `index.html`; each section starts with a `<!-- ====== SECTION ... ====== -->` comment.
Colours are at the top of the `<style>` block (`EDIT COLOURS HERE`, and the `.viz-root` block for the simulations).
