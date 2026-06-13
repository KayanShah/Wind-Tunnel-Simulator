# Wind Tunnel Simulator

**Browser-based CFD wind tunnel with a real Lattice-Boltzmann fluid solver**  
Built by [Kayan Shah](https://github.com/KayanShah)

<br/>

[![Live Demo](https://img.shields.io/badge/Live%20Demo-Vercel-000000?style=for-the-badge&logo=vercel&logoColor=white)](https://wind-tunnel-simulator.vercel.app)
[![Built with](https://img.shields.io/badge/Built%20with-Vanilla%20JS-f7df1e?style=for-the-badge&logo=javascript&logoColor=black)](https://developer.mozilla.org/en-US/docs/Web/JavaScript)
[![Platform](https://img.shields.io/badge/Platform-Browser-4f8ef7?style=for-the-badge&logo=googlechrome&logoColor=white)](https://wind-tunnel-simulator.vercel.app)

<br/>

---

## ✦ About

A real-time aerodynamics simulator that runs a **D2Q9 Lattice-Boltzmann fluid solver** entirely in the browser. Drop any STL file in, dial in your conditions, and watch the flow field, pressure distribution, and forces update live — no server, no install.

No fluff. Just real CFD, in a tab.

---

## ✦ Features

| | Feature |
|---|---|
| 🌊 | **Lattice-Boltzmann Solver** — genuine D2Q9 CFD; flow goes around the body, forms a real wake, forces read from momentum exchange |
| 📦 | **STL Import** — drag-and-drop any STL mesh; cross-sectioned and rasterised into the solver automatically |
| 📐 | **Live Metrics** — Reynolds number, Mach, Cd, Cl, drag/lift forces, dynamic pressure, L/D ratio |
| 🎨 | **Flow Overlays** — streamlines, pressure field, force arrows, boundary layer highlight, labels, grid |
| 📊 | **Four Graphs** — Cp distribution, force bars, silhouette profile, and Cl/Cd polar curve |
| 📖 | **Live Equations** — KaTeX-rendered Re, q, Fd, Fl equations that update with your inputs |
| 💡 | **What-If Coach** — plain-English tips on speed, stall, compressibility, and regime |
| 📤 | **Export** — PNG snapshot, CSV sweep (1–515 m/s), and full PDF report with all four graphs |
| 🔄 | **Undo / Redo** — full history for speed, size, AoA, and object changes |
| 🌍 | **SI + Imperial** — toggle between m/s / N / Pa and mph / lbf / psi at any time |

---

## ✦ Tech Stack

```
Solver       →  D2Q9 Lattice-Boltzmann (vanilla JS, typed arrays)
Math render  →  KaTeX 0.16.8
PDF export   →  jsPDF 2.5.1
Platform     →  Single HTML file — no build step, no framework
Deploy       →  Vercel
```

---

## ✦ Project Structure

```
Wind Tunnel Simulator/
└── index.html    ← everything: solver, renderer, UI, exports
```

---

## ✦ How It Works

The solver is a real CFD implementation, not a lookup table:

1. The body (sphere or STL cross-section) is rasterised into solid no-slip cells
2. Each frame runs **3 Lattice-Boltzmann substeps** — collision + streaming + bounce-back
3. Forces are extracted from **momentum exchange** on the boundary cells
4. Cd and Cl are calibrated from the measured flow field, then fed into standard aerodynamic relations for Fd, Fl, and L/D

Compressibility corrections (Prandtl-Glauert below M 0.95, wave drag above) and a full stall model (Hoerner/Viterna, valid ±90°) are applied on top.

---

## ✦ Setup

No install needed. Open `index.html` directly in a browser, or visit the live demo.

To run locally:

```bash
git clone https://github.com/KayanShah/Wind-Tunnel-Simulator.git
cd Wind-Tunnel-Simulator
open index.html
```

---

## ✦ License

This project is licensed under the **[Creative Commons Attribution 4.0 International (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/)** licence.

You are free to use, modify, and distribute this project provided that you:

- **Credit the author** — Kayan Shah
- **Link to the source** — [github.com/KayanShah](https://github.com/KayanShah)
- **Indicate if changes were made**

> © 2026 Kayan Shah · [github.com/KayanShah](https://github.com/KayanShah)

---

<div align="center">

<br/>

Made by **[Kayan Shah](https://github.com/KayanShah)**

<br/>

</div>
