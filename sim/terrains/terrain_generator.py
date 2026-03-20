"""Terrain heightfield generator for Phase 3.

Generates and populates heightfield data for grass and gravel terrains
in the MuJoCo model.  Called after model loading, before simulation.

Heightfield data is a 2D array of floats in [0, 1].  The physical height
at each grid point is:

    z = geom_pos_z + lerp(-hfield_size[3], +hfield_size[2], data)

where size[2] is the max elevation above the geom origin and size[3] is
the max depth below it.  With geom_pos_z = size[3] and data tapered to 0
at the edges, the lowest points sit at z = 0 (ground level), giving a
smooth transition to adjacent flat surfaces.
"""

import numpy as np
import mujoco


def populate_terrain_heightfields(model):
    """Fill heightfield data for grass and gravel terrains.

    Must be called after MjModel.from_xml_path() and before the first
    physics step.  The heightfields are declared in scene_terrain.xml
    with nrow/ncol but no file — this function writes the elevation data.
    """
    _populate_hfield(model, "grass_hfield", _generate_grass)
    _populate_hfield(model, "gravel_hfield", _generate_gravel)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _populate_hfield(model, name, generator_fn):
    """Look up a heightfield by name and fill its data."""
    hfid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, name)
    if hfid < 0:
        return  # not present in this scene

    nrow = model.hfield_nrow[hfid]
    ncol = model.hfield_ncol[hfid]
    adr = model.hfield_adr[hfid]

    data = generator_fn(nrow, ncol)
    model.hfield_data[adr:adr + nrow * ncol] = (
        data.ravel().astype(model.hfield_data.dtype)
    )


def _generate_grass(nrow, ncol):
    """Grass terrain: gentle, low-frequency undulations.

    Resembles a lawn with gentle mounds and dips from uneven settling
    and tree roots.  Wavelength ~0.5-1.0 m, amplitude ±15 mm.
    Heavy smoothing produces long-wavelength rolls.
    """
    rng = np.random.default_rng(42)
    data = rng.uniform(0.0, 1.0, (nrow, ncol)).astype(np.float32)
    data = _smooth_2d(data, iterations=10)
    data = _normalize(data)
    data = _taper_edges(data, margin=10)
    return data


def _generate_gravel(nrow, ncol):
    """Gravel terrain: rougher, higher-frequency bumps.

    Individual stones create a choppy, irregular surface with shorter
    wavelength variation than grass.  Amplitude ±25 mm.
    Less smoothing retains higher-frequency detail.
    """
    rng = np.random.default_rng(123)
    data = rng.uniform(0.0, 1.0, (nrow, ncol)).astype(np.float32)
    data = _smooth_2d(data, iterations=4)
    data = _normalize(data)
    data = _taper_edges(data, margin=8)
    return data


def _smooth_2d(data, iterations=5):
    """Smooth a 2D array with repeated [0.25, 0.5, 0.25] separable filter.

    Each iteration applies a 1-D triangle kernel along rows then columns.
    Multiple iterations approximate Gaussian smoothing.
    """
    kernel = np.array([0.25, 0.5, 0.25], dtype=np.float32)
    result = data.copy()
    for _ in range(iterations):
        # Along columns (axis 1)
        temp = np.empty_like(result)
        for i in range(result.shape[0]):
            temp[i] = np.convolve(result[i], kernel, mode="same")
        result = temp
        # Along rows (axis 0)
        temp = np.empty_like(result)
        for j in range(result.shape[1]):
            temp[:, j] = np.convolve(result[:, j], kernel, mode="same")
        result = temp
    return result


def _normalize(data):
    """Normalize data to [0, 1] range."""
    dmin, dmax = data.min(), data.max()
    if dmax - dmin < 1e-10:
        return np.full_like(data, 0.5)
    return (data - dmin) / (dmax - dmin)


def _taper_edges(data, margin=10):
    """Taper edges to 0 for smooth transition to adjacent ground surfaces.

    At the very edge (row/col 0), data = 0 → height = -hfield_size[3]
    relative to the geom origin.  With geom_pos_z = hfield_size[3], that
    puts the edge surface at world z = 0 (ground level).

    Uses min(row_weight, col_weight) so corners taper smoothly.
    """
    nrow, ncol = data.shape
    row_w = np.ones(nrow, dtype=np.float32)
    col_w = np.ones(ncol, dtype=np.float32)

    for i in range(min(margin, nrow // 2)):
        w = i / margin  # 0 at edge, approaches 1 at margin
        row_w[i] = w
        row_w[-(i + 1)] = w

    for j in range(min(margin, ncol // 2)):
        w = j / margin
        col_w[j] = w
        col_w[-(j + 1)] = w

    weight = np.minimum(row_w[:, np.newaxis], col_w[np.newaxis, :])
    return data * weight
