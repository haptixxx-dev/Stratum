#version 450

// ============================================================================
// Sky background
// ============================================================================
//
// Paints the analytic sky from sky_common.glsl -- the SAME function
// mesh_pbr.frag lights surfaces with. Anything that changes the look of the
// background changes the ambient light in step, which is the property a
// hand-picked clear colour next to a hand-picked ambient constant could never
// have.
//
// This replaces the flat {0.1, 0.1, 0.12} clear. That clear was the reason a
// wide shot ended in a void: with no sky there is nothing for distant geometry
// to fade into, so aerial perspective had nowhere to go and the horizon just
// stopped.
// ============================================================================

#include "sky_common.glsl"

layout(location = 0) in vec3 frag_ray_dir;

layout(location = 0) out vec4 out_color;

void main() {
    const vec3 dir = normalize(frag_ray_dir);

    // With the sun's disk: this is the only place it may be added. Every shading
    // path uses sky_radiance(), which excludes it, because the sun is already
    // integrated there as an analytic directional light.
    vec3 color = sky_with_sun(dir);

    // Same output transform as mesh_pbr.frag, in the same order, so the horizon
    // and the geometry in front of it agree. Exposure multiplies scene-referred
    // radiance BEFORE the tone curve; applying it after would only clip.
    color *= scene.camera_position.w;
    color = tonemap_aces(color);
    color = linear_to_srgb(color);

    out_color = vec4(color, 1.0);
}
