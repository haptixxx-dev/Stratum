#version 450

// ============================================================================
// Sky background -- fullscreen triangle
// ============================================================================
//
// No vertex buffer is bound and none is wanted: three vertices generated from
// gl_VertexIndex cover the whole target, and a single triangle rather than two
// avoids the diagonal seam where quad-based fullscreen passes double-shade the
// pixels either side of it.
//
// The vertex stage exists to turn the pixel into a WORLD-SPACE RAY, so the
// fragment stage never has to know about matrices. Unprojecting here and
// interpolating the result is exact: the ray origin is constant across the
// triangle and a perspective-correct interpolation of the near-plane point is
// the same point the fragment would have unprojected for itself.
// ============================================================================

// SDL_GPU: vertex uniform buffers are set 1, binding = the slot passed to
// SDL_PushGPUVertexUniformData. This is slot 0.
layout(set = 1, binding = 0) uniform SkyUniforms {
    mat4 inv_view_projection;
    vec4 camera_position;   // xyz = world position, w = unused
} sky;

layout(location = 0) out vec3 frag_ray_dir;

void main() {
    // (0,0), (2,0), (0,2) in UV -> (-1,-1), (3,-1), (-1,3) in NDC.
    const vec2 uv = vec2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2));
    const vec2 ndc = uv * 2.0 - 1.0;

    // REVERSE-Z. begin_render_pass() clears depth to 0 and every mesh pipeline
    // compares GREATER, so 0 is the far plane and 1 is the near plane. The sky
    // is emitted at the far plane and its pipeline disables the depth test and
    // depth writes outright, so this z only has to be a legal value -- geometry
    // is drawn after the sky and simply covers it.
    gl_Position = vec4(ndc, 0.0, 1.0);

    // Unproject the NEAR plane (z = 1 under reverse-Z) and aim from the eye
    // through it. Using the near plane rather than the far one matters: a
    // projection with an infinite far plane sends the far point to w = 0 and the
    // divide below would produce infinities.
    vec4 near_point = sky.inv_view_projection * vec4(ndc, 1.0, 1.0);
    frag_ray_dir = near_point.xyz / near_point.w - sky.camera_position.xyz;
}
