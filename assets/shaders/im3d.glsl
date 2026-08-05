#version 450

// Im3d debug-geometry shader for the SDL_GPU (Vulkan) backend.
//
// Expands each Im3d primitive into screen-space geometry via instancing, pulling
// Im3d::VertexData straight out of a storage buffer. There is no geometry shader
// stage in SDL_GPU and SDL_GPURasterizerState has no line-width field, so the
// per-vertex pixel size in m_positionSize.w has to become real geometry here.
// Modelled on external/im3d/examples/OpenGL31/im3d.glsl.
//
// THERE IS NO BUILD-TIME SHADER STEP. The .spv files beside this source are
// committed artifacts. After editing this file, regenerate all six by hand:
//
//   for p in points lines triangles; do
//     P=$(echo $p | tr a-z A-Z)
//     glslc -fshader-stage=vert -DVERTEX_SHADER -D$P assets/shaders/im3d.glsl \
//           -o assets/shaders/im3d_$p.vert.spv
//     glslc -fshader-stage=frag -DFRAGMENT_SHADER -D$P assets/shaders/im3d.glsl \
//           -o assets/shaders/im3d_$p.frag.spv
//   done

#if !defined(POINTS) && !defined(LINES) && !defined(TRIANGLES)
	#error No primitive type defined
#endif

#define kAntialiasing 2.0

#ifdef VERTEX_SHADER

// SDL_GPU: vertex-stage storage buffers live in set 0.
// Declared as a flat uint array so the std430 array stride is 4 bytes and the
// CPU-side 20-byte Im3d::VertexData stride maps 1:1 with no padding.
layout(std430, set = 0, binding = 0) readonly buffer VertexDataBlock {
	uint uVertexData[];
};

// SDL_GPU: vertex-stage uniform buffers live in set 1.
layout(set = 1, binding = 0) uniform Im3dUniforms {
	mat4 uViewProjMatrix;
	vec2 uViewport;
	uint uFirstVertex;
	uint uPad;
};

vec4 GetPositionSize(int _i)
{
	int b = (int(uFirstVertex) + _i) * 5;
	return vec4(
		uintBitsToFloat(uVertexData[b + 0]),
		uintBitsToFloat(uVertexData[b + 1]),
		uintBitsToFloat(uVertexData[b + 2]),
		uintBitsToFloat(uVertexData[b + 3]));
}

vec4 GetColor(int _i)
{
	uint u = uVertexData[(int(uFirstVertex) + _i) * 5 + 4]; // Im3d::Color packs 0xRRGGBBAA
	return vec4(
		float((u & 0xff000000u) >> 24) / 255.0,
		float((u & 0x00ff0000u) >> 16) / 255.0,
		float((u & 0x0000ff00u) >>  8) / 255.0,
		float((u & 0x000000ffu)      ) / 255.0);
}

#if   defined(POINTS)
	layout(location = 0) noperspective out vec2 vUv;
#elif defined(LINES)
	layout(location = 0) noperspective out float vEdgeDistance;
#endif
layout(location = 1) noperspective out float vSize;
layout(location = 2) smooth        out vec4  vColor;

void main()
{
	// Quad corners derived from gl_VertexIndex (no vertex buffer is bound).
	// 0:(-1,-1) 1:(1,-1) 2:(-1,1) 3:(1,1) -> valid triangle strip.
	vec2 aPosition = vec2(
		((gl_VertexIndex & 1) == 0) ? -1.0 : 1.0,
		(gl_VertexIndex < 2)        ? -1.0 : 1.0);

#if defined(POINTS)
	int vid = gl_InstanceIndex;
	vSize  = max(GetPositionSize(vid).w, kAntialiasing);
	vColor = GetColor(vid);
	vColor.a *= smoothstep(0.0, 1.0, vSize / kAntialiasing);
	gl_Position = uViewProjMatrix * vec4(GetPositionSize(vid).xyz, 1.0);
	vec2 scale = 1.0 / uViewport * vSize;
	gl_Position.xy += aPosition.xy * scale * gl_Position.w;
	vUv = aPosition.xy * 0.5 + 0.5;

#elif defined(LINES)
	int vid0 = gl_InstanceIndex * 2;
	int vid1 = vid0 + 1;
	int vid  = ((gl_VertexIndex & 1) == 0) ? vid0 : vid1;

	vColor = GetColor(vid);
	vSize  = GetPositionSize(vid).w;
	vColor.a *= smoothstep(0.0, 1.0, vSize / kAntialiasing);
	vSize = max(vSize, kAntialiasing);
	vEdgeDistance = vSize * aPosition.y;

	vec4 pos0 = uViewProjMatrix * vec4(GetPositionSize(vid0).xyz, 1.0);
	vec4 pos1 = uViewProjMatrix * vec4(GetPositionSize(vid1).xyz, 1.0);
	vec2 dir  = (pos0.xy / pos0.w) - (pos1.xy / pos1.w);
	dir = normalize(vec2(dir.x, dir.y * uViewport.y / uViewport.x));
	vec2 tng = vec2(-dir.y, dir.x) * vSize / uViewport;

	gl_Position = ((gl_VertexIndex & 1) == 0) ? pos0 : pos1;
	gl_Position.xy += tng * aPosition.y * gl_Position.w;

#elif defined(TRIANGLES)
	int vid = gl_InstanceIndex * 3 + gl_VertexIndex;
	vColor = GetColor(vid);
	vSize  = 0.0;
	gl_Position = uViewProjMatrix * vec4(GetPositionSize(vid).xyz, 1.0);
#endif
}
#endif // VERTEX_SHADER

#ifdef FRAGMENT_SHADER
#if   defined(POINTS)
	layout(location = 0) noperspective in vec2 vUv;
#elif defined(LINES)
	layout(location = 0) noperspective in float vEdgeDistance;
#endif
layout(location = 1) noperspective in float vSize;
layout(location = 2) smooth        in vec4  vColor;

layout(location = 0) out vec4 fResult;

void main()
{
	fResult = vColor;
#if   defined(LINES)
	float d = abs(vEdgeDistance) / vSize;
	d = smoothstep(1.0, 1.0 - (kAntialiasing / vSize), d);
	fResult.a *= d;
#elif defined(POINTS)
	float d = length(vUv - vec2(0.5));
	d = smoothstep(0.5, 0.5 - (kAntialiasing / vSize), d);
	fResult.a *= d;
#endif
	if (fResult.a < 0.001) discard;
}
#endif // FRAGMENT_SHADER
