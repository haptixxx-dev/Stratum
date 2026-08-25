#version 450

// ============================================================================
// Shadow cascade depth pass -- no colour
// ============================================================================
//
// Empty on purpose. The pipeline has NO colour targets at all, so there is
// nothing to write; depth is produced by the rasteriser without any help from
// this stage.
//
// It exists only because SDL_CreateGPUGraphicsPipeline requires a fragment
// shader. Vulkan itself would accept VK_NULL_HANDLE here.
// ============================================================================

void main() {}
