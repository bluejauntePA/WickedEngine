#pragma once
#include "wiRenderPath3D.h"
#include "wiVector.h"

namespace wi
{

	class RenderPath3D_PathTracing : public RenderPath3D
	{
	protected:
		mutable int sam = -1;
		int target = 1024;
		int sampleBatchSize = 1;
		wi::graphics::Texture traceResult;
		wi::graphics::Texture traceDepth;
		wi::graphics::Texture traceStencil;

		wi::vector<uint8_t> texturedata_src;
		wi::vector<uint8_t> texturedata_dst;
		wi::vector<uint8_t> texturedata_albedo;
		wi::vector<uint8_t> texturedata_normal;
		wi::graphics::Texture denoiserAlbedo;
		wi::graphics::Texture denoiserNormal;
		wi::graphics::Texture denoiserResult;
		wi::jobsystem::context denoiserContext;

		void ResizeBuffers() override;

	public:

		~RenderPath3D_PathTracing();

		void Update(float dt) override;
		void Render() const override;
		void Compose(wi::graphics::CommandList cmd) const override;

		int getCurrentSampleCount() const { return sam; }
		void setTargetSampleCount(int value) { target = value; }
		void setSampleBatchSize(int value) { sampleBatchSize = std::max(1, value); }
		float getProgress() const { return (float)sam / (float)target; }
		void setDrawEnvironment(bool value) { drawEnvironment = value; }
		bool getDrawEnvironment() const { return drawEnvironment; }
		void setVisualizeTransparentEnvironment(bool value) { visualizeTransparentEnvironment = value; }
		void setReadbackEnabled(bool value) { readbackEnabled = value; }

		float denoiserProgress = 0;
		float getDenoiserProgress() const { return denoiserProgress; }
		bool isDenoiserAvailable() const;

		void resetProgress() { wi::jobsystem::Wait(denoiserContext); sam = -1; denoiserProgress = 0; volumetriccloudResources.ResetFrame(); }

		uint8_t instanceInclusionMask_PathTrace = 0xFF;

		// This is an identifier of RenderPath subtype that is used for lua binding.
		static constexpr const auto script_check_identifier = relative_path_storage(__FILE__);
		const char* GetScriptBindingID() const override { return script_check_identifier.c_str(); }

	private:
		bool drawEnvironment = true;
		bool visualizeTransparentEnvironment = false;
		bool readbackEnabled = true;
	};

}
