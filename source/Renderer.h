#pragma once

#include <cstdint>
#include <vector>

#include "Camera.h"
#include "DataTypes.h"

struct SDL_Window;
struct SDL_Surface;

namespace dae
{
	class Texture;
	struct Mesh;
	struct Vertex;
	class Timer;
	class Scene;

	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer();

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Update(Timer* pTimer);
		void Render();



		bool SaveBufferToImage() const;



		void NextRenderMode();
		void ToggleRotation();
		void ToggleNormalMap();
		void NextShadingMode();


	private:


		enum class RenderMode
		{
			Default, Depth, END
		};

		enum class ShadingMode
		{
			ObservedArea,
			Diffuse,
			Specular,
			Combined,
			END
		};

		RenderMode m_RenderMode{ RenderMode::Default };
		ShadingMode m_ShadingMode{ ShadingMode::Combined };

		bool m_UseNormalMaps{ true };
		bool m_RotateMesh{ true };

		SDL_Window* m_pWindow{};

		SDL_Surface* m_pFrontBuffer{ nullptr };
		SDL_Surface* m_pBackBuffer{ nullptr };

		uint32_t* m_pBackBufferPixels{};
		float* m_pDepthBufferPixels{};

		Camera m_Camera{};

		int m_Width{};
		int m_Height{};

		float m_AspectRatio{};

		std::vector<Mesh> m_Meshes{};
		Texture* m_pDiffuseTexture{ nullptr };
		Texture* m_pNormalTexture{ nullptr };
		Texture* m_pSpecularTexture{ nullptr };
		Texture* m_pGlossinessTexture{ nullptr };

		Vector3 m_LightDirection{ 0.577f, -0.577f, 0.577f };

		//Function that transforms the vertices from the mesh from World space to Screen space
		void VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex>& vertices_out) const; //W1 Version
		void VertexTransformationFunction(Mesh& mesh) const;


		void ResetDepthBufferAndClearBackground();

		void RenderW1();
		void RenderW2();
		void RenderW3();

		void RenderMeshTriangle(const Mesh& mesh, const std::vector<Vector2>& verticesScreen, size_t currentVertexIdx, bool swapVertices = false);


		void PixelShading(const Vertex_Out& v);

	};
}
