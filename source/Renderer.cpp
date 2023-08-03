//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Texture.h"
#include "Utils.h"
#include <algorithm>
#include <iostream>

using namespace dae;

Renderer::Renderer(SDL_Window* pWindow) :
	m_pWindow(pWindow)
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);

	//Create Buffers
	m_pFrontBuffer = SDL_GetWindowSurface(pWindow);
	m_pBackBuffer = SDL_CreateRGBSurface(0, m_Width, m_Height, 32, 0, 0, 0, 0);
	m_pBackBufferPixels = (uint32_t*)m_pBackBuffer->pixels;

	const int nrPixels{ m_Width * m_Height };
	m_pDepthBufferPixels = new float[nrPixels];
	std::fill_n(m_pDepthBufferPixels, nrPixels, FLT_MAX);

	m_AspectRatio = static_cast<float>(m_Width) / m_Height;


	//Initialize Camera
	m_Camera.Initialize(60.f, { .0f,.0f, 0.f }, m_AspectRatio);


	//Initialize Textures
	m_pDiffuseTexture = Texture::LoadFromFile("Resources/vehicle_diffuse.png");
	m_pSpecularTexture = Texture::LoadFromFile("Resources/vehicle_specular.png");
	m_pGlossinessTexture = Texture::LoadFromFile("Resources/vehicle_gloss.png");
	m_pNormalTexture = Texture::LoadFromFile("Resources/vehicle_normal.png");


	//Initilaize Mesh
	Mesh mesh{ {},{}, PrimitiveTopology::TriangleList };

	Utils::ParseOBJ("Resources/vehicle.obj", mesh.vertices, mesh.indices);
	mesh.worldMatrix = Matrix::CreateTranslation(0.f, 0.f, 50.f) * mesh.worldMatrix;
	m_Meshes.emplace_back(mesh);
}

Renderer::~Renderer()
{
	delete[] m_pDepthBufferPixels;

	delete m_pDiffuseTexture;

	delete m_pNormalTexture;

	delete m_pSpecularTexture;

	delete m_pGlossinessTexture;

}

void Renderer::Update(Timer* pTimer)
{
	m_Camera.Update(pTimer);

	if (m_RotateMesh)
	{
		for (auto& mesh : m_Meshes)
		{
			float rotationSpeedRadian = 1.f;
			mesh.worldMatrix = Matrix::CreateRotationY(rotationSpeedRadian * pTimer->GetElapsed()) * mesh.worldMatrix;
		}
	}
}

void dae::Renderer::Render()
{

	ResetDepthBufferAndClearBackground();
	SDL_LockSurface(m_pBackBuffer);


	//RenderW1();
	//RenderW2();
	RenderW3();

	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);

}

void Renderer::RenderW1()
{
	//Triangles in World Space
	std::vector<Vertex> verticesWorld
	{
		//Triangle 1
		Vertex{Vector3{0.f, 2.f, 0.f}, ColorRGB{ 1.f, 0.f, 0.f } },
		Vertex{ Vector3{1.5f, -1.f, 0.f}, ColorRGB{1.f, 0.f, 0.f} },
		Vertex{ Vector3{-1.5f, -1.f, 0.f}, ColorRGB{1.f, 0.f, 0.f} },

		//Triangle 2
		Vertex{ Vector3{0.f, 4.f, 2.f}, ColorRGB{1.f, 0.f, 0.f} },
		Vertex{ Vector3{3.f, -2.f, 2.f}, ColorRGB{0.f, 1.f, 0.f} },
		Vertex{ Vector3{-3.f, -2.f, 2.f}, ColorRGB{0.f, 0.f, 1.f} }
	};


	//World Space -> NDC
	std::vector<Vertex> verticesNDC;
	VertexTransformationFunction(verticesWorld, verticesNDC);

	//NDC -> Screen Space
	std::vector<Vector2> verticesScreen;
	for (const auto& vertexNdc : verticesNDC)
	{
		verticesScreen.emplace_back( Vector2{ (vertexNdc.position.x + 1.f)/2.f * m_Width, (1.f - vertexNdc.position.y) / 2.f * m_Height});
	}

	const Vector2 screenVector{ static_cast<float>(m_Width), static_cast<float>(m_Height) };

	for (size_t triangleIdx{}; triangleIdx < verticesScreen.size(); triangleIdx += 3)
	{
		
		const Vector2 v0{ verticesScreen[triangleIdx] };
		const Vector2 v1{ verticesScreen[triangleIdx + 1] };
		const Vector2 v2{ verticesScreen[triangleIdx + 2] };


		//Bounding Box - Optimization
		Vector2 minBoundingBox{ Vector2::Min(v0, Vector2::Min(v1, v2)) };
		Vector2 maxBoundingBox{ Vector2::Max(v0, Vector2::Max(v1, v2)) };

		minBoundingBox = Vector2::Max(Vector2::Zero, Vector2::Min(minBoundingBox, screenVector));
		maxBoundingBox = Vector2::Max(Vector2::Zero, Vector2::Min(maxBoundingBox, screenVector));


		for (int px{ static_cast<int>(minBoundingBox.x) }; px < maxBoundingBox.x; ++px)
		{
			for (int py{ static_cast<int>(minBoundingBox.y) }; py < maxBoundingBox.y; ++py)
			{
				const int pixelIdx{ px + py * m_Width };
				const Vector2 currentPixel{ static_cast<float>(px),static_cast<float>(py) };

				//If pixel is in triangle
				if (GeometryUtils::IsInTriangle(currentPixel, v0, v1, v2))
				{

					const float invTriangleArea{ 1.f / Vector2::Cross(v1 - v0,v2 - v0) };

					//Weight 
					float weight0, weight1, weight2;
					weight0 = Vector2::Cross((currentPixel - v1), (v1 - v2)) * invTriangleArea;
					weight1 = Vector2::Cross((currentPixel - v2), (v2 - v0)) * invTriangleArea;
					weight2 = Vector2::Cross((currentPixel - v0), (v0 - v1)) * invTriangleArea;

	

					//Depth
					const float depthWeight
					{
						(verticesWorld[triangleIdx].position.z - m_Camera.origin.z) * weight0 +
						(verticesWorld[triangleIdx + 1].position.z - m_Camera.origin.z) * weight1 +
						(verticesWorld[triangleIdx + 2].position.z - m_Camera.origin.z) * weight2
					};

					if (m_pDepthBufferPixels[pixelIdx] < depthWeight) continue;
					m_pDepthBufferPixels[pixelIdx] = depthWeight;


					//Color
					ColorRGB finalColor =
					{
						verticesWorld[triangleIdx].color * weight0 +
						verticesWorld[triangleIdx + 1].color * weight1 +
						verticesWorld[triangleIdx + 2].color * weight2
					};

					//Update Color in Buffer
					finalColor.MaxToOne();

					m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
						static_cast<uint8_t>(finalColor.r * 255),
						static_cast<uint8_t>(finalColor.g * 255),
						static_cast<uint8_t>(finalColor.b * 255));



				}
			}
		}


	}
	
	//@END
	//Update SDL Surface
	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::RenderW2()
{
	std::vector<Mesh> meshesWorld
	{
		Mesh
		{
			{
				Vertex{Vector3{-3.f, 3.f, -2.f}, colors::White, Vector2{ 0.f, 0.f }},
				Vertex{ Vector3{0.f, 3.f, -2.f}, colors::White, Vector2{0.5f, 0.f} },
				Vertex{ Vector3{3.f, 3.f, -2.f}, colors::White, Vector2{1.f, 0.f} },
				Vertex{ Vector3{-3.f, 0.f, -2.f}, colors::White, Vector2{0.f, 0.5f} },
				Vertex{ Vector3{0.f, 0.f, -2.f}, colors::White, Vector2{0.5f, 0.5f} },
				Vertex{ Vector3{3.f, 0.f, -2.f}, colors::White, Vector2{1.f, 0.5f} },
				Vertex{ Vector3{-3.f, -3.f, -2.f}, colors::White, Vector2{0.f, 1.f} },
				Vertex{ Vector3{0.f, -3.f, -2.f}, colors::White, Vector2{0.5f, 1.f} },
				Vertex{ Vector3{3.f, -3.f, -2.f}, colors::White, Vector2{1.f, 1.f} }
			},
			{
				3, 0, 4, 1, 5, 2,
				2, 6,
				6, 3, 7, 4, 8, 5

				//TriangleList Indices
				/*3, 0, 1,
				1, 4, 3,
				4, 1, 2,
				2, 5, 4,
				6, 3, 4,
				4, 7, 6,
				7, 4, 5,
				5, 8, 7*/

			},
			PrimitiveTopology::TriangleStrip

		}
	};



	for (auto& mesh : meshesWorld)
	{

		//World Space -> NDC
		VertexTransformationFunction(mesh);

		//NDC -> Screen Space

		std::vector<Vector2> verticesScreen;
		verticesScreen.reserve(mesh.vertices_out.size());
		for (const auto& vertexNdc : mesh.vertices_out)
		{
			verticesScreen.emplace_back(Vector2{ (vertexNdc.position.x + 1.f) / 2.f * m_Width, (1.f - vertexNdc.position.y) / 2.f * m_Height });
		};
		



		switch (mesh.primitiveTopology)
		{
		case PrimitiveTopology::TriangleStrip:
			for (size_t vertIdx{}; vertIdx < mesh.indices.size() - 2; ++vertIdx)
			{
				RenderMeshTriangle(mesh, verticesScreen, vertIdx, vertIdx & 1);
			}
			break;
		case PrimitiveTopology::TriangleList:
			for (size_t vertIdx{}; vertIdx < mesh.indices.size(); vertIdx += 3)
			{
				RenderMeshTriangle(mesh, verticesScreen, vertIdx, false);
			}
			break;
		}
	}




}

void dae::Renderer::RenderW3()
{

	for (auto& mesh : m_Meshes)
	{

		//World Space -> NDC
		VertexTransformationFunction(mesh);

		//NDC -> Screen Space

		std::vector<Vector2> verticesScreen;
		verticesScreen.reserve(mesh.vertices_out.size());
		for (const auto& vertexNdc : mesh.vertices_out)
		{
			verticesScreen.emplace_back(Vector2{ (vertexNdc.position.x + 1.f) / 2.f * m_Width, (1.f - vertexNdc.position.y) / 2.f * m_Height });
		};




		switch (mesh.primitiveTopology)
		{
		case PrimitiveTopology::TriangleStrip:
			for (size_t vertIdx{}; vertIdx < mesh.indices.size() - 2; ++vertIdx)
			{
				RenderMeshTriangle(mesh, verticesScreen, vertIdx, vertIdx & 1);
			}
			break;
		case PrimitiveTopology::TriangleList:
			for (size_t vertIdx{}; vertIdx < mesh.indices.size(); vertIdx += 3)
			{
				RenderMeshTriangle(mesh, verticesScreen, vertIdx, false);
			}
			break;
		}
	}

}

void dae::Renderer::RenderMeshTriangle(const Mesh& mesh, const std::vector<Vector2>& verticesScreen, size_t currentVertexIdx, bool swapVertices)
{

	const Vector2 screenVector{ static_cast<float>(m_Width), static_cast<float>(m_Height) };


	const size_t vertIdx0{ mesh.indices[currentVertexIdx + (2 * swapVertices)] };
	const size_t vertIdx1{ mesh.indices[currentVertexIdx + 1] };
	const size_t vertIdx2{ mesh.indices[currentVertexIdx + (!swapVertices * 2)] };

	if (vertIdx0 == vertIdx1 || vertIdx1 == vertIdx2 || vertIdx2 == vertIdx0)
		return;
	

	if (!GeometryUtils::IsVertexInFrustrum(mesh.vertices_out[vertIdx0].position)
		|| !GeometryUtils::IsVertexInFrustrum(mesh.vertices_out[vertIdx1].position)
		|| !GeometryUtils::IsVertexInFrustrum(mesh.vertices_out[vertIdx2].position))
		return;



	const Vector2 v0{ verticesScreen[vertIdx0] };
	const Vector2 v1{ verticesScreen[vertIdx1] };
	const Vector2 v2{ verticesScreen[vertIdx2] };


	//Bounding Box - Optimization
	Vector2 minBoundingBox{ Vector2::Min(v0, Vector2::Min(v1, v2)) };
	Vector2 maxBoundingBox{ Vector2::Max(v0, Vector2::Max(v1, v2)) };

	minBoundingBox = Vector2::Max(Vector2::Zero, Vector2::Min(minBoundingBox, screenVector));
	maxBoundingBox = Vector2::Max(Vector2::Zero, Vector2::Min(maxBoundingBox, screenVector));


	for (int px{ static_cast<int>(minBoundingBox.x) }; px < maxBoundingBox.x; ++px)
	{
		for (int py{ static_cast<int>(minBoundingBox.y) }; py < maxBoundingBox.y; ++py)
		{
			const int pixelIdx{ px + py * m_Width };
			const Vector2 currentPixel{ static_cast<float>(px),static_cast<float>(py) };

			//If pixel is in triangle
			if (GeometryUtils::IsInTriangle(currentPixel, v0, v1, v2))
			{

				const float invTriangleArea{ 1.f / Vector2::Cross(v1 - v0,v2 - v0) };

				//Weight 
				float weight0, weight1, weight2;
				weight0 = Vector2::Cross((currentPixel - v1), (v1 - v2)) * invTriangleArea;
				weight1 = Vector2::Cross((currentPixel - v2), (v2 - v0)) * invTriangleArea;
				weight2 = Vector2::Cross((currentPixel - v0), (v0 - v1)) * invTriangleArea;



				const float depth0{ mesh.vertices_out[vertIdx0].position.z };
				const float depth1{ mesh.vertices_out[vertIdx1].position.z };
				const float depth2{ mesh.vertices_out[vertIdx2].position.z };

				const float interpolatedDepth{ 1.f / 
					(weight0 * (1.f / depth0) + 
					weight1 * (1.f / depth1) + 
					weight2 * (1.f / depth2)) };

				if (m_pDepthBufferPixels[pixelIdx] <= interpolatedDepth || interpolatedDepth < 0.f || interpolatedDepth > 1.f) continue;
				m_pDepthBufferPixels[pixelIdx] = interpolatedDepth;


				Vertex_Out pixel{};

				pixel.position = { currentPixel.x,currentPixel.y, interpolatedDepth,interpolatedDepth };

				pixel.uv = interpolatedDepth *
					((weight0 * mesh.vertices_out[vertIdx0].uv) / depth0 +
					(weight1 * mesh.vertices_out[vertIdx1].uv) / depth1 +
					(weight2 * mesh.vertices_out[vertIdx2].uv) / depth2);


				pixel.normal = Vector3{ interpolatedDepth * 
					(weight0 * mesh.vertices_out[vertIdx0].normal / mesh.vertices_out[vertIdx0].position.w + 
					weight1 * mesh.vertices_out[vertIdx1].normal / mesh.vertices_out[vertIdx1].position.w + 
					weight2 * mesh.vertices_out[vertIdx2].normal / mesh.vertices_out[vertIdx2].position.w) }.Normalized();


				pixel.tangent = Vector3{ interpolatedDepth * 
					(weight0 * mesh.vertices_out[vertIdx0].tangent / mesh.vertices_out[vertIdx0].position.w + 
					weight1 * mesh.vertices_out[vertIdx1].tangent / mesh.vertices_out[vertIdx1].position.w + 
					weight2 * mesh.vertices_out[vertIdx2].tangent / mesh.vertices_out[vertIdx2].position.w) }.Normalized();

				pixel.viewDirection = Vector3{ interpolatedDepth * 
					(weight0 * mesh.vertices_out[vertIdx0].viewDirection / mesh.vertices_out[vertIdx0].position.w +
					weight1 * mesh.vertices_out[vertIdx1].viewDirection / mesh.vertices_out[vertIdx1].position.w + 
					weight2 * mesh.vertices_out[vertIdx2].viewDirection / mesh.vertices_out[vertIdx2].position.w) }.Normalized();

				PixelShading(pixel);
			}
		}
	}
}

void dae::Renderer::PixelShading(const Vertex_Out& v)
{

	//Color
	ColorRGB finalColor{};


	const float lightIntensity{ 7.f };
	const float kd{ 1.f };
	const float shininess{ 25.f };
	Vector3 sampledNormal{ v.normal };
	const ColorRGB ambient{ 0.025f, 0.025f, 0.025f };



	if (m_UseNormalMaps)
	{
		const Vector3 binormal{ Vector3::Cross(v.normal, v.tangent) };
		const Matrix tangentSpaceAxis{ v.tangent, binormal, v.normal, Vector3::Zero };

		Vector3 sampledNormal = m_pNormalTexture->SampleNormal(v.uv);

		//Vector3 sampledNormal = Vector3{ sampledNormalRGB.r, sampledNormalRGB.b, sampledNormalRGB.g };

		sampledNormal = (2.f * sampledNormal) - Vector3{ 1.f, 1.f, 1.f };
		sampledNormal = tangentSpaceAxis.TransformVector(sampledNormal);
		sampledNormal.Normalize();

	}



	switch (m_RenderMode)
	{
	case dae::Renderer::RenderMode::Default:
	{

		const float observedArea{ std::max(0.0f, Vector3::Dot(sampledNormal, -m_LightDirection)) };

		const float exp{ shininess * m_pGlossinessTexture->Sample(v.uv).r };
		const ColorRGB specular{ m_pSpecularTexture->Sample(v.uv) * BRDF::Phong(1.0f, exp, -m_LightDirection, v.viewDirection, sampledNormal) };

		const ColorRGB diffuse{ BRDF::Lambert(kd, m_pDiffuseTexture->Sample(v.uv)) * lightIntensity };


		switch (m_ShadingMode)
		{

			case ShadingMode::ObservedArea:
			{
				finalColor = ColorRGB{ observedArea, observedArea, observedArea };
				break;
			}
			case ShadingMode::Diffuse:
			{
				finalColor = diffuse * observedArea;
				break;
			}
			case ShadingMode::Specular:
			{
				finalColor = specular * observedArea;
				break;
			}
			case ShadingMode::Combined:
			{
				finalColor = (diffuse + specular + ambient) * observedArea;
				break;
			}
		}

		break;
	}
	case dae::Renderer::RenderMode::Depth:
	{
		const float depthCol{ Remap(v.position.w , 0.985f,1.f) };
		finalColor = { depthCol,depthCol,depthCol };
		break;
	}


	}





	//Update Color in Buffer
	finalColor.MaxToOne();

	const int px{ static_cast<int>(v.position.x) };
	const int py{ static_cast<int>(v.position.y) };

	m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));




}

void Renderer::VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex>& vertices_out) const
{
	vertices_out.reserve(vertices_in.size());

	for (const auto& vertexIn : vertices_in)
	{
		Vertex vertexOut{};
		vertexOut.position = m_Camera.invViewMatrix.TransformPoint(vertexIn.position);

		vertexOut.position.x = vertexOut.position.x / vertexOut.position.z / (m_Camera.fov * m_AspectRatio);
		vertexOut.position.y = vertexOut.position.y / vertexOut.position.z / (m_Camera.fov);
		vertices_out.emplace_back(vertexOut);
	}
}

void Renderer::VertexTransformationFunction(Mesh& mesh) const
{
	mesh.vertices_out.clear();
	mesh.vertices_out.reserve(mesh.vertices.size());

	const Matrix worldViewProjectionMatrix{ mesh.worldMatrix * m_Camera.viewMatrix * m_Camera.projectionMatrix };

	for (const auto& vIn : mesh.vertices)
	{
		Vertex_Out vOut{ Vector4{ vIn.position, 1.f}, vIn.color, vIn.uv , vIn.normal, vIn.tangent, vIn.viewDirection };

		vOut.position = worldViewProjectionMatrix.TransformPoint({ vIn.position, 1.0f });

		const float invVInW{ 1.f / vOut.position.w };


		vOut.position.x *= invVInW;
		vOut.position.y *= invVInW;
		vOut.position.z *= invVInW;
		vOut.normal = mesh.worldMatrix.TransformVector(vIn.normal);
		vOut.tangent = mesh.worldMatrix.TransformVector(vIn.tangent);
		vOut.viewDirection = (mesh.worldMatrix.TransformPoint(vIn.position) - m_Camera.origin);
		mesh.vertices_out.emplace_back(vOut);
	}
}

void dae::Renderer::ResetDepthBufferAndClearBackground()
{
	std::fill_n(m_pDepthBufferPixels, (m_Width * m_Height), FLT_MAX);

	SDL_FillRect(m_pBackBuffer, NULL, SDL_MapRGB(m_pBackBuffer->format, 100, 100, 100));
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBackBuffer, "Rasterizer_ColorBuffer.bmp");
}

void dae::Renderer::NextRenderMode()
{
	m_RenderMode = static_cast<RenderMode>((static_cast<int>(m_RenderMode) + 1) % (static_cast<int>(RenderMode::END)));

	std::cout << "Render Mode:  ";

	if (m_RenderMode == Renderer::RenderMode::Default)
		std::cout << "Default" << '\n';
	else
		std::cout << "Depth" << '\n';
}

void dae::Renderer::ToggleRotation()
{
	m_RotateMesh = !m_RotateMesh;
	std::cout << "Rotating: ";

	if (m_RotateMesh)
		std::cout << "enabled" << '\n';
	else
		std::cout << "disabled" << '\n';
}

void dae::Renderer::ToggleNormalMap()
{
	m_UseNormalMaps = !m_UseNormalMaps;
	std::cout << "Normal Maps: ";

	if (m_UseNormalMaps)
		std::cout << "enabled" << '\n';
	else
		std::cout << "disabled" << '\n';
}

void dae::Renderer::NextShadingMode()
{
	m_ShadingMode = static_cast<ShadingMode>((static_cast<int>(m_ShadingMode) + 1) % (static_cast<int>(ShadingMode::END)));


	std::cout << "Shading Mode: ";
	switch (m_ShadingMode)
	{
	case dae::Renderer::ShadingMode::ObservedArea:
		std::cout << "ObservedArea" << '\n';
		break;
	case dae::Renderer::ShadingMode::Diffuse:
		std::cout << "Diffuse" << '\n';
		break;
	case dae::Renderer::ShadingMode::Specular:
		std::cout << "Specular" << '\n';
		break;
	case dae::Renderer::ShadingMode::Combined:
		std::cout << "Combined" << '\n';
		break;
	}
}
