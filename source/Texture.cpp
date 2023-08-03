#include "Texture.h"
#include "Vector2.h"
#include <SDL_image.h>

namespace dae
{
	Texture::Texture(SDL_Surface* pSurface) :
		m_pSurface{ pSurface },
		m_pSurfacePixels{ (uint32_t*)pSurface->pixels }
	{
	}

	Texture::~Texture()
	{
		if (m_pSurface)
		{
			SDL_FreeSurface(m_pSurface);
			m_pSurface = nullptr;
		}
	}

	Texture* Texture::LoadFromFile(const std::string& path)
	{
		//TODO
		//Load SDL_Surface using IMG_LOAD
		//Create & Return a new Texture Object (using SDL_Surface)

		return new Texture(IMG_Load(path.c_str()));
	}

	//std::unique_ptr<Texture> Texture::LoadFromFile(const std::string& path)
	//{
	//	SDL_Surface* pSurface = IMG_Load(path.c_str());
	//	if (!pSurface) {
	//		// Handle the loading error appropriately (e.g., log and return nullptr).
	//		return nullptr;
	//	}
	//	return std::make_unique<Texture>(pSurface);
	//}

	ColorRGB Texture::Sample(const Vector2& uv) const
	{
		const size_t x{ static_cast<size_t>(uv.x * m_pSurface->w) };
		const size_t y{ static_cast<size_t>(uv.y * m_pSurface->h) };

		uint8_t r{}, g{}, b{};

		const Uint32 pixel{ m_pSurfacePixels[x + y * m_pSurface->w] };

		SDL_GetRGB(pixel, m_pSurface->format, &r, &g, &b);

		const float clampColorValue{ 1.f / 255.f };

		return ColorRGB{ r * clampColorValue, g * clampColorValue, b * clampColorValue };

	}

	Vector3 Texture::SampleNormal(const Vector2& uv) const
	{
		const size_t x{ static_cast<size_t>(uv.x * m_pSurface->w) };
		const size_t y{ static_cast<size_t>(uv.y * m_pSurface->h) };

		uint8_t r{}, g{}, b{};

		const Uint32 pixel{ m_pSurfacePixels[x + y * m_pSurface->w] };

		SDL_GetRGB(pixel, m_pSurface->format, &r, &g, &b);

		const float clampColorValue{ 1.f / 255.f };

		return Vector3{ r * clampColorValue, g * clampColorValue, b * clampColorValue };

	}
}